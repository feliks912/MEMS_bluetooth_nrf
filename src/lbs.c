/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief LED Button Service (LBS) sample
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/sys/byteorder.h>

#include "lbs.h"
#include "flashio.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_logger, LOG_LEVEL_DBG);

#ifndef DEVICE_NAME
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME

#ifndef DEVICE_NAME_LEN
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#endif

#endif

#define DEFAULT_TRANSMIT_POWER 0 // in dB
#define LOG_BUFFER_SIZE 256
#define SENSOR_DATA_BUFFER_SIZE 512
#define DEFAULT_RESPONSE_TIMEOUT_MS 1000 // in ms
#define DEFAULT_SENSOR_DATA_CLEAR_ON_READ false
#define DEFAULT_ADV_INT_G_MS (60 * 1000) // global advertising interval in ms
#define DEFAULT_ADV_INT_L_MS 100		 // local advertising interval in ms
#define DEFAULT_ADV_DUR_MS 500			 // advertising duration in ms

static K_MUTEX_DEFINE(g_app_config_mutex);
static K_MUTEX_DEFINE(g_app_runtime_state_mutex);

static K_MUTEX_DEFINE(uptime_at_sync_ms_mutex);
static uint64_t uptime_at_sync_ms = 0;

static K_MUTEX_DEFINE(sensor_data_read_in_progress_mutex);
static bool sensor_data_read_in_progress = false;

typedef struct
{
	char device_name[DEVICE_NAME_LEN];
	char firmware_rev_str[20];
	int8_t transmit_power;
	uint64_t sync_unix_time_ms;
	uint16_t response_timeout_ms;
	uint16_t adv_int_g_ms;
	uint16_t adv_int_l_ms;
	uint16_t adv_dur_ms;
} app_config_t;

typedef struct
{
	uint8_t battery_level;
	uint8_t memory_allocation_perc;
	uint8_t log_data[LOG_BUFFER_SIZE];
	uint16_t device_log_len;
	uint8_t sensor_data[SENSOR_DATA_BUFFER_SIZE];
	uint16_t sensor_data_len;
} app_runtime_state_t;

static const app_config_t DEFAULT_APP_CONFIG = {
	.transmit_power = DEFAULT_TRANSMIT_POWER,
	.sync_unix_time_ms = 0,
	.response_timeout_ms = DEFAULT_RESPONSE_TIMEOUT_MS,
	.adv_int_g_ms = DEFAULT_ADV_INT_G_MS,
	.adv_int_l_ms = DEFAULT_ADV_INT_L_MS,
	.adv_dur_ms = DEFAULT_ADV_DUR_MS,
};

struct disconnect_work
{
	struct k_work work;
	struct bt_conn *conn; // Store the connection object
};

static struct disconnect_work m_disconnect_work;

static app_config_t g_app_config = DEFAULT_APP_CONFIG;
static app_runtime_state_t g_app_runtime_state;

static void disconnect_work_handler(struct k_work *work);
int restore_char_values(void);
static ssize_t read_battery_level(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_memory_aloc_perc(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_device_log(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_device_log_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_sensor_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_sensor_data_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_adv_dur_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_adv_int_g_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_adv_int_l_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_response_timeout_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_transmit_power(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t write_data_clear_disconnect(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t write_adv_dur_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t write_adv_int_g_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t write_adv_int_l_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t write_response_timeout_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t write_transmit_power(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t write_sync_unix_time_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
void set_transmit_power(int8_t power);
int8_t get_transmit_power_level(void);
void set_sync_unix_time_ms(uint64_t time);
uint64_t get_sync_unix_time_ms(void);
void set_response_timeout_ms(uint16_t timeout_ms);
uint16_t get_response_timeout_ms(void);
void set_adv_int_g_ms(uint16_t interval_ms);
uint16_t get_adv_int_g_ms(void);
void set_adv_int_l_ms(uint16_t interval_ms);
uint16_t get_adv_int_l_ms(void);
void set_adv_dur_ms(uint16_t duration_ms);
uint16_t get_adv_dur_ms(void);
void set_device_name(const char *name);
const char *get_device_name(void);
void set_firmware_rev_str(const char *rev_str);
const char *get_firmware_rev_str(void);
void set_battery_level(int8_t level);
uint8_t get_battery_level(void);
void set_device_log(const uint8_t *log_data, uint16_t len);
const uint8_t *get_device_log(uint16_t *len_out);
uint16_t get_device_log_len(void);
void set_memory_allocation_perc(uint8_t perc);
uint8_t get_memory_allocation_perc(void);
void set_sensor_data_length(uint32_t length);
uint32_t get_sensor_data_length(void);
void set_sensor_data(const uint8_t *data, uint16_t len);
const uint8_t *get_sensor_data(uint16_t *len_out);
void set_uptime_at_sync_ms(uint64_t uptime);
uint64_t get_uptime_at_sync_ms(void);
void set_sensor_data_read_in_progress(bool state);
bool get_sensor_data_read_in_progress(void);

static void disconnect_work_handler(struct k_work *work)
{
	// If you used a custom struct like 'disconnect_work'
	struct disconnect_work *disconnect_w = CONTAINER_OF(work, struct disconnect_work, work);
	LOG_INF("Disconnecting connection %p from work queue.", (void *)disconnect_w->conn);
	int err = bt_conn_disconnect(disconnect_w->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	if (err)
	{
		LOG_ERR("Failed to disconnect (err %d)", err);
	}
	bt_conn_unref(disconnect_w->conn);
}

int restore_char_values(void)
{
	int err;
	int ret = 0;

	int8_t transmit_power;
	uint16_t response_timeout_ms;
	uint16_t adv_int_g_ms;
	uint16_t adv_int_l_ms;
	uint16_t adv_dur_ms;

	uint16_t device_log_len;

	err = data_read(FILE_BT_CHAR_TRANSMIT_POWER, &transmit_power, 0, sizeof(transmit_power));
	if (err < 0 || err < sizeof(transmit_power))
	{
		LOG_ERR("Failed to restore transmit power from flash. Err %d", err);
		transmit_power = DEFAULT_TRANSMIT_POWER;
		ret = -1;
	}

	err = data_read(FILE_BT_CHAR_RESPONSE_TIMEOUT, &response_timeout_ms, 0, sizeof(response_timeout_ms));
	if (err < 0 || err < sizeof(response_timeout_ms))
	{
		LOG_ERR("Failed to restore response timeout from flash. Err %d", err);
		response_timeout_ms = DEFAULT_RESPONSE_TIMEOUT_MS;
		ret = -1;
	}

	err = data_read(FILE_BT_CHAR_ADVERTISING_INTERVAL_GLOBAL, &adv_int_g_ms, 0, sizeof(adv_int_g_ms));
	if (err < 0 || err < sizeof(adv_int_g_ms))
	{
		LOG_ERR("Failed to restore global advertising interval from flash. Err %d", err);
		adv_int_g_ms = DEFAULT_ADV_INT_G_MS;
		ret = -1;
	}

	err = data_read(FILE_BT_CHAR_ADVERTISING_INTERVAL_LOCAL, &adv_int_l_ms, 0, sizeof(adv_int_l_ms));
	if (err < 0 || err < sizeof(adv_int_l_ms))
	{
		LOG_ERR("Failed to restore local advertising interval from flash. Err %d", err);
		adv_int_l_ms = DEFAULT_ADV_INT_L_MS;
		ret = -1;
	}

	err = data_read(FILE_BT_CHAR_ADVERTISING_DURATION, &adv_dur_ms, 0, sizeof(adv_dur_ms));
	if (err < 0 || err < sizeof(adv_dur_ms))
	{
		LOG_ERR("Failed to restore advertising duration from flash. Err %d", err);
		adv_dur_ms = DEFAULT_ADV_DUR_MS;
		ret = -1;
	}

	err = data_read(FILE_BT_CHAR_DEVICE_LOG_LENGTH, &device_log_len, 0, sizeof(device_log_len));
	if (err < 0 || err < sizeof(device_log_len))
	{
		LOG_ERR("Failed to restore device log length from flash. Err %d", err);
		adv_dur_ms = 0;
		ret = -1;
	}

	g_app_config.transmit_power = transmit_power;
	g_app_config.response_timeout_ms = response_timeout_ms;
	g_app_config.adv_int_g_ms = adv_int_g_ms;
	g_app_config.adv_int_l_ms = adv_int_l_ms;
	g_app_config.adv_dur_ms = adv_dur_ms;

	g_app_runtime_state.device_log_len = device_log_len;

	return ret; // Indicate success if all attempts were made
}

BT_GATT_SERVICE_DEFINE(app_runtime_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_APP_RUNTIME),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BATTERY_LEVEL, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_battery_level, NULL, &g_app_runtime_state.battery_level),
					   BT_GATT_CHARACTERISTIC(BT_UUID_MEMORY_ALLOCATED_PERCENTAGE, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_memory_aloc_perc, NULL, &g_app_runtime_state.memory_allocation_perc),
					   BT_GATT_CHARACTERISTIC(BT_UUID_DEVICE_LOG, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_device_log, NULL, &g_app_runtime_state.log_data[0]),
					   BT_GATT_CHARACTERISTIC(BT_UUID_DEVICE_LOG_LENGTH, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_device_log_length, NULL, &g_app_runtime_state.device_log_len),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_sensor_data, NULL, &g_app_runtime_state.sensor_data[0]),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA_LENGTH, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_sensor_data_length, NULL, &g_app_runtime_state.sensor_data_len), );

BT_GATT_SERVICE_DEFINE(app_config_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_APP_CONFIG),
					   BT_GATT_CHARACTERISTIC(BT_UUID_TRANSMIT_POWER, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_transmit_power, write_transmit_power, &g_app_config.transmit_power),
					   BT_GATT_CHARACTERISTIC(BT_UUID_CLEAR_FLASH_DATA_DISCONNECT_BIT, BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_data_clear_disconnect, NULL),
					   BT_GATT_CHARACTERISTIC(BT_UUID_UNIX_TIME, BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_sync_unix_time_ms, NULL),
					   BT_GATT_CHARACTERISTIC(BT_UUID_RESPONSE_TIMEOUT, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_response_timeout_ms, write_response_timeout_ms, &g_app_config.response_timeout_ms),
					   BT_GATT_CHARACTERISTIC(BT_UUID_ADVERTISING_INTERVAL_GLOBAL, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_adv_int_g_ms, write_adv_int_g_ms, &g_app_config.adv_int_g_ms),
					   BT_GATT_CHARACTERISTIC(BT_UUID_ADVERTISING_INTERVAL_LOCAL, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_adv_int_l_ms, write_adv_int_l_ms, &g_app_config.adv_int_l_ms),
					   BT_GATT_CHARACTERISTIC(BT_UUID_ADVERTISING_DURATION, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_adv_dur_ms, write_adv_dur_ms, &g_app_config.adv_dur_ms), );

static ssize_t read_battery_level(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								  uint16_t len, uint16_t offset)
{
	uint8_t battery_level = get_battery_level();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Battery Level: %u", attr->handle, (void *)conn, battery_level);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &battery_level, sizeof(battery_level));
}

static ssize_t read_memory_aloc_perc(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									 uint16_t len, uint16_t offset)
{
	uint8_t perc = get_memory_allocation_perc();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Mem Alloc Perc: %u", attr->handle, (void *)conn, perc);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &perc, sizeof(perc));
}

// TODO: Fix this shit
static ssize_t read_device_log(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
							   uint16_t len, uint16_t offset)
{
	const uint8_t *log_data;
	uint16_t log_data_len;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	int mutex_err = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (mutex_err != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", mutex_err);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	// Access protected data
	log_data = g_app_runtime_state.log_data;
	log_data_len = g_app_runtime_state.device_log_len;

	if (offset >= log_data_len)
	{
		LOG_DBG("Read beyond current data length: offset %u >= log_data_len %u", offset, log_data_len);
		k_mutex_unlock(&g_app_runtime_state_mutex);
		return 0;
	}

	uint16_t remaining_bytes = log_data_len - offset;
	uint16_t bytes_to_copy = MIN(remaining_bytes, len);

	memcpy(buf, log_data + offset, bytes_to_copy);

	k_mutex_unlock(&g_app_runtime_state_mutex); // Release mutex

	return bytes_to_copy;
}

static ssize_t read_device_log_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									  uint16_t len, uint16_t offset)
{
	uint16_t length_val = get_device_log_len();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Device Log Len: %u", attr->handle, (void *)conn, length_val);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &length_val, sizeof(length_val));
}

//TODO: Sync this to a sensor data setter?
static ssize_t read_sensor_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								uint16_t len, uint16_t offset)
{

	set_sensor_data_read_in_progress(true);

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	int mutex_err = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (mutex_err != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", mutex_err);
		set_sensor_data_read_in_progress(false);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	// TODO: Add check
	ssize_t bytes_read = data_read(FILE_SENSOR_DATA_LOCATION, buf, offset, len);
	if (bytes_read < 0)
	{
		LOG_ERR("Failed to read sensor data from flash %d", bytes_read);
		set_sensor_data_read_in_progress(false);
		k_mutex_unlock(&g_app_runtime_state_mutex);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	LOG_INF("%d bytes of sensor data read from flash during BLE transfer.", bytes_read);

	k_mutex_unlock(&g_app_runtime_state_mutex);

	return bytes_read;
}

static ssize_t read_sensor_data_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									   uint16_t len, uint16_t offset)
{
	uint32_t length_val = get_sensor_data_length();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Sensor Data Len: %u", attr->handle, (void *)conn, length_val);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &length_val, sizeof(length_val));
}

static ssize_t read_adv_dur_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
							   uint16_t len, uint16_t offset)
{
	uint16_t duration_val = get_adv_dur_ms();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Adv Dur: %u", attr->handle, (void *)conn, duration_val);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &duration_val, sizeof(duration_val));
}

static ssize_t read_adv_int_g_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								 uint16_t len, uint16_t offset)
{
	uint16_t interval_val = get_adv_int_g_ms();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Adv Int Global: %u", attr->handle, (void *)conn, interval_val);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &interval_val, sizeof(interval_val));
}

static ssize_t read_adv_int_l_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								 uint16_t len, uint16_t offset)
{
	uint16_t interval_val = get_adv_int_l_ms();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Adv Int Local: %u", attr->handle, (void *)conn, interval_val);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &interval_val, sizeof(interval_val));
}

static ssize_t read_response_timeout_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
										uint16_t len, uint16_t offset)
{
	uint16_t timeout_val = get_response_timeout_ms();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Response Timeout: %u", attr->handle, (void *)conn, timeout_val);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &timeout_val, sizeof(timeout_val));
}

static ssize_t read_transmit_power(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								   uint16_t len, uint16_t offset)
{
	int8_t power_val = get_transmit_power_level();

	LOG_DBG("Attribute read, handle: %u, conn: %p, Transmit Power: %d", attr->handle, (void *)conn, power_val);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &power_val, sizeof(power_val));
}

// --- Write handlers ---

static ssize_t write_data_clear_disconnect(struct bt_conn *conn,
										   const struct bt_gatt_attr *attr,
										   const void *buf, uint16_t len,
										   uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 2U)
	{
		LOG_DBG("Write data clear disconnect: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0)
	{
		LOG_DBG("Write data clear disconnect: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint8_t val = sys_get_le16(buf);

	if (val == 0)
	{
		LOG_DBG("Write data clear disconnect: Incorrect value %d.", val);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	if (val != get_sensor_data_length())
	{
		LOG_ERR("ERROR\nCentral read incorrect number of sensor data. Sensor data length is %d, central read %d.\nERROR", get_sensor_data_length(), val);
	}

	LOG_DBG("Auto-disconnecting connection %p...", (void *)conn);

	k_work_init(&m_disconnect_work.work, disconnect_work_handler);

	m_disconnect_work.conn = bt_conn_ref(conn); // Increment reference count

	k_work_submit(&m_disconnect_work.work);

	set_sensor_data_read_in_progress(false);

	return len;
}

static ssize_t write_adv_dur_ms(struct bt_conn *conn,
								const struct bt_gatt_attr *attr,
								const void *buf, uint16_t len,
								uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 2U)
	{
		LOG_DBG("Write advertising durations: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0)
	{
		LOG_DBG("Write advertising durations: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint16_t val = sys_get_le16(buf);

	set_adv_dur_ms(val);

	return len;
}

static ssize_t write_adv_int_g_ms(struct bt_conn *conn,
								  const struct bt_gatt_attr *attr,
								  const void *buf, uint16_t len,
								  uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 2U)
	{
		LOG_DBG("Write advertising interval global: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0)
	{
		LOG_DBG("Write advertising interval global: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint16_t val = sys_get_le16(buf);

	set_adv_int_g_ms(val);

	return len;
}

static ssize_t write_adv_int_l_ms(struct bt_conn *conn,
								  const struct bt_gatt_attr *attr,
								  const void *buf, uint16_t len,
								  uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 2U)
	{
		LOG_DBG("Write advertising interval local: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0)
	{
		LOG_DBG("Write advertising interval local: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint16_t val = sys_get_le16(buf);

	set_adv_int_l_ms(val);

	return len;
}

static ssize_t write_response_timeout_ms(struct bt_conn *conn,
										 const struct bt_gatt_attr *attr,
										 const void *buf, uint16_t len,
										 uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 2U)
	{
		LOG_DBG("Write response timeout: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0)
	{
		LOG_DBG("Write response timeout: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint16_t val = sys_get_le16(buf);

	if (val < 0 || val > 1000)
	{
		LOG_DBG("Write response timeout: Invalid value %d. Only [0, 1000] allowed.", val);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	set_response_timeout_ms(val);

	return len;
}

static ssize_t write_transmit_power(struct bt_conn *conn,
									const struct bt_gatt_attr *attr,
									const void *buf, uint16_t len,
									uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 1U)
	{
		LOG_DBG("Write transmit power: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0)
	{
		LOG_DBG("Write transmit power: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	int8_t val = *((int8_t *)buf);

	// TODO: Remove magic numbers here and load from universal config
	if (val != -3 && val != 0 && val != 3 && val != 6 && val != 9)
	{
		LOG_DBG("Write transmit power: Invalid value %d. Only -3, 0, 3, 6, and 9 are allowed.", val);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	set_transmit_power(val);

	return len;
}

static ssize_t write_sync_unix_time_ms(struct bt_conn *conn,
									   const struct bt_gatt_attr *attr,
									   const void *buf, uint16_t len,
									   uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 8U)
	{
		LOG_DBG("Write sync unix time: Incorrect data length %d", len); // Changed log message
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0)
	{
		LOG_DBG("Write sync unix time: Incorrect data offset %d", offset); // Changed log message
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint64_t val = sys_get_le64(buf);

	set_sync_unix_time_ms(val);

	set_uptime_at_sync_ms(k_uptime_get());

	return len;
}

// ------ SETTERS AND GETTERS ------

void set_transmit_power(int8_t power)
{
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return;
	}

	ret = data_overwrite(FILE_BT_CHAR_TRANSMIT_POWER, &power, sizeof(int8_t));
	if (ret < 0)
	{
		LOG_ERR("Failed to store transmit power to flash. Err %d", ret);
	}

	g_app_config.transmit_power = power;
	k_mutex_unlock(&g_app_config_mutex);
}

int8_t get_transmit_power_level(void)
{
	int8_t value;
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return 0;
	}
	value = g_app_config.transmit_power;
	k_mutex_unlock(&g_app_config_mutex);
	return value;
}

void set_sync_unix_time_ms(uint64_t time)
{
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return;
	}
	g_app_config.sync_unix_time_ms = time;
	k_mutex_unlock(&g_app_config_mutex);
}

uint64_t get_sync_unix_time_ms()
{
	uint64_t value;
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return 0;
	}
	value = g_app_config.sync_unix_time_ms;
	k_mutex_unlock(&g_app_config_mutex);
	return value;
}

void set_response_timeout_ms(uint16_t timeout_ms)
{
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return;
	}

	ret = data_overwrite(FILE_BT_CHAR_RESPONSE_TIMEOUT, &timeout_ms, sizeof(uint16_t));
	if (ret < 0)
	{
		LOG_ERR("Failed to store response timeout to flash. Err %d", ret);
	}

	g_app_config.response_timeout_ms = timeout_ms;
	k_mutex_unlock(&g_app_config_mutex);
}

uint16_t get_response_timeout_ms(void)
{
	uint16_t value;
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return 0;
	}
	value = g_app_config.response_timeout_ms;
	k_mutex_unlock(&g_app_config_mutex);
	return value;
}

void set_adv_int_g_ms(uint16_t interval_ms)
{
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return;
	}

	ret = data_overwrite(FILE_BT_CHAR_ADVERTISING_INTERVAL_GLOBAL, &interval_ms, sizeof(uint16_t));
	if (ret < 0)
	{
		LOG_ERR("Failed to store advertising interval global to flash. Err %d", ret);
	}

	g_app_config.adv_int_g_ms = interval_ms;
	k_mutex_unlock(&g_app_config_mutex);
}

uint16_t get_adv_int_g_ms(void)
{
	uint16_t value;
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutexs: %d", ret);
		return 0;
	}
	value = g_app_config.adv_int_g_ms;
	k_mutex_unlock(&g_app_config_mutex);
	return value;
}

void set_adv_int_l_ms(uint16_t interval_ms)
{
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return;
	}

	ret = data_overwrite(FILE_BT_CHAR_ADVERTISING_INTERVAL_LOCAL, &interval_ms, sizeof(uint16_t));
	if (ret < 0)
	{
		LOG_ERR("Failed to store advertising interval local to flash. Err %d", ret);
	}

	g_app_config.adv_int_l_ms = interval_ms;
	k_mutex_unlock(&g_app_config_mutex);
}

uint16_t get_adv_int_l_ms(void)
{
	uint16_t value;
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return 0;
	}
	value = g_app_config.adv_int_l_ms;
	k_mutex_unlock(&g_app_config_mutex);
	return value;
}

void set_adv_dur_ms(uint16_t duration_ms)
{
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return;
	}

	ret = data_overwrite(FILE_BT_CHAR_ADVERTISING_DURATION, &duration_ms, sizeof(uint16_t));
	if (ret < 0)
	{
		LOG_ERR("Failed to store advertising duration to flash. Err %d", ret);
	}

	g_app_config.adv_dur_ms = duration_ms;
	k_mutex_unlock(&g_app_config_mutex);
}

uint16_t get_adv_dur_ms(void)
{
	uint16_t value;
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return 0;
	}
	value = g_app_config.adv_dur_ms;
	k_mutex_unlock(&g_app_config_mutex);
	return value;
}

void set_device_name(const char *name)
{
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return;
	}
	strncpy(g_app_config.device_name, name, DEVICE_NAME_LEN - 1);
	g_app_config.device_name[DEVICE_NAME_LEN - 1] = '\0'; // Ensure null termination
	k_mutex_unlock(&g_app_config_mutex);
}

const char *get_device_name(void)
{
	return g_app_config.device_name;
}

void set_firmware_rev_str(const char *rev_str)
{
	int ret = k_mutex_lock(&g_app_config_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_config_mutex: %d", ret);
		return;
	}
	strncpy(g_app_config.firmware_rev_str, rev_str, sizeof(g_app_config.firmware_rev_str) - 1);
	g_app_config.firmware_rev_str[sizeof(g_app_config.firmware_rev_str) - 1] = '\0';
	k_mutex_unlock(&g_app_config_mutex);
}

const char *get_firmware_rev_str(void)
{
	return g_app_config.firmware_rev_str;
}

void set_battery_level(int8_t level)
{
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return;
	}
	if (level > 100)
	{
		level = 100;
	}
	else if (level < 0)
	{
		level = 0;
	}
	g_app_runtime_state.battery_level = (uint8_t)level;
	k_mutex_unlock(&g_app_runtime_state_mutex);
}

uint8_t get_battery_level(void)
{
	uint8_t value;
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return 0;
	}
	value = g_app_runtime_state.battery_level;
	k_mutex_unlock(&g_app_runtime_state_mutex);
	return value;
}

// TODO: replace set and get device log from flash directly.
void set_device_log(const uint8_t *log_data, uint16_t len)
{
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return;
	}
	uint16_t bytes_to_copy = (len < LOG_BUFFER_SIZE) ? len : LOG_BUFFER_SIZE;
	memcpy(g_app_runtime_state.log_data, log_data, bytes_to_copy);
	g_app_runtime_state.device_log_len = bytes_to_copy;
	k_mutex_unlock(&g_app_runtime_state_mutex);
}

const uint8_t *get_device_log(uint16_t *len_out)
{
	// Similar considerations as get_device_name for returning pointer to shared data.
	// If `len_out` is passed, it's also updated under mutex protection.
	const uint8_t *value;
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		if (len_out != NULL)
			*len_out = 0; // Indicate no data
		return NULL;
	}
	if (len_out != NULL)
	{
		*len_out = g_app_runtime_state.device_log_len;
	}
	value = g_app_runtime_state.log_data;
	k_mutex_unlock(&g_app_runtime_state_mutex);
	return value;
}

uint16_t get_device_log_len(void)
{
	uint16_t value;
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return 0;
	}
	value = g_app_runtime_state.device_log_len;
	k_mutex_unlock(&g_app_runtime_state_mutex);
	return value;
}

void set_memory_allocation_perc(uint8_t perc)
{
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return;
	}
	g_app_runtime_state.memory_allocation_perc = perc;
	k_mutex_unlock(&g_app_runtime_state_mutex);
}

uint8_t get_memory_allocation_perc(void)
{
	uint8_t value;
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return 0;
	}
	value = g_app_runtime_state.memory_allocation_perc;
	k_mutex_unlock(&g_app_runtime_state_mutex);
	return value;
}

void set_sensor_data_length(uint32_t length)
{
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return;
	}
	g_app_runtime_state.sensor_data_len = length;
	k_mutex_unlock(&g_app_runtime_state_mutex);
}

uint32_t get_sensor_data_length(void)
{
	uint32_t value;
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return 0;
	}
	value = g_app_runtime_state.sensor_data_len;
	k_mutex_unlock(&g_app_runtime_state_mutex);
	return value;
}

// ------- RUNTIME VARIABLES GETTERS AND SETTERS --------

void set_sensor_data(const uint8_t *data, uint16_t len)
{
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		return;
	}
	uint16_t bytes_to_copy = (len < SENSOR_DATA_BUFFER_SIZE) ? len : SENSOR_DATA_BUFFER_SIZE;
	memcpy(g_app_runtime_state.sensor_data, data, bytes_to_copy);
	g_app_runtime_state.sensor_data_len = bytes_to_copy;
	k_mutex_unlock(&g_app_runtime_state_mutex);
}

const uint8_t *get_sensor_data(uint16_t *len_out)
{
	const uint8_t *value;
	int ret = k_mutex_lock(&g_app_runtime_state_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock g_app_runtime_state_mutex: %d", ret);
		if (len_out != NULL)
			*len_out = 0;
		return NULL;
	}
	if (len_out != NULL)
	{
		*len_out = g_app_runtime_state.sensor_data_len;
	}
	value = g_app_runtime_state.sensor_data;
	k_mutex_unlock(&g_app_runtime_state_mutex);
	return value;
}

void set_uptime_at_sync_ms(uint64_t uptime)
{
	int mutex_err = k_mutex_lock(&uptime_at_sync_ms_mutex, K_FOREVER);
	if (mutex_err != 0)
	{
		LOG_ERR("Failed to lock uptime_at_sync_ms_mutex: %d", mutex_err);
		return;
	}
	uptime_at_sync_ms = uptime;
	k_mutex_unlock(&uptime_at_sync_ms_mutex);
}

uint64_t get_uptime_at_sync_ms(void)
{
	uint64_t value;
	int ret = k_mutex_lock(&uptime_at_sync_ms_mutex, K_FOREVER);
	if (ret != 0)
	{
		LOG_ERR("Failed to lock uptime_at_sync_ms_mutex: %d", ret);
		return 0;
	}
	value = uptime_at_sync_ms;
	k_mutex_unlock(&uptime_at_sync_ms_mutex);
	return value;
}

void set_sensor_data_read_in_progress(bool state)
{
	int mutex_err = k_mutex_lock(&sensor_data_read_in_progress_mutex, K_FOREVER);
	if (mutex_err < 0)
	{
		LOG_ERR("Failed to lock sensor_data_read_in_progress_mutex: %d", mutex_err);
		return;
	}
	sensor_data_read_in_progress = state;
	k_mutex_unlock(&sensor_data_read_in_progress_mutex);
}

bool get_sensor_data_read_in_progress(void)
{
	int mutex_err = k_mutex_lock(&sensor_data_read_in_progress_mutex, K_FOREVER);
	if (mutex_err < 0)
	{
		LOG_ERR("Failed to lock sensor_data_read_in_progress_mutex: %d", mutex_err);
		return true; // Err on the safe side?
	}
	bool val = sensor_data_read_in_progress;
	k_mutex_unlock(&sensor_data_read_in_progress_mutex);

	return val;
}