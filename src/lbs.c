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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_lbs, LOG_LEVEL_DBG);

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
#define DEFAULT_SENSOR_ODR_HZ 1000
#define DEFAULT_SENSOR_DATA_CLEAR_ON_READ false
#define DEFAULT_ADV_INT_G_MS (60 * 1000) // global advertising interval in ms
#define DEFAULT_ADV_INT_L_MS 100		 // local advertising interval in ms
#define DEFAULT_ADV_DUR_MS 500			 // advertising duration in ms
#define DEFAULT_AUTO_DISCONNECT_BIT false

// BLE synchronized unix time in ms

typedef struct
{
	char device_name[DEVICE_NAME_LEN];
	char firmware_rev_str[20];
	int8_t transmit_power;
	uint64_t sync_unix_time_ms;
	bool auto_disconnect;
	uint16_t response_timeout_ms;
	uint16_t sensor_odr_hz;
	bool sensor_data_clear_on_read;
	uint16_t adv_int_g_ms;
	uint16_t adv_int_l_ms;
	uint16_t adv_dur_ms;
} app_config_t;

typedef struct
{
	uint8_t battery_level;
	uint8_t memory_allocation_perc;
	uint8_t device_log[LOG_BUFFER_SIZE];
	uint16_t device_log_len;
	uint8_t sensor_data[SENSOR_DATA_BUFFER_SIZE];
	uint16_t sensor_data_len;
} app_runtime_state_t;

static const app_config_t DEFAULT_APP_CONFIG = {
	.transmit_power = DEFAULT_TRANSMIT_POWER,
	.auto_disconnect = DEFAULT_AUTO_DISCONNECT_BIT,
	.sync_unix_time_ms = 0,
	.response_timeout_ms = DEFAULT_RESPONSE_TIMEOUT_MS,
	.sensor_odr_hz = DEFAULT_SENSOR_ODR_HZ,
	.sensor_data_clear_on_read = DEFAULT_SENSOR_DATA_CLEAR_ON_READ,
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

static ssize_t read_battery_level(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								  uint16_t len, uint16_t offset);
static ssize_t read_memory_aloc_perc(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									 uint16_t len, uint16_t offset);
static ssize_t read_device_log(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
							   uint16_t len, uint16_t offset);
static ssize_t read_device_log_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									  uint16_t len, uint16_t offset);
static ssize_t read_sensor_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								uint16_t len, uint16_t offset);
static ssize_t read_sensor_data_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									   uint16_t len, uint16_t offset);

static ssize_t read_transmit_power(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								   uint16_t len, uint16_t offset);
static ssize_t write_transmit_power(struct bt_conn *conn,
									const struct bt_gatt_attr *attr,
									const void *buf, uint16_t len,
									uint16_t offset, uint8_t flags);
static ssize_t write_sync_unix_time_ms(struct bt_conn *conn,
								  const struct bt_gatt_attr *attr,
								  const void *buf, uint16_t len,
								  uint16_t offset, uint8_t flags);
static ssize_t read_auto_disconnect(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									uint16_t len, uint16_t offset);
static ssize_t write_auto_disconnect(struct bt_conn *conn,
									 const struct bt_gatt_attr *attr,
									 const void *buf, uint16_t len,
									 uint16_t offset, uint8_t flags);
static ssize_t read_response_timeout_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
										uint16_t len, uint16_t offset);
static ssize_t write_response_timeout_ms(struct bt_conn *conn,
										 const struct bt_gatt_attr *attr,
										 const void *buf, uint16_t len,
										 uint16_t offset, uint8_t flags);
static ssize_t read_sensor_odr(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
							   uint16_t len, uint16_t offset);
static ssize_t write_sensor_odr(struct bt_conn *conn,
								const struct bt_gatt_attr *attr,
								const void *buf, uint16_t len,
								uint16_t offset, uint8_t flags);
static ssize_t read_sensor_data_clear_on_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
											  uint16_t len, uint16_t offset);
static ssize_t write_sensor_data_clear_on_read(struct bt_conn *conn,
											   const struct bt_gatt_attr *attr,
											   const void *buf, uint16_t len,
											   uint16_t offset, uint8_t flags);
static ssize_t read_adv_int_l_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								 uint16_t len, uint16_t offset);
static ssize_t write_adv_int_l_ms(struct bt_conn *conn,
								  const struct bt_gatt_attr *attr,
								  const void *buf, uint16_t len,
								  uint16_t offset, uint8_t flags);
static ssize_t read_adv_int_g_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								 uint16_t len, uint16_t offset);
static ssize_t write_adv_int_g_ms(struct bt_conn *conn,
								  const struct bt_gatt_attr *attr,
								  const void *buf, uint16_t len,
								  uint16_t offset, uint8_t flags);
static ssize_t read_adv_dur_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
							   uint16_t len, uint16_t offset);
static ssize_t write_adv_dur_ms(struct bt_conn *conn,
								const struct bt_gatt_attr *attr,
								const void *buf, uint16_t len,
								uint16_t offset, uint8_t flags);

BT_GATT_SERVICE_DEFINE(app_runtime_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_APP_RUNTIME),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BATTERY_LEVEL, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_battery_level, NULL, &g_app_runtime_state.battery_level),
					   BT_GATT_CHARACTERISTIC(BT_UUID_MEMORY_ALLOCATED_PERCENTAGE, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_memory_aloc_perc, NULL, &g_app_runtime_state.memory_allocation_perc),
					   BT_GATT_CHARACTERISTIC(BT_UUID_DEVICE_LOG, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_device_log, NULL, &g_app_runtime_state.device_log[0]),
					   BT_GATT_CHARACTERISTIC(BT_UUID_DEVICE_LOG_LENGTH, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_device_log_length, NULL, &g_app_runtime_state.device_log_len),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_sensor_data, NULL, &g_app_runtime_state.sensor_data[0]),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA_LENGTH, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_sensor_data_length, NULL, &g_app_runtime_state.sensor_data_len), );

BT_GATT_SERVICE_DEFINE(app_config_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_APP_CONFIG),
					   BT_GATT_CHARACTERISTIC(BT_UUID_TRANSMIT_POWER, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_transmit_power, write_transmit_power, &g_app_config.transmit_power),
					   BT_GATT_CHARACTERISTIC(BT_UUID_UNIX_TIME, BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_sync_unix_time_ms, &g_app_config.sync_unix_time_ms),
					   BT_GATT_CHARACTERISTIC(BT_UUID_AUTO_DISCONNECT, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_auto_disconnect, write_auto_disconnect, &g_app_config.auto_disconnect),
					   BT_GATT_CHARACTERISTIC(BT_UUID_RESPONSE_TIMEOUT, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_response_timeout_ms, write_response_timeout_ms, &g_app_config.response_timeout_ms),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_ODR, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_sensor_odr, write_sensor_odr, &g_app_config.sensor_odr_hz),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA_CLEAR_BIT, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_sensor_data_clear_on_read, write_sensor_data_clear_on_read, &g_app_config.sensor_data_clear_on_read),
					   BT_GATT_CHARACTERISTIC(BT_UUID_ADVERTISING_INTERVAL_GLOBAL, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_adv_int_g_ms, write_adv_int_g_ms, &g_app_config.adv_int_g_ms),
					   BT_GATT_CHARACTERISTIC(BT_UUID_ADVERTISING_INTERVAL_LOCAL, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_adv_int_l_ms, write_adv_int_l_ms, &g_app_config.adv_int_l_ms),
					   BT_GATT_CHARACTERISTIC(BT_UUID_ADVERTISING_DURATION, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_adv_dur_ms, write_adv_dur_ms, &g_app_config.adv_dur_ms), );

static ssize_t read_battery_level(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								  uint16_t len, uint16_t offset)
{
	const uint8_t *data = (const uint8_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static ssize_t read_memory_aloc_perc(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									 uint16_t len, uint16_t offset)
{
	const uint8_t *data = (const uint8_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static uint16_t device_log_chunk_offset = 0;
static ssize_t read_device_log(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
							   uint16_t len, uint16_t offset)
{
	const uint8_t *data = (const uint8_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	/* if (offset >= device_log_chunk_offset)
	{
		LOG_DBG("Read beyond current data length: offset %u >= data_len %u\n", offset, device_log_chunk_offset);
		return 0;
	} */

	uint16_t remaining_bytes = device_log_chunk_offset - offset;

	uint16_t bytes_to_copy = MIN(remaining_bytes, len);

	memcpy(buf, data + offset, bytes_to_copy);

	return bytes_to_copy;
}

static ssize_t read_device_log_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									  uint16_t len, uint16_t offset)
{
	const uint16_t *data = (const uint16_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static uint16_t sensor_data_chunk_offset = 0;
static ssize_t read_sensor_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								uint16_t len, uint16_t offset)
{
	printk("In read sensor data.\n");

	const uint8_t *data = (const uint8_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	/* if (offset >= sensor_data_chunk_offset)
	{
		LOG_DBG("Read beyond current data length: offset %u >= data_len %u\n", offset, sensor_data_chunk_offset);
		return 0;
	}
 */
	uint16_t remaining_bytes = sensor_data_chunk_offset - offset;

	uint16_t bytes_to_copy = MIN(remaining_bytes, len);

	memcpy(buf, data + offset, bytes_to_copy);

	// Defer the disconnection if auto_disconnect is enabled
	if (g_app_config.auto_disconnect)
	{
		LOG_DBG("Scheduling disconnect for connection %p...", (void *)conn);

		k_work_init(&m_disconnect_work.work, disconnect_work_handler);
		m_disconnect_work.conn = bt_conn_ref(conn); // IMPORTANT: Increment reference count
		k_work_submit(&m_disconnect_work.work);
	}
	else
	{
		LOG_DBG("Auto disconnect bit is set to false.");
	}

	return bytes_to_copy;
}

static ssize_t read_sensor_data_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									   uint16_t len, uint16_t offset)
{
	const uint8_t *data = (const uint8_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static ssize_t read_adv_dur_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
							   uint16_t len, uint16_t offset)
{
	const uint16_t *data = (const uint16_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
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

	// TODO: Add value check

	g_app_config.adv_dur_ms = sys_get_le16(buf);

	return len;
}

static ssize_t read_adv_int_g_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								 uint16_t len, uint16_t offset)
{
	const uint16_t *data = (const uint16_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
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

	// TODO: Add value check

	g_app_config.adv_int_g_ms = sys_get_le16(buf);

	return len;
}

static ssize_t read_adv_int_l_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								 uint16_t len, uint16_t offset)
{
	const uint16_t *data = (const uint16_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
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

	// TODO: Add value check

	g_app_config.adv_int_l_ms = sys_get_le16(buf);

	return len;
}

static ssize_t read_sensor_data_clear_on_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
											  uint16_t len, uint16_t offset)
{
	const bool *data = (const bool *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static ssize_t write_sensor_data_clear_on_read(struct bt_conn *conn,
											   const struct bt_gatt_attr *attr,
											   const void *buf, uint16_t len,
											   uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 1U)
	{
		LOG_DBG("Write sensor data clear on read: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0)
	{
		LOG_DBG("Write sensor data clear on read: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint8_t val = *((uint8_t *)buf);

	if (val == 0x00)
	{
		g_app_config.sensor_data_clear_on_read = false;
	}
	else if (val == 0x01)
	{
		g_app_config.sensor_data_clear_on_read = true;
	}
	else
	{
		LOG_DBG("Write sensor data clear on read: Invalid value 0x%02x. Only 0x00 or 0x01 are allowed.", val);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	return len;
}

static ssize_t read_sensor_odr(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
							   uint16_t len, uint16_t offset)
{
	const uint16_t *data = (const uint16_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static ssize_t write_sensor_odr(struct bt_conn *conn,
								const struct bt_gatt_attr *attr,
								const void *buf, uint16_t len,
								uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 2U)
	{
		LOG_DBG("Write sensor ODR: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0)
	{
		LOG_DBG("Write sensor ODR: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint16_t val = sys_get_le16(buf);

	// TODO: Remove magic numbers here and load from universal config
	if (val != 100 && val != 200 && val != 500 && val != 1000 && val != 1500 && val != 2500)
	{
		LOG_DBG("Write sensor ODR: Invalid value %d. Only 100, 200, 500, 1000, 1500, and 2500 are allowed.", val);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	g_app_config.sensor_odr_hz = val;

	return len;
}

static ssize_t read_response_timeout_ms(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
										uint16_t len, uint16_t offset)
{
	const uint16_t *data = (const uint16_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
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

	// TODO: Add value check

	g_app_config.response_timeout_ms = sys_get_le16(buf);

	return len;
}

static ssize_t read_auto_disconnect(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									uint16_t len, uint16_t offset)
{
	const bool *data = (const bool *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static ssize_t write_auto_disconnect(struct bt_conn *conn,
									 const struct bt_gatt_attr *attr,
									 const void *buf, uint16_t len,
									 uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 1U)
	{
		LOG_DBG("Write auto disconnect: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0)
	{
		LOG_DBG("Write auto disconnect: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint8_t val = *((uint8_t *)buf);

	if (val == 0x00)
	{
		g_app_config.auto_disconnect = false;
	}
	else if (val == 0x01)
	{
		g_app_config.auto_disconnect = true;
	}
	else
	{
		LOG_DBG("Write auto disconnect: Invalid value 0x%02x. Only 0x00 or 0x01 are allowed.", val);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	return len;
}

static ssize_t read_transmit_power(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								   uint16_t len, uint16_t offset)
{
	const int8_t *data = (const int8_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
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

	g_app_config.transmit_power = val;

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
		LOG_DBG("Write response timeout: Incorrect data length %d", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0)
	{
		LOG_DBG("Write response timeout: Incorrect data offset %d", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	// TODO: Add value check

	g_app_config.sync_unix_time_ms = sys_get_le64(buf);

	return len;
}

// ------ SETTERS AND GETTERS ------

void set_transmit_power(int8_t power)
{
	g_app_config.transmit_power = power;
}

int8_t get_transmit_power_level(void)
{
	return g_app_config.transmit_power;
}

void set_sync_unix_time_ms(uint64_t time)
{
	g_app_config.sync_unix_time_ms = time;
}

uint64_t get_sync_unix_time_ms()
{
	return g_app_config.sync_unix_time_ms;
}

void set_auto_disconnect(bool enable)
{
	g_app_config.auto_disconnect = enable;
}

bool get_auto_disconnect(void)
{
	return g_app_config.auto_disconnect;
}

void set_response_timeout_ms(uint16_t timeout_ms)
{
	g_app_config.response_timeout_ms = timeout_ms;
}

uint16_t get_response_timeout_ms(void)
{
	return g_app_config.response_timeout_ms;
}

void set_sensor_odr_hz(uint16_t odr_hz)
{
	g_app_config.sensor_odr_hz = odr_hz;
}

uint16_t get_sensor_odr_hz(void)
{
	return g_app_config.sensor_odr_hz;
}

void set_sensor_data_clear_on_read(bool clear)
{
	g_app_config.sensor_data_clear_on_read = clear;
}

bool get_sensor_data_clear_on_read(void)
{
	return g_app_config.sensor_data_clear_on_read;
}

void set_adv_int_g_ms(uint16_t interval_ms)
{
	g_app_config.adv_int_g_ms = interval_ms;
}

uint16_t get_adv_int_g_ms(void)
{
	return g_app_config.adv_int_g_ms;
}

void set_adv_int_l_ms(uint16_t interval_ms)
{
	g_app_config.adv_int_l_ms = interval_ms;
}

uint16_t get_adv_int_l_ms(void)
{
	return g_app_config.adv_int_l_ms;
}

void set_adv_dur_ms(uint16_t duration_ms)
{
	g_app_config.adv_dur_ms = duration_ms;
}

uint16_t get_adv_dur_ms(void)
{
	return g_app_config.adv_dur_ms;
}

// --- Implementations for app_runtime_state_t Getters and Setters ---

void set_device_name(const char *name)
{
	// Ensure the string fits within the buffer, including null terminator.
	strncpy(g_app_config.device_name, name, DEVICE_NAME_LEN - 1);
	g_app_config.device_name[DEVICE_NAME_LEN - 1] = '\0'; // Ensure null termination
}

const char *get_device_name(void)
{
	return g_app_config.device_name;
}

void set_battery_level(int8_t level)
{
	if (level > 100)
	{
		level = 100;
	}
	else if (level < 0)
	{ // FIXME: In case of negative overflow
		level = 0;
	}
	g_app_runtime_state.battery_level = (uint8_t)level;
}

uint8_t get_battery_level(void)
{
	return g_app_runtime_state.battery_level;
}

void set_firmware_rev_str(const char *rev_str)
{
	// Ensure the string fits within the buffer, including null terminator.
	strncpy(g_app_config.firmware_rev_str, rev_str, sizeof(g_app_config.firmware_rev_str) - 1);
	g_app_config.firmware_rev_str[sizeof(g_app_config.firmware_rev_str) - 1] = '\0'; // Ensure null termination
}

const char *get_firmware_rev_str(void)
{
	return g_app_config.firmware_rev_str;
}

void set_device_log(const uint8_t *log_data, uint16_t len)
{
	// Copy data up to the buffer size.
	uint16_t bytes_to_copy = (len < LOG_BUFFER_SIZE) ? len : LOG_BUFFER_SIZE;
	memcpy(g_app_runtime_state.device_log, log_data, bytes_to_copy);
	g_app_runtime_state.device_log_len = bytes_to_copy;
}

const uint8_t *get_device_log(uint16_t *len_out)
{
	if (len_out != NULL)
	{
		*len_out = g_app_runtime_state.device_log_len;
	}
	return g_app_runtime_state.device_log;
}

uint16_t get_device_log_len(void)
{
	return g_app_runtime_state.device_log_len;
}

void set_memory_allocation_perc(uint8_t perc)
{
	g_app_runtime_state.memory_allocation_perc = perc;
}

uint8_t get_memory_allocation_perc(void)
{
	return g_app_runtime_state.memory_allocation_perc;
}

void set_sensor_data_length(uint32_t count)
{
	g_app_runtime_state.sensor_data_len = count;
}

uint32_t get_sensor_data_len(void)
{
	return g_app_runtime_state.sensor_data_len;
}

void set_sensor_data(const uint8_t *data, uint16_t len)
{
	// Copy data up to the buffer size.
	uint16_t bytes_to_copy = (len < SENSOR_DATA_BUFFER_SIZE) ? len : SENSOR_DATA_BUFFER_SIZE;
	memcpy(g_app_runtime_state.sensor_data, data, bytes_to_copy);
	g_app_runtime_state.sensor_data_len = bytes_to_copy;
}

const uint8_t *get_sensor_data(uint16_t *len_out)
{
	if (len_out != NULL)
	{
		*len_out = g_app_runtime_state.sensor_data_len;
	}
	return g_app_runtime_state.sensor_data;
}