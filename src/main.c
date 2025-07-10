/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/drivers/gpio.h>

#include "lbs.h"
#include "flashio.h"

#include <zephyr/settings/settings.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/storage/flash_map.h>

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/state.h>

#include <zephyr/drivers/timer/system_timer.h>

#include <zephyr/random/random.h>

#include <nrfx_timer.h>

#include <errno.h>

#define BLE_STATIC_ADDR {0xFF, 0x00, 0x11, 0x22, 0x33, 0x33}

#define SENSOR_DATA_BUFFER_SIZE_MAX 10

#define WORQ_THREAD_STACK_SIZE 2048

LOG_MODULE_REGISTER(main_logger, LOG_LEVEL_DBG);

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,										 // You don't need a separate fs_data struct for LittleFS
	.storage_dev = (void *)FIXED_PARTITION_ID(littlefs_storage), // Use FIXED_PARTITION_ID
	.mnt_point = FS_ROOT,										 // Choose a mount point
};

#ifndef DEVICE_NAME
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME

#ifndef DEVICE_NAME_LEN
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#endif

#endif

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

static struct gpio_callback button0_cb;
static struct gpio_callback button1_cb;
static struct gpio_callback button2_cb;
static struct gpio_callback button3_cb;

#define LED_RUN_BLINK_INTERVAL 1000

// ------ ADVERTISING -------

static bool pairing_mode = false;
static int bt_identity_static = -1;
static bool advertising_in_progress = false;

static struct k_work_delayable work_adv;

struct adv_mfg_data
{
	uint16_t company_id;
	bool device_initialized;
	uint16_t sensor_data_length;
} __packed;

// FIXME: Set device initialized when unix timestamp has ben
static struct adv_mfg_data app_adv_mfg_data = {
	.company_id = CONFIG_BT_COMPANY_ID,
	.device_initialized = false,
	.sensor_data_length = 0,
};

#define ADV_INT_MIN BT_GAP_ADV_FAST_INT_MIN_1
#define ADV_INT_MAX BT_GAP_ADV_FAST_INT_MAX_1

#define BT_LE_ADV_CONN_ACCEPT_LIST BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_USE_TX_POWER | BT_LE_ADV_OPT_NO_2M, \
												   ADV_INT_MIN, ADV_INT_MAX, NULL)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN), // TODO: comment out later
};

struct bt_data sd[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&app_adv_mfg_data, sizeof(struct adv_mfg_data)),
};

static bool advertising_ready = false;

static void cb_accept_list_cb(const struct bt_bond_info *info, void *user_data)
{
	int *bond_cnt = user_data;

	if ((*bond_cnt) < 0)
	{
		return;
	}

	int err = bt_le_filter_accept_list_add(&info->addr);
	LOG_INF("Added following peer to accept list: %x %x\n", info->addr.a.val[0],
			info->addr.a.val[1]);
	if (err)
	{
		LOG_INF("Cannot add peer to filter accept list (err: %d)\n", err);
		(*bond_cnt) = -EIO;
	}
	else
	{
		(*bond_cnt)++;
	}
}

static int accept_list_setup(uint8_t local_id)
{
	int err = bt_le_filter_accept_list_clear();

	if (err)
	{
		LOG_INF("Cannot clear accept list. Err: %d", err);
		return err;
	}

	int bond_cnt = 0;

	bt_foreach_bond(local_id, cb_accept_list_cb, &bond_cnt);

	return bond_cnt;
}

static void work_adv_handler(struct k_work *work)
{
	int err = 0;

	if (advertising_in_progress)
	{ // Stop advertising and schedule the work to run after adv_dur_ms
		err = bt_le_adv_stop();
		if (err < 0)
		{
			LOG_ERR("Failed to stop advertising. Err %d", err);
			return;
		}

		uint16_t adv_int_g_ms = get_adv_int_g_ms();

		if (adv_int_g_ms == 0)
		{
			LOG_ERR("Failed to schedule advertising work. Global advertising interval is %d", adv_int_g_ms);
			return;
		}

		LOG_INF("Advertising stopped. Work scheduled after %d.", adv_int_g_ms);
		advertising_ready = true;
		advertising_in_progress = false;
		k_work_schedule(&work_adv, K_MSEC(adv_int_g_ms));

		return;
	}

	app_adv_mfg_data.sensor_data_length = get_sensor_data_length();

	if (pairing_mode == true)
	{
		err = bt_le_filter_accept_list_clear();
		if (err)
		{
			LOG_INF("Cannot clear accept list (err: %d)\n", err);
		}
		else
		{
			LOG_INF("Clearing accept list succesfull");
		}
		pairing_mode = false;
		err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd,
							  ARRAY_SIZE(sd));
		if (err)
		{
			LOG_INF("Advertising failed to start (err %d)\n", err);
			return;
		}

		LOG_INF("Advertising successfully started\n");

		advertising_ready = false;
		advertising_in_progress = true;

		uint16_t adv_dur_ms = get_adv_dur_ms();

		k_work_schedule(&work_adv, K_MSEC(adv_dur_ms));

		return;
	}
	/* STEP 3.4.1 - Remove the original code that does normal advertising */

	// err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	//  if (err) {
	//  	LOG_INF("Advertising failed to start (err %d)\n", err);
	//  	return;
	//  }
	// LOG_INF("Advertising successfully started\n");

	/* STEP 3.4.2 - Start advertising with the Accept List */
	if (bt_identity_static != -1)
	{ // Static BT identity successfuly created.
		int allowed_cnt = accept_list_setup(bt_identity_static);
		if (allowed_cnt < 0)
		{
			LOG_INF("Acceptlist setup failed (err:%d)\n", allowed_cnt);
		}
		else
		{
			if (allowed_cnt == 0)
			{
				LOG_INF("Advertising with no Accept list \n");
				err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd,
									  ARRAY_SIZE(sd));
			}
			else
			{
				LOG_INF("Advertising with Accept list \n");
				LOG_INF("Acceptlist setup number = %d \n", allowed_cnt);
				err = bt_le_adv_start(BT_LE_ADV_CONN_ACCEPT_LIST, ad, ARRAY_SIZE(ad), sd,
									  ARRAY_SIZE(sd));
			}
			if (err)
			{
				LOG_INF("Advertising failed to start (err %d)\n", err);
				return;
			}

			LOG_INF("Advertising successfully started. Will advertise for %d milliseconds", get_adv_dur_ms());

			advertising_ready = false;
			advertising_in_progress = true;
			k_work_schedule(&work_adv, K_MSEC(get_adv_dur_ms()));
		}
	}
}

static void advertising_start(void)
{
	LOG_DBG("Advertising work submitted.");
	k_work_schedule(&work_adv, K_NO_WAIT);
}

// ------- CALLBACKS -------

static void update_data_length(struct bt_conn *conn)
{
	int err;
	struct bt_conn_le_data_len_param my_data_len = {
		.tx_max_len = BT_GAP_DATA_LEN_MAX,
		.tx_max_time = BT_GAP_DATA_TIME_MAX,
	};
	err = bt_conn_le_data_len_update(conn, &my_data_len);
	if (err)
	{
		LOG_ERR("data_len_update failed (err %d)", err);
	}
}

void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	uint16_t tx_len = info->tx_max_len;
	uint16_t tx_time = info->tx_max_time;
	uint16_t rx_len = info->rx_max_len;
	uint16_t rx_time = info->rx_max_time;
	LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
	LOG_INF("New MTU: %d bytes", bt_gatt_get_mtu(conn) - 3);
}

/** @brief BT connection (pariring procedure) callback
 *
 * This callback notifies the application that the pairing procedure
 * has been completed.
 *
 * @param conn - Connection object.
 * @param err – HCI error. Zero for success, non-zero otherwise.
 */
static void cb_bt_on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		LOG_INF("Connection failed (err %u)\n", err);
		return;
	}

	LOG_INF("Connected. Requesting data length and mtu updates.");

	update_data_length(conn);
	// update_mtu(conn);

	LOG_INF("Connected. MTU is %u, uatt MTU is %d\n", bt_gatt_get_mtu(conn), bt_gatt_get_uatt_mtu(conn));

	gpio_pin_set_dt(&led0, 1);
}

/** @brief BT disconnection callback
 *
 * This callback notifies the application that the device paired
 * Has disconnected, and provides a reason.
 *
 * @note The connection object isn't given to be freed due to
 * the disconnection. Use cb_bt_recycled to reuse the connection object
 * when it gets freed.
 *
 * @param conn - Connection object.
 * @param reason – BT_HCI_ERR_* reason for the disconnection.
 */
static void cb_bt_on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);
	gpio_pin_set_dt(&led0, 0);
}

/** @brief BT connection object is released and ready for use
 *
 * This callback notifies the application that the BT object
 * has returned to the pool and is ready to be reused.
 *
 * @note "No guarantee. First come, first serve. Treat as ISR."
 */
static void cb_bt_recycled(void)
{
	LOG_INF("Connection object available from previous conn. Advertising ready.\n");
	// advertising_start();
	advertising_ready = true;
}

/** @brief BT connection security changed callback
 *
 * This callback notifies that the existing connection's security
 * has been changed due to a bonding and encryption procedure.
 *
 * @param conn - Connection object
 * @param level - new level of the connection security (BT_SECURITY_L1-4)
 * @param err - Security error. Zero for success, non-zero otherwise.
 *
 * @note Unchanged security level means the keys have been reset.
 */
static void cb_bt_on_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		LOG_INF("Security changed: %s level %u\n", addr, level);
	}
	else
	{
		LOG_INF("Security failed: %s level %u err %s\n", addr, level, bt_security_err_to_str(err));
	}
}

/** @brief BT connection parameter request callback
 *
 * A request has been made by a remote device to change
 * the connection parameters. The application accepts the
 * new parameters by returning true, or rejects them
 * by returning false.
 *
 * @param conn - Connection object
 * @param param - Connection parameters. Can be altered before accepting,
 * which will make the device respond to the request with new parameters.
 */
static bool cb_bt_on_le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	// TODO: Filter parameters if unacceptable

	LOG_INF("cb_bt_on_le_param_req triggered.");

	return true;
}

/** @brief BT connection parameters updated callback
 *
 * The connection parameters agreed upon have been updated
 * in the active connection.
 *
 * @param conn - Connection object
 * @param interval – Connection interval.
 * @param latency – Connection latency.
 * @param timeout – Connection supervision timeout.
 */
static void cb_bt_on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	LOG_INF("cb_bt_on_le_param_updated triggered.");
}

struct bt_conn_cb connection_callbacks = {
	.connected = cb_bt_on_connected,
	.disconnected = cb_bt_on_disconnected,
	.recycled = cb_bt_recycled,
	.security_changed = cb_bt_on_security_changed,
	.le_param_req = cb_bt_on_le_param_req,
	.le_param_updated = cb_bt_on_le_param_updated,
	.le_data_len_updated = on_le_data_len_updated,
};

static void cb_bt_auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u\n", addr, passkey);
}

/** @brief Pairing request callback
 *
 * A remote device has requested to pair,
 * and no other callbacks is in use or available.
 *
 * Accept the pairing request by calling bt_conn_auth_pairing_confirm().
 * Reject the pairing request by calling bt_conn_auth_cancel().asm
 *
 * Tehe cancel callback must be supplied as well because that's the only
 * way the application can find out to stop requesting the user to accept
 * the pairing request.
 *
 * @param conn - Connection object
 */
static void cb_bt_auth_pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing requested by %s.", addr);

	int err;

	err = bt_conn_auth_pairing_confirm(conn);

	if (!err)
	{
		LOG_INF("Pairing approved successfuly to %s.", addr);
	}
	else
	{
		LOG_INF("Pairing failed to aprove to %s.", addr);
	}
}

/** @brief Pairing cancellation callback
 *
 * This callbacks notifies the application of the
 * cancellation of an ongoing pairing request.
 *
 * @note Must be provided when any pairing request callback
 * is used.
 *
 * @param conn - Connection object
 */
static void cb_bt_auth_cancel(struct bt_conn *conn)
{

	// TODO: cancel the pairing button polling job if it exists.

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.pairing_confirm = cb_bt_auth_pairing_confirm, // cb_bt_auth_pairing_confirm,
	.passkey_confirm = NULL,
	.passkey_display = NULL,
	.cancel = cb_bt_auth_cancel,
};

static void cb_bt_pairing_failed(struct bt_conn *conn,
								 enum bt_security_err reason)
{

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_ERR("Pairing failed with %s. Reason %s", addr, bt_security_err_to_str(reason));
}

static void cb_bt_pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing with %s complete and %s bonded", addr, bonded ? "is" : "is not");
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_failed = cb_bt_pairing_failed,
	.pairing_complete = cb_bt_pairing_complete,
};

// ------- SENSOR -------

static struct k_work work_sensor_data_gen;

typedef struct
{
	uint32_t time_delta;
	uint8_t data_length;
	uint8_t *raw_data;
} sensor_data_t;

static sensor_data_t sensor_data_buffer[(2 * SENSOR_DATA_BUFFER_SIZE_MAX)]; // 2x for storing values during BLE transfer.
static K_MUTEX_DEFINE(sensor_data_buffer_mutex);

static ssize_t sensor_data_buffer_size = 0;
static int64_t previous_sync_unix_time_ms = 0;
static uint32_t previous_sensor_data_time = 0;

bool sensor_data_buffer_clear();

sensor_data_t sensor_data;

sensor_data_t *sensor_data_generate()
{
	LOG_INF("Generating sensor data...");

	uint8_t data_length = sys_rand8_get() % 100 + 1;

	if (sensor_data.raw_data != NULL)
	{
		free(sensor_data.raw_data);
	}

	uint8_t *raw_data = (uint8_t *)malloc(data_length * sizeof(uint8_t));
	if (raw_data == NULL)
	{
		LOG_ERR("Failed to allocate memory for raw data.");
		return NULL;
	}

	for (uint8_t i = 0; i < data_length; i++)
	{
		*(raw_data + i) = sys_rand8_get();
	}

	sensor_data.time_delta = k_uptime_get();
	sensor_data.data_length = data_length;
	sensor_data.raw_data = raw_data;

	LOG_INF("Generated sensor data sample of %d elements at time %d.", sensor_data.data_length, sensor_data.time_delta);

	return &sensor_data;
}

uint32_t log_total_data_length_in_flash = 0;

#define HEX_CHUNK_BUFFER_SIZE_BYTES 16 // How many raw bytes to process in one hex formatting chunk
#define HEX_OUTPUT_STRING_LEN (HEX_CHUNK_BUFFER_SIZE_BYTES * 3 + 1) // (16*2 for hex chars) + (16-1 for spaces) + 1 for null terminator = 32 + 15 + 1 = 48

// Helper function to print a specific buffer in hex
void print_buffer_hex(const uint8_t *buffer, size_t length)
{
    // Use a local static buffer to avoid stack overflow for large lengths,
    // and to persist between calls if needed (though not strictly necessary here).
    // The buffer size is chosen to be reasonable for a single printk call.
    char hex_string_buffer[HEX_OUTPUT_STRING_LEN];
    int hex_string_idx;

    size_t current_offset = 0;

    while (current_offset < length) {
        // Determine how many raw bytes to process in this chunk
        size_t bytes_to_process = length - current_offset;
        if (bytes_to_process > HEX_CHUNK_BUFFER_SIZE_BYTES) {
            bytes_to_process = HEX_CHUNK_BUFFER_SIZE_BYTES;
        }

        hex_string_idx = 0; // Reset index for the new hex string chunk

        // Format the raw bytes into a hexadecimal string
        for (size_t k = 0; k < bytes_to_process; k++) {
            // Convert byte to two hex characters
            // Use static const char for hex_digits to avoid re-initialization
            static const char hex_digits[] = "0123456789abcdef";
            hex_string_buffer[hex_string_idx++] = hex_digits[(buffer[current_offset + k] >> 4) & 0xF];
            hex_string_buffer[hex_string_idx++] = hex_digits[buffer[current_offset + k] & 0xF];

            // Add a space between hex values for readability, except after the last byte of the chunk
            if (k < bytes_to_process - 1) {
                hex_string_buffer[hex_string_idx++] = ' ';
            }
        }
        // Null-terminate the formatted hex string
        hex_string_buffer[hex_string_idx] = '\0';

        // Print the entire formatted string with a single printk call
        printk("%s", hex_string_buffer); // Add newline after each chunk

        // Advance the offset
        current_offset += bytes_to_process;
    }
}

sensor_data_t *sensor_data_buffer_element_add(sensor_data_t *sensor_data)
{
	int err;

	if (sensor_data == NULL)
	{
		LOG_ERR("Failed to add sensor data element to buffer. Sensor data pointer is NULL.");
		return NULL;
	}

	int64_t sync_unix_time = get_sync_unix_time_ms(); // Get synchronized time from BLE time sync.

	if (sync_unix_time == 0)
	{
		LOG_ERR("Failed to add sensor data element to buffer. Attempted to create data before first BLE time synchronization.");
		return NULL;
	}

	if (sync_unix_time != previous_sync_unix_time_ms && !get_sensor_data_read_in_progress())
	{ // First sensor readout after BLE time sync and after reading sensor data. Prevents data being written to flash during transfer.
		previous_sync_unix_time_ms = sync_unix_time;

		// Race conditioning, if sensor data is created during BLE transfer they will be larger than 0.
		if (sensor_data_buffer_size != 0)
		{
			LOG_INF("Sync unix time says this should be the first sensor readout, but sensor data buffer size is %d", sensor_data_buffer_size);
		}
		previous_sensor_data_time = get_uptime_at_sync_ms();

		err = data_overwrite(FILE_SENSOR_DATA_LOCATION, &sync_unix_time, sizeof(sync_unix_time));
		if (err < 0)
		{
			LOG_ERR("Failed to add sensor data element to buffer. Failed to write first timestamp to file %s. Data_write returned with %d", FILE_SENSOR_DATA_LOCATION, err);
			return NULL;
		}

		set_sensor_data_length(0);
		log_total_data_length_in_flash = 0;

		LOG_INF("Sensor data in flash has been cleared.");
	}

	uint32_t current_uptime = sensor_data->time_delta; // k_sys_uptime_get has been stored in time_delta

	sensor_data->time_delta = current_uptime - previous_sensor_data_time;

	previous_sensor_data_time = current_uptime;

	err = k_mutex_lock(&sensor_data_buffer_mutex, K_FOREVER);
	if (err < 0)
	{
		LOG_ERR("Failed to lock sensor data buffer mutex. Error %d", err);
		return NULL;
	}

	if (sensor_data_buffer_size >= SENSOR_DATA_BUFFER_SIZE_MAX && !get_sensor_data_read_in_progress())
	{ // Store to flash and reset

		LOG_INF("Sensor data buffer filled. Transfer to flash initiated.");

		uint64_t total_data_size_bytes = sensor_data_buffer_size * (sizeof(uint32_t) + sizeof(uint8_t));

		for (uint64_t i = 0; i < sensor_data_buffer_size; i++)
		{
			total_data_size_bytes += sensor_data_buffer[i].data_length;
		}

		if (total_data_size_bytes == 0)
		{
			LOG_ERR("Total sensor buffer data size in bytes is 0.");
			k_mutex_unlock(&sensor_data_buffer_mutex);
			return NULL;
		}

		uint8_t *data_p = (uint8_t *)malloc(total_data_size_bytes * sizeof(uint8_t));
		if (data_p == NULL)
		{
			LOG_ERR("Failed to allocate memory for sensor data buffer transfer into flash.");
			k_mutex_unlock(&sensor_data_buffer_mutex);
			return NULL;
		}

		uint64_t ptr_offset = 0;

		for (uint64_t i = 0; i < sensor_data_buffer_size; i++)
		{
			for (uint8_t j = 0; j < 4; j++)
			{
				*(data_p + ptr_offset++) = (sensor_data_buffer[i].time_delta >> (j * 8)) && 0xFF;
			}

			*(data_p + ptr_offset++) = sensor_data_buffer[i].data_length;

			for (uint8_t j = 0; j < sensor_data_buffer[i].data_length; j++)
			{
				*(data_p + ptr_offset++) = *(sensor_data_buffer[i].raw_data + j);
			}

			free(sensor_data_buffer[i].raw_data);
			sensor_data_buffer[i].raw_data = NULL;
		}

		if (ptr_offset != total_data_size_bytes)
		{
			LOG_ERR("Total sensor data length in bytes is not equal to pointer offset. Error.");
			k_mutex_unlock(&sensor_data_buffer_mutex);
			free(data_p);
			return NULL;
		}

		err = data_append(FILE_SENSOR_DATA_LOCATION, data_p, total_data_size_bytes);
		if (err < 0)
		{
			LOG_ERR("Failed to append sensor data to flash: %d", err);
			k_mutex_unlock(&sensor_data_buffer_mutex);
			free(data_p);
			return NULL;
		}

		set_sensor_data_length(get_sensor_data_length() + sensor_data_buffer_size);
		log_total_data_length_in_flash += sensor_data_buffer_size;

		sensor_data_buffer_clear();
		sensor_data_buffer_size = 0;

		//print_buffer_hex(data_p, total_data_size_bytes);

		free(data_p);

		//print_zephyr_fs_details();
	}

	sensor_data_buffer[sensor_data_buffer_size++] = *sensor_data;

	LOG_INF("Sensor data info: timestamp[ms]: %d, data length: %d", sensor_data->time_delta, sensor_data->data_length);

	LOG_INF("Added sensor data element to sensor data buffer. Sensor data buffer's length is now %d. Total sensor data length in flash is %d", sensor_data_buffer_size, log_total_data_length_in_flash);

	k_mutex_unlock(&sensor_data_buffer_mutex);

	return &sensor_data_buffer[sensor_data_buffer_size - 1];
}

sensor_data_t *sensor_data_buffer_element_get(size_t index)
{
	if (index >= sensor_data_buffer_size)
	{
		LOG_ERR("Failed to fetch sensor data element at index %u. Index is out of bounds of sensor_data_buffer_size (%u)", index, sensor_data_buffer_size);
		return NULL;
	}

	return &sensor_data_buffer[index];
}

bool sensor_data_buffer_element_remove(size_t index)
{
	if (index >= sensor_data_buffer_size)
	{
		LOG_ERR("Failed to remove sensor data element at index %d. Index is out of bounds of sensor_data_buffer_size (%u)", index, sensor_data_buffer_size);
		return false;
	}

	// FIXME: Circular buffer please.

	int err = k_mutex_lock(&sensor_data_buffer_mutex, K_FOREVER);

	if (err < 0)
	{
		LOG_ERR("Failed to lock sensor data buffer mutex. Err %d", err);
		return false;
	}

	free(sensor_data_buffer[index].raw_data);

	sensor_data_buffer[index].raw_data = NULL;

	k_mutex_unlock(&sensor_data_buffer_mutex);

	sensor_data_buffer_size--;

	LOG_INF("Sensor data buffer element at index %d removed.", index);

	return true;
}

bool sensor_data_buffer_clear()
{
	if (sensor_data_buffer_size == 0)
	{
		return true;
	}

	int err = k_mutex_lock(&sensor_data_buffer_mutex, K_FOREVER);

	if (err < 0)
	{
		LOG_ERR("Failed to lock sensor data buffer mutex. Err %d", err);
		return false;
	}

	for (size_t i = 0; i <= sensor_data_buffer_size; i++)
	{
		free(sensor_data_buffer[i].raw_data);
		sensor_data_buffer[i].raw_data = NULL;
	}

	k_mutex_unlock(&sensor_data_buffer_mutex);

	sensor_data_buffer_size = 0;

	LOG_INF("Sensor data buffer cleared.");

	return true;
}

// ------- BLINK --------

struct k_work_delayable work_blink;

void work_blink_handler()
{
	gpio_pin_toggle_dt(&led0);
	k_work_schedule(&work_blink, K_MSEC(1000));
}

// ------- INITIALIZATION --------

static void bt_ready(int err)
{
	if (err)
	{
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	LOG_INF("Bluetooth initialized");

	err = settings_load();
	if (err == 0)
	{
		LOG_INF("main: Settings loaded!");
	}
	else
	{
		LOG_ERR("main: Failed to load settings. Err %d.", err);
	}

	// All Bluetooth subsystems are ready. Now we can start advertising.
	// advertising_start();
	advertising_ready = true;
}

bool gen_working = false;

struct k_work_delayable work_sensor_data_gen_delayable;

void cb_gpio_data_gen_scheduled(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	if (gen_working)
	{
		k_work_cancel_delayable(&work_sensor_data_gen_delayable);
		gen_working = false;
		LOG_INF("DATA GEN CANCELLED.");
	}
	else
	{
		LOG_INF("DATA GEN STARTED.");
		k_work_schedule(&work_sensor_data_gen_delayable, K_NO_WAIT);
		gen_working = true;
	}
}

void cb_gpio_data_gen(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_work_submit(&work_sensor_data_gen);
}

static struct k_work work_gpio_irq_log;

void cb_gpio_adv(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_work_submit(&work_gpio_irq_log);
}

int gpio_init()
{
	int err;

	if (!device_is_ready(button0.port))
	{
		LOG_ERR("GPIO LEDs port %s is not ready.", led0.port->name);
		return -1;
	}

	err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_LOW);
	if (err < 0)
	{
		LOG_ERR("GPIO led0 config failed. Err %d", err);
		return -1;
	}

	err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_LOW);
	if (err < 0)
	{
		LOG_ERR("GPIO led1 config failed. Err %d", err);
		return -1;
	}

	err = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_LOW);
	if (err < 0)
	{
		LOG_ERR("GPIO led2 config failed. Err %d", err);
		return -1;
	}

	err = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_LOW);
	if (err < 0)
	{
		LOG_ERR("GPIO led3 config failed. Err %d", err);
		return -1;
	}

	if (!device_is_ready(button0.port))
	{
		LOG_ERR("GPIO buttons port %s is not ready.", button0.port->name);
		return -1;
	}

	err = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (err < 0)
	{
		LOG_ERR("GPIO button0 config failed. Err %d", err);
		return -1;
	}

	err = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_FALLING);
	if (err < 0)
	{
		LOG_ERR("GPIO button0 int config failed. Err %d", err);
	}

	gpio_init_callback(&button0_cb, cb_gpio_adv, BIT(button0.pin));
	gpio_add_callback(button0.port, &button0_cb);
	if (err < 0)
	{
		LOG_ERR("GPIO button0 callback addition failed. Err %d", err);
	}

	err = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (err < 0)
	{
		LOG_ERR("GPIO button1 config failed. Err %d", err);
		return -1;
	}

	err = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_FALLING);
	if (err < 0)
	{
		LOG_ERR("GPIO button1 int config failed. Err %d", err);
	}

	gpio_init_callback(&button1_cb, NULL, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb);
	if (err < 0)
	{
		LOG_ERR("GPIO button1 callback addition failed. Err %d", err);
	}

	err = gpio_pin_configure_dt(&button2, GPIO_INPUT);
	if (err < 0)
	{
		LOG_ERR("GPIO button2 config failed. Err %d", err);
		return -1;
	}

	err = gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_FALLING);
	if (err < 0)
	{
		LOG_ERR("GPIO button2 int config failed. Err %d", err);
	}

	gpio_init_callback(&button2_cb, cb_gpio_data_gen_scheduled, BIT(button2.pin));
	gpio_add_callback(button2.port, &button2_cb);
	if (err < 0)
	{
		LOG_ERR("GPIO button2 callback addition failed. Err %d", err);
	}

	err = gpio_pin_configure_dt(&button3, GPIO_INPUT);
	if (err < 0)
	{
		LOG_ERR("GPIO button3 config failed. Err %d", err);
		return -1;
	}

	err = gpio_pin_interrupt_configure_dt(&button3, GPIO_INT_EDGE_FALLING);
	if (err < 0)
	{
		LOG_ERR("GPIO button3 int config failed. Err %d", err);
	}

	gpio_init_callback(&button3_cb, cb_gpio_data_gen, BIT(button3.pin));
	gpio_add_callback(button3.port, &button3_cb);
	if (err < 0)
	{
		LOG_ERR("GPIO button3 callback addition failed. Err %d", err);
	}

	return 0;
}

void work_sensor_data_gen_delayable_handler()
{
	sensor_data_buffer_element_add(sensor_data_generate());
	k_work_schedule(&work_sensor_data_gen_delayable, K_MSEC(500));
}

void work_sensor_data_gen_handler()
{
	sensor_data_buffer_element_add(sensor_data_generate());
}

void work_gpio_irq_log_handler()
{
	if (advertising_ready)
	{
		advertising_start();
	}
	else
	{
		LOG_INF("Can't manually start advertising, advertising is not ready.");
	}
}

int init_all()
{
	int err;

	err = fs_mount(&lfs_mnt);
	if (err == 0)
	{
		LOG_INF("LittleFS mounted successfully at %s", lfs_mnt.mnt_point);
	}
	else
	{
		LOG_ERR("Failed to mount LittleFS.");
		return -1;
	}

	print_zephyr_fs_details();

	err = gpio_init();
	if (err < 0)
	{
		LOG_ERR("GPIO init failed.");
		return -1;
	}
	LOG_INF("GPIO init complete.");

	k_work_init(&work_sensor_data_gen, work_sensor_data_gen_handler);
	k_work_init(&work_gpio_irq_log, work_gpio_irq_log_handler);
	k_work_init_delayable(&work_adv, work_adv_handler);
	k_work_init_delayable(&work_blink, work_blink_handler);
	k_work_schedule(&work_blink, K_NO_WAIT);

	k_work_init_delayable(&work_sensor_data_gen_delayable, work_sensor_data_gen_delayable_handler);

	err = register_device_initialized_p(&app_adv_mfg_data.device_initialized);
	if (err < 0)
	{
		LOG_ERR("Failed to register device initialized pointer. Returning.");
		return -1;
	}

	// ------- LOAD CHARACTERISTICS FROM FLASH MEMORY -------

	err = char_init_values();
	if (err < 0)
	{
		LOG_ERR("Failed to restore characteristics from flash, but since this might be the first time they might not exist. Continuing.");
	}
	else
	{
		LOG_INF("Successfuly restored characteristics from flash.");
	}

	// ------- SETUP STATIC RANDOM MAC ADDRESS -------

	uint8_t addr[] = BLE_STATIC_ADDR;

	addr[5] |= 0xC0;

	bt_addr_le_t le_addr = {
		.type = BT_ADDR_LE_RANDOM,
	};

	memcpy(le_addr.a.val, addr, sizeof(addr));

	err = bt_id_create(&le_addr, NULL);
	if (err < 0)
	{
		LOG_ERR("Failed to create BT identity. Err %d (%s)", err, bt_att_err_to_str(err));
	}
	else
	{
		bt_identity_static = err;
		LOG_INF("Static BT identity %d created.", err);
	}

	size_t count = CONFIG_BT_ID_MAX;
	bt_addr_le_t read_addr[CONFIG_BT_ID_MAX];
	bt_id_get(read_addr, &count);

	if (count > 0)
	{
		LOG_INF("Using BLE addr: %02X:%02X:%02X:%02X:%02X:%02X",
				read_addr[0].a.val[5], read_addr[0].a.val[4], read_addr[0].a.val[3], read_addr[0].a.val[2], read_addr[0].a.val[1], read_addr[0].a.val[0]);
	}
	else
	{
		LOG_ERR("Can't get BLE addr. Err %d (%s)", count, bt_att_err_to_str(count));
	}

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err < 0)
	{
		LOG_ERR("Bluetooth connection authentication callbacks failed to register. Err %d", err);
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err < 0)
	{
		LOG_ERR("Bluetooth connection authentication information callbacks failed to register. Err %d", err);
	}

	err = bt_conn_cb_register(&connection_callbacks);
	if (err < 0)
	{
		LOG_ERR("Bluetooth connection callback failed to register. Err %d", err);
	}

	err = bt_enable(bt_ready);
	if (err)
	{
		LOG_INF("Bluetooth init failed. Err %d (%s)", err, bt_att_err_to_str(err));
		return -1;
	}

	LOG_INF("Bluetooth initialized\n");

	return 0;
}

// ------- MISCELANEOUS END --------

int main(void)
{
	LOG_INF("Starting init...\n");

	int err;

	err = init_all();
	if (err < 0)
	{
		LOG_ERR("main: Initialization failed.");
		return -1;
	}

	LOG_INF("Init complete.");
}
