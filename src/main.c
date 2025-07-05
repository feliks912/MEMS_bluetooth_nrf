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

#include <errno.h>

#define BLE_STATIC_ADDR {0xFF, 0x00, 0x11, 0x22, 0x33, 0x33}

#define WORQ_THREAD_STACK_SIZE 2048

LOG_MODULE_REGISTER(main_logger, LOG_LEVEL_DBG);

#define FS_ROOT "/lfs1"

#define FILE_CHAR_LOCATION FS_ROOT "/characteristics"
#define FILE_SENSOR_DATA_LOCATION FS_ROOT "/sdata"
#define FILE_LOGS_LOCATION FS_ROOT "/logs"

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

static struct k_timer g_k_timer;

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
static struct k_work gpio_irq_log_work;

// ------ ADVERTISING -------

static bool pairing_mode = false;

#define ADV_INT_MIN BT_GAP_ADV_FAST_INT_MIN_1
#define ADV_INT_MAX BT_GAP_ADV_FAST_INT_MAX_1

#define BT_LE_ADV_CONN_ACCEPT_LIST BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_USE_TX_POWER | BT_LE_ADV_OPT_NO_2M, \
												   ADV_INT_MIN, ADV_INT_MAX, NULL)

static struct k_work adv_work;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_SOME, BT_UUID_APP_CONFIG_VAL),
};

static bool advertising_ready = false;

static void setup_accept_list_cb(const struct bt_bond_info *info, void *user_data)
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

static int setup_accept_list(uint8_t local_id)
{
	int err = bt_le_filter_accept_list_clear();

	if (err)
	{
		LOG_INF("Cannot clear accept list. Err: %d", err);
		return err;
	}

	int bond_cnt = 0;

	bt_foreach_bond(local_id, setup_accept_list_cb, &bond_cnt);

	return bond_cnt;
}

static void adv_work_handler(struct k_work *work)
{
	int err = 0;

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
		advertising_ready = false;
		LOG_INF("Advertising successfully started\n");
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
	int allowed_cnt = setup_accept_list(BT_ID_DEFAULT);
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
			LOG_INF("Acceptlist setup number  = %d \n", allowed_cnt);
			err = bt_le_adv_start(BT_LE_ADV_CONN_ACCEPT_LIST, ad, ARRAY_SIZE(ad), sd,
								  ARRAY_SIZE(sd));
		}
		if (err)
		{
			LOG_INF("Advertising failed to start (err %d)\n", err);
			return;
		}
		advertising_ready = false;
		LOG_INF("Advertising successfully started\n");
	}
}

static void advertising_start(void)
{
	LOG_DBG("Advertising work submitted.");
	k_work_submit(&adv_work);
}

// ------ ADVERTISING END -------

// ------- CALLBACKS -------

/** @brief BT connection (pariring procedure) callback
 *
 * This callback notifies the application that the pairing procedure
 * has been completed.
 *
 * @param conn - Connection object.
 * @param err – HCI error. Zero for success, non-zero otherwise.
 */
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		LOG_INF("Connection failed (err %u)\n", err);
		return;
	}

	LOG_INF("Connected\n");

	gpio_pin_set_dt(&led0, 1);
}

/** @brief BT disconnection callback
 *
 * This callback notifies the application that the device paired
 * Has disconnected, and provides a reason.
 *
 * @note The connection object isn't given to be freed due to
 * the disconnection. Use recycled_cb to reuse the connection object
 * when it gets freed.
 *
 * @param conn - Connection object.
 * @param reason – BT_HCI_ERR_* reason for the disconnection.
 */
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
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
static void recycled_cb(void)
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
static void on_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
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
static bool on_le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	// TODO: Filter parameters if unacceptable

	LOG_INF("On_le_param_req triggered.");

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
static void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
}

struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
	.recycled = recycled_cb,
	.security_changed = on_security_changed,
	.le_param_req = on_le_param_req,
	.le_param_updated = on_le_param_updated};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
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
static void auth_pairing_confirm(struct bt_conn *conn)
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
static void auth_cancel(struct bt_conn *conn)
{

	// TODO: cancel the pairing button polling job if it exists.

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.pairing_confirm = auth_pairing_confirm, // auth_pairing_confirm,
	.passkey_confirm = NULL,
	.passkey_display = NULL,
	.cancel = auth_cancel,
};

static void pairing_failed(struct bt_conn *conn,
						   enum bt_security_err reason)
{

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_ERR("Pairing failed with %s. Reason %s", addr, bt_security_err_to_str(reason));
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing with %s complete and %s bonded", addr, bonded ? "is" : "is not");
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_failed = pairing_failed,
	.pairing_complete = pairing_complete,
};

static void btn_cb(uint32_t button_state, uint32_t has_changed)
{

	/* STEP 2.2 - Add extra button handling to remove bond information */
	if (has_changed & button2.pin)
	{

		uint32_t BTN_BOND_DEL_state = button_state & button2.pin;
		if (BTN_BOND_DEL_state == 0)
		{
			LOG_INF("Button 1 released.");

			int err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
			if (err)
			{
				LOG_INF("Cannot delete bond (err: %d)\n", err);
			}
			else
			{
				LOG_INF("Bond deleted succesfully");
			}
		}
		else
		{
			LOG_INF("Button 1 pressed.");
		}
	}
	/* STEP 4.2.2 Add extra button handling pairing mode (advertise without using Accept List) */

	if (has_changed & button3.pin)
	{
		uint32_t BTN_PAIR_state = button_state & button3.pin;
		if (BTN_PAIR_state == 0)
		{

			LOG_INF("Button 0 released.");

			pairing_mode = true;
			int err_code = bt_le_adv_stop();
			if (err_code)
			{
				LOG_INF("Cannot stop advertising err= %d \n", err_code);
				return;
			}
			// recycled_cb will be called after the advertising stopped we will continue advertise when we receive that callback
		}
		else
		{
			LOG_INF("Button 0 pressed.");
		}
	}
}

// ------- CALLBACKS END -------

// ------- FLASH ------

const int data_append(const char *filename, const char *data, ssize_t len)
{
	int written = 0;
	struct fs_file_t l_file;
	fs_file_t_init(&l_file);

	int err = fs_open(&l_file, filename, FS_O_CREATE | FS_O_APPEND);
	if (err < 0)
	{
		LOG_ERR("Failed to open file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Opened file %s.", filename);
	}

	err = fs_write(&l_file, data, len);
	if (err < 0)
	{
		LOG_ERR("Failed to write %d bytes to %s. Err %d (%s)", len, filename, err, strerror(-err));
		fs_close(&l_file);
		return err;
	}
	else
	{
		LOG_DBG("Written %d bytes to %s.", len, filename);
		written = err;
	}

	err = fs_close(&l_file);
	if (err < 0)
	{
		LOG_ERR("Failed to close file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Closed file %s", filename);
	}

	return written;
}

const int data_write(const char *filename, const char *data, ssize_t len)
{
	int written = 0;
	struct fs_file_t l_file;
	fs_file_t_init(&l_file);

	int err = fs_open(&l_file, filename, FS_O_CREATE | FS_O_WRITE);
	if (err < 0)
	{
		LOG_ERR("Failed to open file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Oppened file %s.", filename);
	}

	err = fs_seek(&l_file, 0, FS_SEEK_SET);
	if (err < 0)
	{
		LOG_ERR("Failed to set file seek of file %s to 0. Err %d (%s)", filename, err, strerror(-err));
		fs_close(&l_file);
		return err;
	}

	err = fs_write(&l_file, data, len);
	if (err < 0)
	{
		LOG_ERR("Failed to append %d bytes to %s. Err %d (%s)", len, filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Written %d bytes to %s.", len, filename);
		written = err;
	}

	err = fs_close(&l_file);
	if (err < 0)
	{
		LOG_ERR("Failed to close file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Closed file %s", filename);
	}

	return written;
}

const int data_read(const char *filename, void *buf, ssize_t offset, ssize_t len)
{
	int read = 0;
	struct fs_file_t l_file;
	fs_file_t_init(&l_file);

	int err = fs_open(&l_file, filename, FS_O_READ);
	if (err < 0)
	{
		LOG_ERR("Failed to open file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Oppened file %s.", filename);
	}

	err = fs_seek(&l_file, offset, FS_SEEK_SET);
	if (err < 0)
	{
		LOG_ERR("Failed to set file seek of file %s to %d. Err %d (%s)", filename, offset, err, strerror(-err));
		fs_close(&l_file);
		return err;
	}

	err = fs_read(&l_file, buf, len);
	if (err < 0)
	{
		LOG_ERR("Failed to read %d bytes from %s. Err %d (%s)", len, filename, err, strerror(-err));
		fs_close(&l_file);
		return err;
	}
	else
	{
		LOG_DBG("Read %d bytes from %s.", len, filename);
		read = err;
	}

	err = fs_close(&l_file);
	if (err < 0)
	{
		LOG_ERR("Failed to close file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Closed file %s", filename);
	}

	return read;
}

// ------- FLASH END ------

// ------- SENSOR -------

typedef struct
{
	uint64_t start_time;
	uint32_t length_time;
	uint16_t ODR;
	uint16_t length_data;
	uint16_t raw_data[];
} sensor_data_t;

static sensor_data_t **sensor_data_list = NULL;

static size_t sensor_data_list_size = 0;
static size_t sensor_data_list_capacity = 0;

#define INITIAL_SENSOR_DATA_LIST_CAPACITY 8

bool sensor_data_list_init()
{
	if (sensor_data_list != NULL)
	{
		return false;
	}

	sensor_data_list = (sensor_data_t **)malloc(INITIAL_SENSOR_DATA_LIST_CAPACITY * sizeof(sensor_data_t *));
	if (sensor_data_list == NULL)
	{
		sensor_data_list_capacity = 0;
		sensor_data_list_size = 0;
		LOG_ERR("Failed to allocate initial sensor_data_t pointer space with malloc.");
		return false;
	}
	sensor_data_list_capacity = INITIAL_SENSOR_DATA_LIST_CAPACITY;
	sensor_data_list_size = 0;
	return true;
}

bool sensor_data_list_add(uint64_t start_time, uint32_t length_time, uint16_t ODR,
						  uint16_t length_data, const uint16_t *raw_data)
{
	if (sensor_data_list_size >= sensor_data_list_capacity)
	{
		size_t new_list_capacity = sensor_data_list_capacity + 1;
		if (new_list_capacity < sensor_data_list_capacity)
		{
			LOG_ERR("Sensor data list capacity overflow.");
			return false;
		}

		sensor_data_t **temp_list = (sensor_data_t **)realloc(sensor_data_list, new_list_capacity * sizeof(sensor_data_t *));
		if (temp_list == NULL)
		{
			LOG_ERR("Failed to allocate temporary sensor data list with realloc.");
			return false;
		}
		sensor_data_list = temp_list;
		sensor_data_list_capacity = new_list_capacity;
	}

	size_t struct_with_data_size = sizeof(sensor_data_t) + (length_data * sizeof(uint8_t));
	sensor_data_t *new_data = (sensor_data_t *)malloc(struct_with_data_size);
	if (new_data == NULL)
	{
		LOG_ERR("Failed to allocate new sensor data struct with malloc.");
		return false;
	}

	new_data->start_time = start_time;
	new_data->length_time = length_time;
	new_data->ODR = ODR;
	new_data->length_data;
	memcpy(new_data->raw_data, raw_data, length_data * sizeof(uint8_t));

	sensor_data_list[sensor_data_list_size++] = new_data;

	return true;
}

sensor_data_t *sensor_data_list_get(size_t index)
{
	if (index >= sensor_data_list_size)
	{
		LOG_ERR("Failed to fetch sensor data element at index %u. Index is out of bounds of sensor_data_list_size (%u)", index, sensor_data_list_size);
		return NULL;
	}

	return sensor_data_list[index];
}

bool sensor_data_list_remove(size_t index)
{
	if (index >= sensor_data_list_size)
	{
		LOG_ERR("Failed to remove sensor data element at index %d. Index is out of bounds of sensor_data_list_size (%u)", index, sensor_data_list_size);
		return false;
	}

	//FIXME: Circular buffer please.

	free(sensor_data_list[index]);
	sensor_data_list_size--;

	return true;
}

void sensor_data_list_clear(){
	if(sensor_data_list == NULL){
		return;
	}

	for(size_t i = 0; i <= sensor_data_list_size; i++) {
		free(sensor_data_list[i]);
	}

	free(sensor_data_list);

	sensor_data_list = NULL;
	sensor_data_list_size = 0;
	sensor_data_list_capacity = 0;	
}

// ------- SENSOR END -------

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

void gpio_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_work_submit(&gpio_irq_log_work);
}

int64_t dt = 0;
void timer_check(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	LOG_INF("dt: %u", k_uptime_delta(&dt));
}

int init_gpio()
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

	gpio_init_callback(&button0_cb, gpio_cb, BIT(button0.pin));
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

	gpio_init_callback(&button1_cb, gpio_cb, BIT(button1.pin));
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

	gpio_init_callback(&button2_cb, gpio_cb, BIT(button2.pin));
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

	gpio_init_callback(&button3_cb, timer_check, BIT(button3.pin));
	gpio_add_callback(button3.port, &button3_cb);
	if (err < 0)
	{
		LOG_ERR("GPIO button3 callback addition failed. Err %d", err);
	}

	return 0;
}

void gpio_irq_log_work_handler()
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

	err = init_gpio();
	if (err < 0)
	{
		LOG_ERR("GPIO init failed.");
		return -1;
	}
	LOG_INF("GPIO init complete.");

	k_work_init(&gpio_irq_log_work, gpio_irq_log_work_handler);
	k_work_init(&adv_work, adv_work_handler);

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

// ------- INITIALIZATION END --------

// ------- MISCELANEOUS --------

void set_bt_led(bool state)
{
	gpio_pin_set_dt(&led1, state);
}

struct k_work_delayable w_blink;

void blink()
{
	gpio_pin_toggle_dt(&led0);
	k_work_schedule(&w_blink, K_MSEC(1000));
}

void expiration_fn()
{
	LOG_INF("Timer expired. Current uptime is %u ms", k_uptime_get());
}

// ------- MISCELANEOUS END --------

int main(void)
{

	LOG_INF("Starting!\n");

	int err;

	err = init_all();
	if (err < 0)
	{
		LOG_ERR("main: Initialization failed.");
		return -1;
	}

	LOG_INF("Init complete.");

	k_work_init_delayable(&w_blink, blink);
	k_work_schedule(&w_blink, K_MSEC(1000));
}
