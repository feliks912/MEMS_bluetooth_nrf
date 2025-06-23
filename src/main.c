/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>

#include <zephyr/settings/settings.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/zms.h>

#include <dk_buttons_and_leds.h>
#include "lbs.h"

#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/fs/fs_interface.h>
#include <zephyr/fs/littlefs.h>

LOG_MODULE_REGISTER(main_logger, LOG_LEVEL_DBG);

#ifndef DEVICE_NAME
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME

#ifndef DEVICE_NAME_LEN
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#endif

#endif

#define LED_RUN_STATUS DK_LED1
#define LED_CON_STATUS DK_LED2
#define LED_RUN_BLINK_INTERVAL 1000

#define LED_USER DK_LED3

// Pressing BTN_CONN simulates RTC on regular intervals.
// Upon pressing it without any bonded devices, pairing mode is activated.
#define BTN_CONN DK_BTN1_MSK
static bool pairing_mode = false;

// Delete bond information button
#define BTN_BOND_DEL DK_BTN2_MSK

#define ADV_INT_MIN BT_GAP_ADV_FAST_INT_MIN_1
#define ADV_INT_MAX BT_GAP_ADV_FAST_INT_MAX_1

// use BT_GAP_MS_TO_ADV_INTERVAL(min_adv_int), BT_GAP_MS_TO_ADV_INTERVAL(max_adv_int) instead of INT_MIN/MAX_1/2 later during testing
// #define BT_LE_ADV_CONN_ACCEPT_LIST                                                                                                                                                     \
// 	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_USE_TX_POWER | BT_LE_ADV_OPT_NO_2M | BT_LE_ADV_OPT_DISABLE_CHAN_37 | BT_LE_ADV_OPT_DISABLE_CHAN_38, \
// 					ADV_INT_MIN, ADV_INT_MAX, NULL)

/* #define BT_LE_ADV_CONN_ACCEPT_LIST BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN, \
												   ADV_INT_MIN, ADV_INT_MAX, NULL) */

#define BT_LE_ADV_CONN_ACCEPT_LIST BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_USE_TX_POWER | BT_LE_ADV_OPT_NO_2M, \
												   ADV_INT_MIN, ADV_INT_MAX, NULL)

static bool app_button_state;
static struct k_work adv_work;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

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
		LOG_INF("Cannot clear accept list (err: %d)\n", err);
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
		LOG_INF("Advertising successfully started\n");
	}
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

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

	dk_set_led_on(LED_CON_STATUS);
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
	dk_set_led_off(LED_CON_STATUS);
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
	LOG_INF("Connection object available from previous conn. Disconnect/stop advertising is completed!\n");
	advertising_start();
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

static void app_led_cb(bool led_state)
{
	dk_set_led(LED_USER, led_state);
}

static bool app_button_cb(void)
{
	return app_button_state;
}

static struct bt_lbs_cb lbs_callbacks = {
	.led_cb = app_led_cb,
	.button_cb = app_button_cb,
};

static void btn_cb(uint32_t button_state, uint32_t has_changed)
{

	/* STEP 2.2 - Add extra button handling to remove bond information */
	if (has_changed & BTN_BOND_DEL)
	{

		uint32_t BTN_BOND_DEL_state = button_state & BTN_BOND_DEL;
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

	if (has_changed & BTN_CONN)
	{
		uint32_t BTN_PAIR_state = button_state & BTN_CONN;
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

static int init_button(void)
{
	int err;

	err = dk_buttons_init(btn_cb);
	if (err)
	{
		LOG_INF("Cannot init buttons (err: %d)\n", err);
	}

	return err;
}

/* static struct zms_fs fs = {
	.sector_size = DT_PROP(DT_CHOSEN(zephyr_flash), erase_block_size),
	.sector_count = 6,
	.offset = FIXED_PARTITION_OFFSET(storage_partition)};
 */
int main(void)
{
	int blink_status = 0;
	int err;

	LOG_INF("Starting Lesson 5 - Exercise 2 \n");

	err = dk_leds_init();
	if (err)
	{
		LOG_INF("LEDs init failed (err %d)\n", err);
		return -1;
	}

	err = init_button();
	if (err)
	{
		LOG_INF("Button init failed (err %d)\n", err);
		return -1;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err)
	{
		LOG_INF("Failed to register authorization information callbacks.\n");
		return -1;
	}

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err)
	{
		LOG_INF("Failed to register authorization callbacks.\n");
		return -1;
	}
	bt_conn_cb_register(&connection_callbacks);

	err = bt_enable(NULL);
	if (err)
	{
		LOG_INF("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	LOG_INF("Bluetooth initialized\n");

	/* STEP 1.3 - Add setting load function */
	settings_load();

	err = bt_lbs_init(&lbs_callbacks);
	if (err)
	{
		LOG_INF("Failed to init LBS (err:%d)\n", err);
		return -1;
	}

	/* fs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if (fs.flash_device == NULL)
	{
		LOG_ERR("No storage partition device on this board. Failed to init fs.");
	}
	else
	{
		err = zms_mount(&fs);

		if (err < 0)
		{
			LOG_ERR("Can't init fs! %d", err);
		}
		else
		{
			LOG_INF("fs init success.");
		}
	}

	size_t active_sector_free_space = zms_active_sector_free_space(&fs);
	size_t active_zms_free_space = zms_calc_free_space(&fs);

	LOG_INF("Sector free bytes: %zu. ate_wra: %u, data_wra: %u, ate_size: %zu. ZMS total free space: %zu",
			active_sector_free_space, fs.ate_wra, fs.data_wra, fs.ate_size, active_zms_free_space); */

	k_work_init(&adv_work, adv_work_handler);

	advertising_start();

	for (;;)
	{
		dk_set_led(LED_RUN_STATUS, (++blink_status) % 2);
		k_sleep(K_MSEC(LED_RUN_BLINK_INTERVAL));
	}
}
