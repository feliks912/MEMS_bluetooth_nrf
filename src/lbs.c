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

#include "lbs.h"

#include <zephyr/logging/log.h>
#define CONFIG_BT_LBS_LOG_LEVEL 3
LOG_MODULE_REGISTER(bt_lbs, CONFIG_BT_LBS_LOG_LEVEL);

static bool notify_enabled;
static bool button_state;
static struct bt_lbs_cb lbs_cb;

static void lbslc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

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
#define DEFAULT_ADV_INT_GLO_MS (60 * 1000) // in ms
#define DEFAULT_ADV_DUR_MS 500			   // in ms
#define DEFAULT_ADV_INT_LOC_MS 100		   // in ms
#define DEFAULT_AUTO_DISCONNECT_BIT false

typedef struct
{
	int8_t transmit_power_level;
	bool auto_disconnect;
	uint16_t response_timeout_ms;
	uint16_t sensor_odr_hz;
	bool sensor_data_clear_on_read;
	uint16_t adv_int_glo_ms;
	uint16_t adv_int_loc_ms;
	uint16_t adv_dur_ms;
} app_config_t;

typedef struct
{
	char device_name[DEVICE_NAME_LEN];
	uint8_t battery_level;
	char firmware_rev_str[20];
	uint8_t device_log[LOG_BUFFER_SIZE];
	uint16_t device_log_len;
	uint8_t memory_allocation_perc;
	uint32_t sensor_data_count;
	uint8_t sensor_data[SENSOR_DATA_BUFFER_SIZE];
	uint16_t sensor_data_len;
} app_runtime_state_t;

static app_config_t g_app_config;
static app_runtime_state_t g_app_runtime_state;

static const app_config_t DEFAULT_APP_CONFIG = {
	.transmit_power_level = DEFAULT_TRANSMIT_POWER,
	.auto_disconnect = DEFAULT_AUTO_DISCONNECT_BIT,
	.response_timeout_ms = DEFAULT_RESPONSE_TIMEOUT_MS,
	.sensor_odr_hz = DEFAULT_SENSOR_ODR_HZ,
	.sensor_data_clear_on_read = DEFAULT_SENSOR_DATA_CLEAR_ON_READ,
	.adv_int_glo_ms = DEFAULT_ADV_INT_GLO_MS,
	.adv_dur_ms = DEFAULT_ADV_DUR_MS,
	.adv_int_loc_ms = DEFAULT_ADV_INT_LOC_MS,
};



static uint16_t sensor_data_chunk_offset = 0;

static ssize_t read_sensor_data_length(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
									   uint16_t len, uint16_t offset)
{
	const uint8_t *data = (const uint8_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(*data));
}

static ssize_t read_sensor_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
								uint16_t len, uint16_t offset)
{
	const uint8_t *data = (const uint8_t *)attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (offset >= sensor_data_chunk_offset)
	{
		LOG_DBG("Read beyond current data length: offset %u >= data_len %u\n", offset, sensor_data_chunk_offset);
		return 0;
	}

	uint16_t remaining_bytes = sensor_data_chunk_offset - offset;

	uint16_t bytes_to_copy = MIN(remaining_bytes, len);

	memcpy(buf, data + offset, bytes_to_copy);

	return bytes_to_copy;
}

static ssize_t write_led(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
						 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 1U)
	{
		LOG_DBG("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0)
	{
		LOG_DBG("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (lbs_cb.led_cb)
	{
		uint8_t val = *((uint8_t *)buf);

		if (val == 0x00 || val == 0x01)
		{
			lbs_cb.led_cb(val ? true : false);
		}
		else
		{
			LOG_DBG("Write led: Incorrect value");
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
	}

	return len;
}

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS),
					   BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED, BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_led, NULL),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA, BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ_ENCRYPT, read_sensor_data, NULL, g_app_runtime_state.sensor_data), );

int bt_lbs_init(struct bt_lbs_cb *callbacks)
{
	if (callbacks)
	{
		lbs_cb.led_cb = callbacks->led_cb;
		lbs_cb.button_cb = callbacks->button_cb;
	}

	return 0;
}

int bt_lbs_send_button_state(bool button_state)
{
	if (!notify_enabled)
	{
		return -EACCES;
	}

	return bt_gatt_notify(NULL, &lbs_svc.attrs[2], &button_state, sizeof(button_state));
}
