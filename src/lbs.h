/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_LBS_H_
#define BT_LBS_H_

/**@file
 * @defgroup bt_lbs LED Button Service API
 * @{
 * @brief API for the LED Button Service (LBS).
 */

#ifdef __cplusplus
extern "C"
{
#endif

#include <zephyr/types.h>

/** @brief LBS Service UUID. */
#define BT_UUID_LBS_VAL \
    BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

/** @brief Button Characteristic UUID. */
#define BT_UUID_LBS_BUTTON_VAL \
    BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

/** @brief LED Characteristic UUID. */
#define BT_UUID_LBS_LED_VAL \
    BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_BATTERY_LEVEL_VAL \
    BT_UUID_128_ENCODE(0xc0dec0fe, 0x0bad, 0x41c7, 0x992f, 0xa5d063dbfeee)
#define BT_UUID_TRANSMIT_POWER_LEVEL_VAL \
    BT_UUID_128_ENCODE(0xb1eec10a, 0x0007, 0x4d3c, 0xa1ca, 0xae3e7e098a2b)
#define BT_UUID_DEVICE_NAME_VAL \
    BT_UUID_128_ENCODE(0x00002a00, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_FIRMWARE_REVISION_STRING_VAL \
    BT_UUID_128_ENCODE(0xcabacafe, 0xf00d, 0x4b1b, 0x9b1b, 0x1b1b1b1b1b1b)
#define BT_UUID_DEVICE_LOG_VAL \
    BT_UUID_128_ENCODE(0xbeefc0de, 0xf00d, 0x4d3c, 0xa1ca, 0xae3e7e098a2b)
#define BT_UUID_MEMORY_ALLOCATED_PERCENTAGE_VAL \
    BT_UUID_128_ENCODE(0xdeadc0de, 0xbeef, 0x4b1b, 0x9b1b, 0x1b1b1b1b1b1b)
#define BT_UUID_TOTAL_SENSOR_DATA_VAL \
    BT_UUID_128_ENCODE(0x0badf00d, 0xcafe, 0x4b1b, 0x9b1b, 0x2c931b1b1b1b)
#define BT_UUID_AUTO_DISCONNECT_VAL \
    BT_UUID_128_ENCODE(0xfadebabe, 0x0bad, 0x41c7, 0x992f, 0xa5d063dbfeee)
#define BT_UUID_RESPONSE_TIMEOUT_VAL \
    BT_UUID_128_ENCODE(0xf007face, 0xbabe, 0x47f5, 0xb542, 0xbbfd9b436872)
#define BT_UUID_SENSOR_ODR_VAL \
    BT_UUID_128_ENCODE(0x4242c0de, 0xf007, 0x4d3c, 0xa1ca, 0xae3e7e098a2b)
#define BT_UUID_SENSOR_DATA_VAL \
    BT_UUID_128_ENCODE(0xc0debabe, 0xface, 0x4f89, 0xb07d, 0xf9d9b20a76c8)
#define BT_UUID_SENSOR_DATA_CLEAR_BIT_VAL \
    BT_UUID_128_ENCODE(0xcabba6ee, 0xc0de, 0x4414, 0xa6f6, 0x46a397e18422)
#define BT_UUID_ADVERTISING_INTERVAL_GLOBAL_VAL \
    BT_UUID_128_ENCODE(0xeeafbeef, 0xcafe, 0x4d3c, 0xa1ca, 0xae3e7e098a2b)
#define BT_UUID_ADVERTISING_DURATION_VAL \
    BT_UUID_128_ENCODE(0xbabebeef, 0xcafe, 0x4d3c, 0xa1ca, 0xae3e7e098a2b)
#define BT_UUID_ADVERTISING_INTERVAL_LOCAL_VAL \
    BT_UUID_128_ENCODE(0xc0ffee00, 0xbabe, 0x4d3c, 0xa1ca, 0xae3e7e098a2b)

#define BT_UUID_LBS BT_UUID_DECLARE_128(BT_UUID_LBS_VAL)
#define BT_UUID_LBS_BUTTON BT_UUID_DECLARE_128(BT_UUID_LBS_BUTTON_VAL)
#define BT_UUID_LBS_LED BT_UUID_DECLARE_128(BT_UUID_LBS_LED_VAL)

#define BT_UUID_BATTERY_LEVEL BT_UUID_DECLARE_128(BT_UUID_BATTERY_LEVEL_VAL)
#define BT_UUID_TRANSMIT_POWER_LEVEL BT_UUID_DECLARE_128(BT_UUID_TRANSMIT_POWER_LEVEL_VAL)
#define BT_UUID_DEVICE_NAME BT_UUID_DECLARE_128(BT_UUID_DEVICE_NAME_VAL)
#define BT_UUID_FIRMWARE_REVISION_STRING BT_UUID_DECLARE_128(BT_UUID_FIRMWARE_REVISION_STRING_VAL)
#define BT_UUID_DEVICE_LOG BT_UUID_DECLARE_128(BT_UUID_DEVICE_LOG_VAL)
#define BT_UUID_MEMORY_ALLOCATED_PERCENTAGE BT_UUID_DECLARE_128(BT_UUID_MEMORY_ALLOCATED_PERCENTAGE_VAL)
#define BT_UUID_TOTAL_SENSOR_DATA BT_UUID_DECLARE_128(BT_UUID_TOTAL_SENSOR_DATA_VAL)
#define BT_UUID_AUTO_DISCONNECT BT_UUID_DECLARE_128(BT_UUID_AUTO_DISCONNECT_VAL)
#define BT_UUID_RESPONSE_TIMEOUT BT_UUID_DECLARE_128(BT_UUID_RESPONSE_TIMEOUT_VAL)
#define BT_UUID_SENSOR_ODR BT_UUID_DECLARE_128(BT_UUID_SENSOR_ODR_VAL)
#define BT_UUID_SENSOR_DATA BT_UUID_DECLARE_128(BT_UUID_SENSOR_DATA_VAL)
#define BT_UUID_SENSOR_DATA_CLEAR_BIT BT_UUID_DECLARE_128(BT_UUID_SENSOR_DATA_CLEAR_BIT_VAL)
#define BT_UUID_ADVERTISING_INTERVAL_GLOBAL BT_UUID_DECLARE_128(BT_UUID_ADVERTISING_INTERVAL_GLOBAL_VAL)
#define BT_UUID_ADVERTISING_DURATION BT_UUID_DECLARE_128(BT_UUID_ADVERTISING_DURATION_VAL)
#define BT_UUID_ADVERTISING_INTERVAL_LOCAL BT_UUID_DECLARE_128(BT_UUID_ADVERTISING_INTERVAL_LOCAL_VAL)

    /** @brief Callback type for when an LED state change is received. */
    typedef void (*led_cb_t)(const bool led_state);

    /** @brief Callback type for when the button state is pulled. */
    typedef bool (*button_cb_t)(void);

    /** @brief Callback struct used by the LBS Service. */
    struct bt_lbs_cb
    {
        /** LED state change callback. */
        led_cb_t led_cb;
        /** Button read callback. */
        button_cb_t button_cb;
    };

    /** @brief Initialize the LBS Service.
     *
     * This function registers a GATT service with two characteristics: Button
     * and LED.
     * Send notifications for the Button Characteristic to let connected peers know
     * when the button state changes.
     * Write to the LED Characteristic to change the state of the LED on the
     * board.
     *
     * @param[in] callbacks Struct containing pointers to callback functions
     *			used by the service. This pointer can be NULL
     *			if no callback functions are defined.
     *
     *
     * @retval 0 If the operation was successful.
     *           Otherwise, a (negative) error code is returned.
     */
    int bt_lbs_init(struct bt_lbs_cb *callbacks);

    /** @brief Send the button state.
     *
     * This function sends a binary state, typically the state of a
     * button, to all connected peers.
     *
     * @param[in] button_state The state of the button.
     *
     * @retval 0 If the operation was successful.
     *           Otherwise, a (negative) error code is returned.
     */
    int bt_lbs_send_button_state(bool button_state);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_LBS_H_ */
