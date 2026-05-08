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
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "my_lbs.h"

LOG_MODULE_DECLARE(my_BLEService);

extern uint8_t ledLocationInterval;

/* Runtime state for client subscriptions and cached callbacks. */
static bool notify_mysensor_enabled, indicate_enabled, button_state;
static bool notify_eeg_ch_enabled[8u];
static struct my_lbs_cb lbs_cb;
static struct bt_gatt_indicate_params ind_params;

static void mylbsbc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	(void)attr;
	/* Button characteristic uses indication (confirmed by peer ACK). */
	indicate_enabled = (value == BT_GATT_CCC_INDICATE);
}

static void mylbsbc_ccc_mysensor_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	(void)attr;
	/* Sensor characteristic uses notification (unconfirmed). */
	notify_mysensor_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void mylbsbc_ccc_eegsensor_cfg_changed(uint8_t ch_idx, uint16_t value)
{
    if (ch_idx >= ARRAY_SIZE(notify_eeg_ch_enabled)) {
        return;
    }
    notify_eeg_ch_enabled[ch_idx] = (value == BT_GATT_CCC_NOTIFY);
}

static void ccc_eeg_ch1(const struct bt_gatt_attr *attr, uint16_t value)
{
    (void)attr;
    mylbsbc_ccc_eegsensor_cfg_changed(0, value);
}
static void ccc_eeg_ch2(const struct bt_gatt_attr *attr, uint16_t value)
{
    (void)attr;
    mylbsbc_ccc_eegsensor_cfg_changed(1, value);
}
static void ccc_eeg_ch3(const struct bt_gatt_attr *attr, uint16_t value)
{
    (void)attr;
    mylbsbc_ccc_eegsensor_cfg_changed(2, value);
}
static void ccc_eeg_ch4(const struct bt_gatt_attr *attr, uint16_t value)
{
    (void)attr;
    mylbsbc_ccc_eegsensor_cfg_changed(3, value);
}
static void ccc_eeg_ch5(const struct bt_gatt_attr *attr, uint16_t value)
{
    (void)attr;
    mylbsbc_ccc_eegsensor_cfg_changed(4, value);
}
static void ccc_eeg_ch6(const struct bt_gatt_attr *attr, uint16_t value)
{
    (void)attr;
    mylbsbc_ccc_eegsensor_cfg_changed(5, value);
}
static void ccc_eeg_ch7(const struct bt_gatt_attr *attr, uint16_t value)
{
    (void)attr;
    mylbsbc_ccc_eegsensor_cfg_changed(6, value);
}
static void ccc_eeg_ch8(const struct bt_gatt_attr *attr, uint16_t value)
{
    (void)attr;
    mylbsbc_ccc_eegsensor_cfg_changed(7, value);
}


static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	LOG_DBG("Indication %s\n", err != 0U ? "fail" : "success");
}

static ssize_t write_led(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	/*
	 * Expected write payload (4 bytes):
	 * [0] red LED intensity
	 * [1] sensor integration time
	 * [2] IR LED intensity
	 * [3] LED location interval selector
	 */
	if (len != 4U) {
		LOG_DBG("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_DBG("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (lbs_cb.setSensorAndLED) {
		// Read the received value
		uint8_t val_led_red = *((uint8_t *)buf); // Red LED intensity
		uint8_t val_intgT = *((uint8_t *)buf+1); // Photo Detector integration time
		uint8_t val_led_ir= *((uint8_t *)buf+2); // IR LED intensity
		
		/* Shared global used by the application sampling/scan logic. */
		ledLocationInterval = *((uint8_t *)buf+3);

		// lbs_cb.setSensorAndLED(val_led_red, val_intgT);
		lbs_cb.setSensorAndLED(val_led_red, val_led_ir, val_intgT);
	}

	return len;
}

static ssize_t read_button(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	// get a pointer to button_state which is passed in the BT_GATT_CHARACTERISTIC() and stored in attr->user_data
	const char *value = attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (lbs_cb.button_cb) {
		// Call the application callback function to update the get the current value of the button
		button_state = lbs_cb.button_cb();
		return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
	}

	return 0;
}

/* Service Declaration */
/* NOTE:
    * BT_GATT_CHRC_NOTIFY: Property to notify the central device ASYNCHRONOUSLY
    * BT_GATT_PERM_NONE: Property to not read/write this property
    * BT_GATT_PERM_READ: Property to allow read requests from client
    * BT_GATT_PERM_WRITE: Property to allow write requests from client
*/
BT_GATT_SERVICE_DEFINE(
	my_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS),
	/* attrs[1]: Button characteristic declaration, attrs[2]: button value */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_BUTTON, BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE, BT_GATT_PERM_READ, read_button, NULL, &button_state),
	/* attrs[3]: Button CCC descriptor */
	BT_GATT_CCC(mylbsbc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* attrs[4]: LED control characteristic (write-only from central) */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_led, NULL),
	/* attrs[6]: Sensor characteristic declaration, attrs[7]: sensor value */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_MYSENSOR, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	/* attrs[8]: Sensor CCC descriptor */
	BT_GATT_CCC(mylbsbc_ccc_mysensor_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    /* attribute for ADS1299 sensors */
    /* Create a seperate BLE notify chanel for ADS data */
    // All channelss
    BT_GATT_CHARACTERISTIC(BT_UUID_EEGSENSOR_CH1, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_eeg_ch1, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EEGSENSOR_CH2, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_eeg_ch2, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EEGSENSOR_CH3, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_eeg_ch3, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EEGSENSOR_CH4, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_eeg_ch4, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EEGSENSOR_CH5, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_eeg_ch5, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EEGSENSOR_CH6, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_eeg_ch6, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EEGSENSOR_CH7, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_eeg_ch7, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EEGSENSOR_CH8, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_eeg_ch8, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int my_lbs_init(struct my_lbs_cb *callbacks)
{
	if (callbacks) {
		lbs_cb.setSensorAndLED = callbacks->setSensorAndLED;
		lbs_cb.button_cb = callbacks->button_cb;
        // TODO ADS1299
	}

	return 0;
}

int my_lbs_send_button_state_indicate(bool button_state)
{
	if (!indicate_enabled) {
		return -EACCES;
	}
	/* Attribute index 2 is the button characteristic value. */
	ind_params.attr = &my_lbs_svc.attrs[2];
	ind_params.func = indicate_cb; // A remote device has ACKed at its host layer (ATT ACK)
	ind_params.destroy = NULL;
	ind_params.data = &button_state;
	ind_params.len = sizeof(button_state);
	return bt_gatt_indicate(NULL, &ind_params);
}

int my_lbs_send_sensor_notify(uint32_t sensor_value)
{
	if (!notify_mysensor_enabled) {
		return -EACCES;
	}

	/* Attribute index 7 is the sensor characteristic value in this layout. */
	return bt_gatt_notify(NULL, &my_lbs_svc.attrs[7], &sensor_value, sizeof(sensor_value));
}

int my_lbs_send_sensor_notify_RED_NIRS(uint16_t RED630, uint16_t RED680, uint16_t NIR)
{
	if (!notify_mysensor_enabled) {
		return -EACCES;
	}
	uint16_t data[3] = {RED630, RED680, NIR};
	return bt_gatt_notify(NULL, &my_lbs_svc.attrs[7], &data, sizeof(data));
}

int my_lbs_send_sensor_notify_RED_NIRS_LEDindex(uint16_t RED630, uint16_t RED680, uint16_t NIR, uint16_t LEDlocation)
{
	if (!notify_mysensor_enabled) {
		return -EACCES;
	}
	uint16_t data[4] = {RED630, RED680, NIR, LEDlocation};
	return bt_gatt_notify(NULL, &my_lbs_svc.attrs[7], &data, sizeof(data));
}

int my_lbs_send_eeg_ch_notify(uint8_t ch, const uint8_t *data, uint16_t len)
{
    static const uint8_t eeg_val_attr_idx[8] = {10U, 13U, 16U, 19U, 22U, 25U, 28U, 31U};

    if ((ch < 1U) || (ch > 8U) || (data == NULL) || (len == 0U)) {
        return -EINVAL;
    }
    if (!notify_eeg_ch_enabled[ch - 1U]) {
        return -EACCES;
    }

    return bt_gatt_notify(NULL, &my_lbs_svc.attrs[eeg_val_attr_idx[ch - 1U]], data, len);
}
