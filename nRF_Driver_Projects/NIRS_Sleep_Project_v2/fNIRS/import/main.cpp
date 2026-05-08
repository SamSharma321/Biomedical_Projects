/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include "as7341.h"
#include "my_lbs.h"
#include "ads1299.h"


/* Stack size (bytes) allocated to each Zephyr thread defined at file end. */
#define THREAD_STACK_SIZE 1024
/* Scheduling priority for the LED control thread (lower value = higher priority). */
#define THREAD0_PRIORITY 7
/* Scheduling priority for the sensor acquisition thread. */
#define THREAD1_PRIORITY 7
/* BLE advertised device name pulled from Zephyr Kconfig (with fallback). */
#ifdef CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#else
#define DEVICE_NAME "BITN_FNIR"
#endif
/* Length of DEVICE_NAME excluding trailing null terminator. */
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Devicetree alias for RED LED channel (closest emitter pair). */
#define ledRED DT_ALIAS(ledred)
/* Devicetree alias for IR LED channel (closest emitter pair). */
#define ledIR DT_ALIAS(ledir)
/* Devicetree node for RED LED channel at ~2 cm source-detector distance. */
#define ledRED2cm DT_NODELABEL(ledred_2cm)
/* Devicetree node for IR LED channel at ~2 cm source-detector distance. */
#define ledIR2cm DT_NODELABEL(ledir_2cm)
/* Devicetree node for RED LED channel at ~3 cm source-detector distance. */
#define ledRED3cm DT_NODELABEL(ledred_3cm)
/* Devicetree node for IR LED channel at ~3 cm source-detector distance. */
#define ledIR3cm DT_NODELABEL(ledir_3cm)

/* PWM period for LED drive control in nanoseconds. */
#define PWM_PERIOD_NS 200000
/* Reserved sensor update interval constant (currently unused in runtime logic). */
#define sensorDataUpdateInterval 500

/* PWM spec object for RED LED channel (primary location). */
static const struct pwm_dt_spec pwm_ledRED = PWM_DT_SPEC_GET(ledRED);
/* PWM spec object for IR LED channel (primary location). */
static const struct pwm_dt_spec pwm_ledIR = PWM_DT_SPEC_GET(ledIR);

/* PWM spec object for RED LED channel at 2 cm location. */
static const struct pwm_dt_spec pwm_ledRED2cm = PWM_DT_SPEC_GET(ledRED2cm);
/* PWM spec object for IR LED channel at 2 cm location. */
static const struct pwm_dt_spec pwm_ledIR2cm = PWM_DT_SPEC_GET(ledIR2cm);

/* PWM spec object for RED LED channel at 3 cm location. */
static const struct pwm_dt_spec pwm_ledRED3cm = PWM_DT_SPEC_GET(ledRED3cm);
/* PWM spec object for IR LED channel at 3 cm location. */
static const struct pwm_dt_spec pwm_ledIR3cm = PWM_DT_SPEC_GET(ledIR3cm);

/* Time in seconds between source-location index increments in main loop. */
uint8_t ledLocationInterval = 1;

/* BLE advertising configuration (connectable, identity address, ~500 ms interval). */
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
	800,						  /* Min Advertising Interval 500ms (800*0.625ms) */
	801,						  /* Max Advertising Interval 500.625ms (801*0.625ms) */
	NULL);						  /* Set to NULL for undirected advertising */

/* Primary advertising payload (flags + complete device name). */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Scan response payload containing custom LBS service UUID. */
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};
/* Active BLE connection reference used across callbacks and worker loops. */
struct bt_conn *my_conn = NULL;
/* Reusable MTU exchange request context. */
static struct bt_gatt_exchange_params exchange_params;

/* AS7341 spectral sensor driver object instance. */
Adafruit_AS7341 spectrumSensor;
/* Optional counter/debug variable for sample-rate tracking (currently unused). */
uint16_t hertz = 0;
/* True while a BLE central is connected and notifications are allowed. */
bool bleConnected = false;
/* Buffer for AS7341 channel samples pushed via BLE notifications. */
uint16_t sensordata[12] = {0};
/* LED intensity and source-location state shared between callback/threads/main. */
uint8_t LEDintensity, irLEDintensity, redLEDintensity, LEDlocation = 0;
/* BLE notify pacing interval in milliseconds (updated from integration time callback). */
uint16_t BLEtransferInterval = 20;

LOG_MODULE_REGISTER(main_log, LOG_LEVEL_DBG);


#ifdef ADS1299
ads1299 adsObj[NUM_OF_ADS1299_DEV];
#endif

/* Requests maximum BLE data length to increase payload throughput. */
static void update_data_length(struct bt_conn *conn)
{
	int err; /* Return code from BLE stack API calls. */
	(void)conn; /* Connection is currently taken from global my_conn. */
	struct bt_conn_le_data_len_param my_data_len = {
		.tx_max_len = BT_GAP_DATA_LEN_MAX,
		.tx_max_time = BT_GAP_DATA_TIME_MAX,
	};
    struct bt_conn_le_phy_param my_ble_param = { // Initial setting to 2 Mbps
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_1M,
    };

	err = bt_conn_le_data_len_update(my_conn, &my_data_len);
    err |= bt_conn_le_phy_update(my_conn, &my_ble_param);

	if (err)
	{
		LOG_ERR("data_len_update failed (err %d)", err);
	}
}

/* Handles completion status of ATT MTU exchange and logs effective payload MTU. */
static void exchange_func(struct bt_conn *conn, uint8_t att_err, // Not used. Why?
						  struct bt_gatt_exchange_params *params)
{
	(void)params; /* Unused callback context in current implementation. */
	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
	if (!att_err)
	{
		uint16_t payload_mtu = /* ATT payload bytes available to application data. */
			bt_gatt_get_mtu(conn) - 3; // 3 bytes used for Attribute headers.
		LOG_INF("New MTU: %d bytes", payload_mtu);
	}
}

/* Triggers ATT MTU exchange for the active connection. */
static void update_mtu(struct bt_conn *conn) // TODO NOTE: Not used. Why?
{
	int err; /* Return code from MTU exchange request. */
	exchange_params.func = exchange_func;

	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err)
	{
		LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
	}
}

/* BLE connection callback: stores connection ref and enables LED output state. */
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err %u)", err);
		return;
	}

	LOG_INF("Connected");
	my_conn = bt_conn_ref(conn);
	// update_data_length(my_conn);
	// update_mtu(my_conn);
	redLEDintensity = 100;
	irLEDintensity = 100;
	bleConnected = true;
#ifdef ADS1299
    for (unsigned int i = 0U; i < NUM_OF_ADS1299_DEV; i++) {
        if (adsObj[i].getDevIdx() != 0xFF) {
            // Enter standby mode
            adsObj[i].issueActnCmd(ADS_WAKEUP);
            k_usleep(150u);
            // Start continuous reading command
            adsObj[i].issueActnCmd(ADS_START);
            k_usleep(5);
            adsObj[i].issueActnCmd(ADS_RDATAC);
        }
    }
#endif
}

/* BLE disconnection callback: clears connection state and turns off LED outputs. */
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	(void)conn; /* Current implementation tracks active link via global my_conn. */
	LOG_INF("Disconnected (reason %u)", reason);
	bleConnected = false;

    if (my_conn) {
	    bt_conn_unref(my_conn);
        my_conn = NULL;
    }

	redLEDintensity = 0;
	irLEDintensity = 0;
#ifdef ADS1299
    for (unsigned int i = 0U; i < NUM_OF_ADS1299_DEV; i++) {
        if (adsObj[i].getDevIdx() != 0xFF) {
            // Stop continuous reading command
            adsObj[i].issueActnCmd(ADS_SDATAC);
            // Enter standby mode
            adsObj[i].issueActnCmd(ADS_STANDBY);
        }
    }
#endif

    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err && (err != -EALREADY)) {
        LOG_ERR("Advertising failed to restart (err %d)", err);
    }
}

/* Connection callback table registered with Zephyr Bluetooth stack. */
struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};

/* GATT write callback: updates LED duty targets and AS7341 integration timing. */
// Update user requests for AS7341a and LED intensities
static void app_set_sensor_LED_cb(uint16_t redledIntensity, uint16_t irledIntensity, uint16_t sensorIntTime_20ms)
{
    int err; /* Reserved for future API return-code checks. */
    (void)err;
    
    // Cap intensities
    if (redledIntensity > 100) redledIntensity = 100;
	if (irledIntensity > 100) irledIntensity = 100;

	// LEDintensity = ledIntensity;
	redLEDintensity = redledIntensity;
	irLEDintensity = irledIntensity;

	// t = (ATIME+1)x(ASTEP+1)x2.78us
	spectrumSensor.setATIME(99); // t=(ASTEP+1)x278us
	float temp = sensorIntTime_20ms * 20000 / 278 - 1; /* Derived ASTEP from requested 20 ms units. */
	spectrumSensor.setASTEP((uint16_t)temp); // recommended value 599
	BLEtransferInterval = sensorIntTime_20ms * 20; // TODO change this when you integrate ADS1299
	LOG_INF("Sensor Integration Time: %dms", (uint16_t)((temp + 1) * 278 / 1000));
	LOG_INF("BLE interval Time: %dms", sensorIntTime_20ms * 20);
	LOG_INF("RED LED intensity: %d", redledIntensity);
	LOG_INF("IR LED intensity: %d", irledIntensity);
}

/* Application callback table passed into custom LBS service init. */
static struct my_lbs_cb app_callbacks = {
	.setSensorAndLED = app_set_sensor_LED_cb,
};


/* LED worker thread: drives all PWM channels continuously using current intensities. */
void thread_led(void)
{
    int err; /* Aggregated error result from repeated PWM set operations. */

    if (!pwm_is_ready_dt(&pwm_ledRED) || !pwm_is_ready_dt(&pwm_ledIR) ||
        !pwm_is_ready_dt(&pwm_ledRED2cm) || !pwm_is_ready_dt(&pwm_ledIR2cm) ||
        !pwm_is_ready_dt(&pwm_ledRED3cm) || !pwm_is_ready_dt(&pwm_ledIR3cm)) {
        LOG_ERR("Error: One or more PWM devices are not ready");
        return;
    }

    // Continuously runnig thread with occcassional sleep
    // Assign this to higher priority - ADS1299 sampling is 250 anyways
    while (1)
    {
        // LED1
        err = pwm_set_dt(&pwm_ledRED, PWM_PERIOD_NS, PWM_PERIOD_NS * redLEDintensity / 100);
        err |= pwm_set_dt(&pwm_ledIR, PWM_PERIOD_NS, PWM_PERIOD_NS * irLEDintensity / 100);
        // LED2
        err |= pwm_set_dt(&pwm_ledRED2cm, PWM_PERIOD_NS, PWM_PERIOD_NS * redLEDintensity / 100);
        err |= pwm_set_dt(&pwm_ledIR2cm, PWM_PERIOD_NS, PWM_PERIOD_NS * irLEDintensity / 100);
        // LED3 
        // TODO is this present on the new board?
        // TODO check the pin outs for these
        err |= pwm_set_dt(&pwm_ledRED3cm, PWM_PERIOD_NS, PWM_PERIOD_NS * redLEDintensity / 100);
        err |= pwm_set_dt(&pwm_ledIR3cm, PWM_PERIOD_NS, PWM_PERIOD_NS * irLEDintensity / 100);

        if (err) {
            LOG_ERR("pwm_set_dt returned error %d", err);
            return;
        }
        k_msleep(200);
    }
}


/* Sensor worker thread: configures AS7341 and sends periodic BLE notifications. */
void thread_sensor(void)
{
	spectrumSensor.begin();
	// t = (ATIME+1)x(ASTEP+1)x2.78us
	spectrumSensor.setASTEP(599); // recommended value 599
	spectrumSensor.setATIME(59);  // recommended value 29
    // TODO 29 or 59?
	spectrumSensor.setSMUXLowChannels(false);
	spectrumSensor.enableSpectralMeasurement(true);

	while (1) {
		if (bleConnected) {
			spectrumSensor.readCurrentMUXChannels(sensordata);
			// my_lbs_send_sensor_notify_RED_NIRS(sensordata[2], sensordata[3], sensordata[5]);
			my_lbs_send_sensor_notify_RED_NIRS_LEDindex(sensordata[2], sensordata[3], sensordata[5], LEDlocation);
		}
		// LOG_INF("%d: \t%d, \t%d", hertz, sensordata[2], sensordata[5]);
		k_msleep(BLEtransferInterval);
	}
}

/* Application entry point: initializes BLE/LBS and rotates LED source index. */
int main(void)
{
	LOG_INF("System starting up");
	int err; /* Return code holder for BLE/service initialization calls. */

#ifdef ADS1299
    // Input pin structures
    adsPinsType pinConfigs[NUM_OF_ADS1299_DEV];

    pinConfigs[0] = {
        .spiPins = {
            .csPin = ADS1299_SPI_CS_PIN,
            .misoPin = ADS1299_SPI_MISO_PIN,
            .mosiPin = ADS1299_SPI_MOSI_PIN,
            .clkPin = ADS1299_SPI_SCLK_PIN
        },
        .drdy = ADS1299_DRDY_PIN,
        .pwrdnResetPin = ADS1299_PWDN_RST_PIN,
        .start = ADS1299_START_PIN
    };

    if (initSpi(pinConfigs[0].spiPins) != ADS1299_SUCCESS) {
        LOG_ERR("ADS1299 SPI init failed");
        return -1;
    }

    // Initialize ADS1299 device(s)
    adsRetType status[NUM_OF_ADS1299_DEV][6] = {0};
    for (unsigned int i = 0U; i < NUM_OF_ADS1299_DEV; i++) {
        if (adsObj[i].getDevIdx() == 0xFF) {
            LOG_ERR("Error while creating ADS1299 device instance.\n");
            return -1;
        }
        status[i][0] = adsObj[i].ads1299_init(pinConfigs[i], NUM_OF_ADS_CH);
        if (status[i][0] != ADS1299_SUCCESS) {
            LOG_ERR("ADS1299 init failed (idx %u, err %d)", i, (int)status[i][0]);
            return -1;
        }
        adsObj[i].adsReset();
        k_msleep(150);
        status[i][1] = adsObj[i].issueActnCmd(ADS_WAKEUP);
        if (status[i][1] != ADS1299_SUCCESS) {
            LOG_ERR("ADS1299 wakeup failed (idx %u, err %d)", i, (int)status[i][1]);
            return -1;
        }
        k_msleep(150);
        status[i][2] = adsObj[i].issueActnCmd(ADS_SDATAC);
        if (status[i][2] != ADS1299_SUCCESS) {
            LOG_ERR("ADS1299 SDATAC failed (idx %u, err %d)", i, (int)status[i][2]);
            return -1;
        }
        k_msleep(1);
        // Check device type once after valid reset/wakeup sequence.
        status[i][3] = adsObj[i].checkDevType();
        if (status[i][3] != ADS1299_SUCCESS) {
            LOG_ERR("ADS1299 device-ID check failed (idx %u, err %d)", i, (int)status[i][3]);
            return -1;
        }
        // If required, change the resgiter value sto custom values within these lines:
        /*
            Change in the following way:
            adsObj[i].currentConfig[0u] = desired_value_for_that_register;
        */
        /* Initialize all ADS1299 registers to required values */
        status[i][4] = adsObj[i].adsInitRegs();
        if (status[i][4] != ADS1299_SUCCESS) {
            LOG_ERR("ADS1299 register init failed (idx %u, err %d)", i, (int)status[i][4]);
            return -1;
        }
        // Put to standbu mode after verification
        status[i][5] = adsObj[i].issueActnCmd(ADS_STANDBY);
        if (status[i][5] != ADS1299_SUCCESS) {
            LOG_ERR("ADS1299 standby failed (idx %u, err %d)", i, (int)status[i][5]);
            return -1;
        }
    }
#endif

	err = bt_enable(NULL); // Enable the Bluetooth LE stack
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return -1;
	}
	LOG_INF("Bluetooth initialized");
	bt_conn_cb_register(&connection_callbacks);

	err = my_lbs_init(&app_callbacks);
	if (err) {
		LOG_ERR("Failed to init LBS (err:%d)\n", err);
		return -1;
	}

    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd)); // Start advertising
    if (err && (err != -EALREADY))
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return -1;
    }
    LOG_INF("Advertising successfully started");

	while (1) {
		if (bleConnected) {
			// spectrumSensor.readAllChannels(sensordata);
			// my_lbs_send_sensor_notify_ALL(sensordata);
			LEDlocation++;
			if (LEDlocation == 3) { //0 1 2 반복  각각 세트 
				LEDlocation = 0;
			}
		    k_msleep(1000*ledLocationInterval);
		}
        k_msleep(100);
	}
	return 0;
}

/* Zephyr static thread definition for LED PWM control task. */
K_THREAD_DEFINE(threadled_id, THREAD_STACK_SIZE, thread_led, NULL, NULL, NULL,
					THREAD0_PRIORITY, 0, 0);
/* Zephyr static thread definition for AS7341 sampling/notification task. */
K_THREAD_DEFINE(threadsensor_id, THREAD_STACK_SIZE, thread_sensor, NULL, NULL, NULL,
					THREAD1_PRIORITY, 0, 0);
