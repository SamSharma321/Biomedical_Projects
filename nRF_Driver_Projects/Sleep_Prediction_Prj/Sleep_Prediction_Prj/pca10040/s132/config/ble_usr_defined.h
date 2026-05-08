#ifndef BLE_USR_DEFINED_
#define BLE_USR_DEFINED_ 
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "ble_dis.h"


/* UUID or ECG and IMU signals */
#define BLE_UUID_BIOPOTENTIAL_ECG_MEASUREMENT_SERVICE 0x0340                    /**< Random UUID for ECG service */
#define BLE_UUID_BIOPOTENTIAL_IMU_MEASUREMENT_SERVICE 0x1819                    /**< Random UUID for IMU service */
#ifndef BLE_UUID_BIOPOTENTIAL_SPO_MEASUREMENT_SERVICE
#define BLE_UUID_BIOPOTENTIAL_SPO_MEASUREMENT_SERVICE 0xAEC0                    /**< Random UUID for PPG/SpO2 service */
#endif


/* Extern variables */
/*! Variable to record connection with central device */
extern bool m_connected;

/* Function Prototypes for Linkage */
void conn_params_init(void);
void peer_manager_init(void);
void ble_stack_init(void);
void advertising_start(bool erase_bonds);
void advertising_init(void);
void gap_params_init(void);
void gatt_init(void);

#endif // BLE_USR_DEFINED_
