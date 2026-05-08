/* Created By: Sameera Sharma for BITN */
#ifndef BLE_USR_DEFINED_
#define BLE_USR_DEFINED_ 

#include <stdbool.h>

/* Dynamic BLE power/link control master switch. */
#ifndef BLE_DYNAMIC_PWR_CONTROL
#define BLE_DYNAMIC_PWR_CONTROL                        1
#endif

/* Dynamic BLE power/link logs on RTT. */
#ifndef BLE_DYNAMIC_PWR_LOG_ENABLE
#define BLE_DYNAMIC_PWR_LOG_ENABLE                     0
#endif

/* Expose dynamic BLE parameters (RSSI/TX) to client (BFEE characteristic). */
#ifndef BLE_DYNAMIC_PWR_EXPOSE_TO_CLIENT
#define BLE_DYNAMIC_PWR_EXPOSE_TO_CLIENT               0
#endif

/* Backward-compatible alias used in codebase for BFEE characteristic exposure. */
#ifndef EXPOSE_BLE_PARAM
#define EXPOSE_BLE_PARAM                               BLE_DYNAMIC_PWR_EXPOSE_TO_CLIENT
#endif
#if (BLE_DYNAMIC_PWR_CONTROL == 0)
#undef EXPOSE_BLE_PARAM
#define EXPOSE_BLE_PARAM                               0
#endif

/* Optional security request on every connect (can trigger disconnects with some centrals). */
#ifndef REQUEST_SECURITY_ON_CONNECT
#define REQUEST_SECURITY_ON_CONNECT                    0
#endif

/* UUID or ECG and IMU signals */
#define BLE_UUID_BIOPOTENTIAL_ECG_MEASUREMENT_SERVICE 0x0340                    /**< Random UUID for ECG service */
#define BLE_UUID_BIOPOTENTIAL_IMU_MEASUREMENT_SERVICE 0x1819                    /**< Random UUID for IMU service */
#define BLE_UUID_BLE_PARAM_SERVICE                    0x1CE0                    /**< Optional BLE parameter service UUID. */
#define BLE_UUID_BLE_PARAM_RSSI_CHAR                  0xBFEE                    /**< Optional BLE RSSI characteristic UUID (int8 dBm). */

/* BLE RSSI/Tx diagnostics and adaptive Tx power tuning.
 * Supported tx_power values for sd_ble_gap_tx_power_set (s132): -40, -20, -16, -12, -8, -4, 0, +3, +4 dBm.
 */
#ifndef BLE_PARAM_UPDATE_INTERVAL_MS
#define BLE_PARAM_UPDATE_INTERVAL_MS                  100
#endif

#ifndef BLE_ADAPTIVE_TX_POWER_ENABLE
#define BLE_ADAPTIVE_TX_POWER_ENABLE                  1
#endif

#ifndef BLE_TX_TRACKER_ENABLE
#define BLE_TX_TRACKER_ENABLE                         1
#endif

#ifndef BLE_TX_TRACKER_LOG_INTERVAL_MS
#define BLE_TX_TRACKER_LOG_INTERVAL_MS                1000
#endif

#ifndef BLE_TX_RECOVERY_NO_TXOK_MS
#define BLE_TX_RECOVERY_NO_TXOK_MS                    BLE_PARAM_UPDATE_INTERVAL_MS
#endif

#ifndef BLE_TX_RECOVERY_COOLDOWN_MS
#define BLE_TX_RECOVERY_COOLDOWN_MS                   2000
#endif

#ifndef BLE_CONN_TX_POWER_INIT_DBM
#define BLE_CONN_TX_POWER_INIT_DBM                    4
#endif

#ifndef BLE_CONN_TX_POWER_MIN_DBM
#define BLE_CONN_TX_POWER_MIN_DBM                     -20
#endif

#ifndef BLE_ADV_TX_POWER_DBM
#define BLE_ADV_TX_POWER_DBM                          4
#endif

#ifndef BLE_TX_POWER_LOW_RSSI_DBM
#define BLE_TX_POWER_LOW_RSSI_DBM                     -82
#endif

#ifndef BLE_TX_POWER_HIGH_RSSI_DBM
#define BLE_TX_POWER_HIGH_RSSI_DBM                    -60
#endif

#ifndef BLE_TX_POWER_ACCEPTABLE_RSSI_DBM
#define BLE_TX_POWER_ACCEPTABLE_RSSI_DBM              -68
#endif

#ifndef BLE_TX_POWER_SUPER_GOOD_RSSI_DBM
#define BLE_TX_POWER_SUPER_GOOD_RSSI_DBM              -54
#endif

#ifndef BLE_TX_POWER_LOW_HOLD_MS
#define BLE_TX_POWER_LOW_HOLD_MS                      1000
#endif

#ifndef BLE_TX_POWER_LOW_BOOST_STEPS
#define BLE_TX_POWER_LOW_BOOST_STEPS                  2
#endif

#ifndef BLE_TX_POWER_HIGH_HOLD_MS
#define BLE_TX_POWER_HIGH_HOLD_MS                     45000
#endif

#ifndef BLE_TX_POWER_SUPER_GOOD_STEP_MS
#define BLE_TX_POWER_SUPER_GOOD_STEP_MS               5000
#endif

#ifndef BLE_TX_DOWN_REQUIRES_STABLE_TX_MS
#define BLE_TX_DOWN_REQUIRES_STABLE_TX_MS             1000
#endif

#if (BLE_DYNAMIC_PWR_CONTROL == 0)
#undef BLE_ADAPTIVE_TX_POWER_ENABLE
#define BLE_ADAPTIVE_TX_POWER_ENABLE                  0
#undef BLE_TX_TRACKER_ENABLE
#define BLE_TX_TRACKER_ENABLE                         0
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
#if (EXPOSE_BLE_PARAM == 1)
void ble_param_service_init(void);
#endif

#endif // BLE_USR_DEFINED_
