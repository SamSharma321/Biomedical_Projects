#include "ble_usr_defined.h"
#include "peer_manager_types.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_log.h"
#include "usr_config.h"
#include "ads1299-x.h"
#include "nrf_log_ctrl.h"
#include "usr_config.h"
#include "nrf_delay.h"
#if defined(AS7341)
#include "ble_as7341.h"
#endif
#if defined(LED_CONTROL)
#include "led.h"
#endif
#include <string.h>

#ifndef BLE_TX_RECOVERY_COOLDOWN_MS
#define BLE_TX_RECOVERY_COOLDOWN_MS 2000
#endif

#ifndef BLE_TX_DOWN_REQUIRES_STABLE_TX_MS
#define BLE_TX_DOWN_REQUIRES_STABLE_TX_MS 3000
#endif

#ifndef BLE_PHY_2MBPS_TX_DBM_MAX
#define BLE_PHY_2MBPS_TX_DBM_MAX -8
#endif

#ifndef BLE_LAST_PEER_FILTER_MS
#define BLE_LAST_PEER_FILTER_MS 10000
#endif

#if (BLE_DYNAMIC_PWR_CONTROL == 1) && (BLE_DYNAMIC_PWR_LOG_ENABLE == 1)
#define BLE_DYN_LOG_INFO(...) NRF_LOG_INFO(__VA_ARGS__)
#define BLE_DYN_LOG_WARNING(...) NRF_LOG_WARNING(__VA_ARGS__)
#else
#define BLE_DYN_LOG_INFO(...) do {} while (0)
#define BLE_DYN_LOG_WARNING(...) do {} while (0)
#endif

/* PREDEFINED MACROS */
#define APP_ADV_INTERVAL                              64                                      /**< Fast advertising interval (in units of 0.625 ms; 64 = 40 ms). */
#define APP_ADV_DURATION                              0                                       /**< The advertising duration in units of 10 milliseconds. 0 = advertise continuously. */
#define APP_BLE_OBSERVER_PRIO                         3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                          1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define MIN_CONN_INTERVAL                             MSEC_TO_UNITS(7.5, UNIT_1_25_MS)         /**< Minimum acceptable connection interval for high-throughput notifications. */
#define MAX_CONN_INTERVAL                             MSEC_TO_UNITS(15, UNIT_1_25_MS)          /**< Maximum acceptable connection interval for high-throughput notifications. */
#define SLAVE_LATENCY                                 0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                              MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY                APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY                 APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT                  3                                       /**< Number of attempts before giving up the connection parameter negotiation. */
#define BLE_PARAM_UPDATE_INTERVAL                     APP_TIMER_TICKS(BLE_PARAM_UPDATE_INTERVAL_MS) /**< RSSI update interval for optional BLE param characteristic. */
#define SEC_PARAM_BOND                                1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                                0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                                0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                            0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES                     BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                                 0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE                        7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE                        16                                      /**< Maximum encryption key size. */
#define NRF_SDH_BLE_HVN_QUEUE_SIZE                    32

/* FUNCTION MACROS */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */

/* GLOBAL VARIABLES */
// m_conn_handle is used by some BLE modules (e.g., conn params). BLE_CONN_HANDLE_INVALID means "not connected".
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static ble_gap_adv_params_t m_adv_params;                                        /**< Advertising parameters, updated before each start. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing encoded advertising data. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing encoded scan response data. */
static uint8_t m_ble_phy_current = BLE_GAP_PHY_1MBPS;
APP_TIMER_DEF(m_adv_filter_fallback_timer_id);
static bool m_adv_filter_fallback_timer_created = false;
#if (EXPOSE_BLE_PARAM == 1)
APP_TIMER_DEF(m_ble_param_timer_id);
static uint16_t m_ble_param_service_handle = BLE_GATT_HANDLE_INVALID;
static ble_gatts_char_handles_t m_ble_param_rssi_char_handles;
static int8_t m_ble_param_rssi_dbm = -127;
static uint8_t m_ble_param_rssi_channel_idx = 0;
static int8_t m_ble_param_tx_power_dbm = BLE_CONN_TX_POWER_INIT_DBM;
static int8_t m_ble_param_payload[2] = {-127, BLE_CONN_TX_POWER_INIT_DBM}; /* [0]=RSSI dBm, [1]=Tx power dBm */
static int32_t m_ble_param_rssi_accum = 0;
static uint16_t m_ble_param_rssi_samples = 0;
static uint32_t m_ble_tx_low_ms = 0;
static uint32_t m_ble_tx_high_ms = 0;
static uint32_t m_ble_tx_recovery_cooldown_ms = 0;
static uint32_t m_ble_tx_stable_ok_ms = 0;
static bool m_ble_tx_hvn_seen_since_tick = false;
static uint8_t m_ble_tx_power_level_idx = 0;
static int8_t const m_ble_tx_power_levels[] = {-40, -20, -16, -12, -8, -4, 0, 3, 4};
static bool ble_conn_tx_power_boost_steps(uint8_t steps);
#if (BLE_TX_TRACKER_ENABLE == 1)
typedef struct
{
    uint32_t hvn_tx_complete_pkts_total;
    uint32_t hvn_tx_complete_pkts_window;
    uint32_t boosts_total;
    uint32_t boosts_window;
    uint32_t low_rssi_windows_total;
    uint32_t low_rssi_windows_window;
    uint32_t disconnect_total;
    uint32_t disconnect_window;
    uint32_t gatt_timeout_total;
    uint32_t gatt_timeout_window;
    uint32_t no_tx_ok_ms;
    uint32_t recovery_attempts_total;
    uint32_t recovery_attempts_window;
    uint32_t recovery_success_total;
    uint32_t recovery_success_window;
    uint32_t log_elapsed_ms;
} ble_tx_tracker_t;

static ble_tx_tracker_t m_ble_tx_tracker = {0};
static uint8_t m_ble_last_disconnect_reason = 0;
#endif
#endif

/*! @brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    }
};
static void advertising_payload_encode(bool filtered_connect_req);
static void adv_filter_fallback_timeout_handler(void * p_context);

static void advertising_payload_encode(bool filtered_connect_req)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    ble_uuid_t adv_uuids[4];
    uint8_t adv_uuid_count = 0u;

#if defined(ADS1299)
    adv_uuids[adv_uuid_count].uuid = BLE_UUID_BIOPOTENTIAL_ECG_MEASUREMENT_SERVICE;
    adv_uuids[adv_uuid_count].type = BLE_UUID_TYPE_BLE;
    adv_uuid_count++;
#endif
#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
    adv_uuids[adv_uuid_count].uuid = BLE_UUID_BATTERY_SERVICE;
    adv_uuids[adv_uuid_count].type = BLE_UUID_TYPE_BLE;
    adv_uuid_count++;
#endif
#if defined(AS7341)
    extern ble_as7341_t m_as7341;
    if (m_as7341.uuid_type != BLE_UUID_TYPE_UNKNOWN)
    {
        adv_uuids[adv_uuid_count].uuid = BLE_UUID_AS7341_SERVICE;
        adv_uuids[adv_uuid_count].type = m_as7341.uuid_type;
        adv_uuid_count++;
    }
#endif
    adv_uuids[adv_uuid_count].uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE;
    adv_uuids[adv_uuid_count].type = BLE_UUID_TYPE_BLE;
    adv_uuid_count++;

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    // SoftDevice does not allow discoverable advertising with whitelist connect filtering.
    advdata.flags              = filtered_connect_req
                               ? BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED
                               : BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = adv_uuid_count;
    srdata.uuids_complete.p_uuids  = adv_uuids;

    m_adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    m_adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);
}


#if (PEER_MANAGER_ENABLED == 1)
static pm_peer_id_t m_last_peer_id = PM_PEER_ID_INVALID;                        /**< Last successfully secured peer. */

static void adv_filter_configure_for_last_peer(void)
{
    ret_code_t err_code;

    m_last_peer_id = PM_PEER_ID_INVALID;
    m_adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;

    err_code = pm_peer_ranks_get(&m_last_peer_id, NULL, NULL, NULL);
    if (err_code == NRF_ERROR_NOT_FOUND)
    {
        (void)pm_whitelist_set(NULL, 0);
        (void)pm_device_identities_list_set(NULL, 0);
        NRF_LOG_INFO("No bonded peer found. Advertising open for new pairing.");
        return;
    }
    APP_ERROR_CHECK(err_code);

    pm_peer_id_t const peers[] = {m_last_peer_id};
    err_code = pm_whitelist_set(peers, 1);
    APP_ERROR_CHECK(err_code);
    err_code = pm_device_identities_list_set(peers, 1);
    APP_ERROR_CHECK(err_code);

    m_adv_params.filter_policy = BLE_GAP_ADV_FP_FILTER_CONNREQ;
    NRF_LOG_INFO("Advertising filtered to last peer_id %d.", m_last_peer_id);
}

/*!
 * @brief Function for handling Peer Manager events.
 * @param[in] p_evt  Peer Manager event.
 * @return None.
 */
static void pm_evt_handler(pm_evt_t const * p_evt) // Only happens during peer manager initialization
{
    // Peer Manager event hook: pm_handler_on_pm_evt() handles common PM actions
    // pm_handler_flash_clean() runs flash garbage collection when needed.
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
    switch (p_evt->evt_id) {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            // Since the peer info has already been removed from flash, erase_bonds is set to false + begin advertisement
            advertising_start(false);
            break;
        case PM_EVT_CONN_SEC_SUCCEEDED:
            // Keep most recently secured peer as the preferred reconnection target.
            m_last_peer_id = p_evt->peer_id;
            break;
        default:
            break;
    }
}
#endif /* #if (PEER_MANAGER_ENABLED == 1) */

static void adv_filter_fallback_timeout_handler(void * p_context)
{
    ret_code_t err_code;
    UNUSED_PARAMETER(p_context);

    if (m_connected || (m_conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        return;
    }

    if (m_adv_params.filter_policy != BLE_GAP_ADV_FP_FILTER_CONNREQ)
    {
        return;
    }

#if (PEER_MANAGER_ENABLED == 1)
    err_code = sd_ble_gap_adv_stop(m_adv_handle);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err_code);
    }

    // Clear PM filter lists only after advertising is stopped.
    err_code = pm_whitelist_set(NULL, 0);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != BLE_ERROR_GAP_WHITELIST_IN_USE) &&
        (err_code != BLE_ERROR_GAP_INVALID_BLE_ADDR))
    {
        APP_ERROR_CHECK(err_code);
    }
    err_code = pm_device_identities_list_set(NULL, 0);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != BLE_ERROR_GAP_DEVICE_IDENTITIES_IN_USE) &&
        (err_code != BLE_ERROR_GAP_INVALID_BLE_ADDR) &&
        (err_code != NRF_ERROR_NOT_SUPPORTED))
    {
        APP_ERROR_CHECK(err_code);
    }
#else
    err_code = sd_ble_gap_adv_stop(m_adv_handle);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err_code);
    }
#endif

    m_adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    advertising_payload_encode(false);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Fallback to open advertising after %d ms last-peer filter window.", BLE_LAST_PEER_FILTER_MS);
}


/*! @brief Function for the GAP initialization.
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 * @return None.
 */
void gap_params_init(void) {
    // GAP parameters: sets the advertised device name and the preferred connection parameters (PPCP).
    // This project selects the device name based on ADS1299_REGDEFAULT_CONFIG1 (sampling rate setting).
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    /* YOUR_JOB: Use an appearance value matching the application's use case. */
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR); // Just a random value for detecting a medical device for scanners
    APP_ERROR_CHECK(err_code);
    /* Allocate a memory for the gap parameters */
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;
    // PPCP - Peripheral Preferred Connection Parameters: communicate the BLE settings to the 
    // central device (above parameters to establish a proper connections)
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code); // Error in initializing GAP parameters
}


/*! @brief Function for initializing the GATT (Generic Attribute Profile - dictates the data structure of the communication) module. */
/*! @return None. */
void gatt_init(void)
{
    // Initializes the nrf_ble_gatt module (MTU negotiation and related GATT settings).
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/*! @brief Function for handling a Connection Parameters error.
 * @param[in] nrf_error  Error code containing information about what went wrong.
 * @return None.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/*! @brief   Function for handling the Connection Parameters Module.
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 * @return None.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        // Keep the link alive with existing parameters instead of force-disconnecting.
        NRF_LOG_WARNING("Connection parameter update failed; keeping current link parameters.");
    }
}


/*
 * @brief Function for initializing the Connection Parameters module.
 * @return None.
 */
void conn_params_init(void)
{
    // Connection Parameters module: after connect, it requests updates to match MIN/MAX_CONN_INTERVAL etc.
    // cp_init.p_conn_params = NULL means it uses the PPCP set in gap_params_init() via sd_ble_gap_ppcp_set().
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

#if (EXPOSE_BLE_PARAM == 1)
static bool ble_param_rssi_notify_is_enabled(void)
{
    uint8_t cccd_value_buf[2] = {0};
    ble_gatts_value_t cccd_value;
    ret_code_t err_code;

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return false;
    }

    memset(&cccd_value, 0, sizeof(cccd_value));
    cccd_value.len = sizeof(cccd_value_buf);
    cccd_value.p_value = cccd_value_buf;
    err_code = sd_ble_gatts_value_get(m_conn_handle,
                                      m_ble_param_rssi_char_handles.cccd_handle,
                                      &cccd_value);
    if (err_code != NRF_SUCCESS)
    {
        return false;
    }

    return ble_srv_is_notification_enabled(cccd_value_buf);
}

#if (BLE_TX_TRACKER_ENABLE == 1)
static void ble_tx_tracker_log_if_due(void)
{
    if (m_ble_tx_tracker.log_elapsed_ms < BLE_TX_TRACKER_LOG_INTERVAL_MS)
    {
        return;
    }

    BLE_DYN_LOG_INFO("BLE tracker: tx_ok=%u boosts=%u low_rssi=%u disc=%u",
                     m_ble_tx_tracker.hvn_tx_complete_pkts_window,
                     m_ble_tx_tracker.boosts_window,
                     m_ble_tx_tracker.low_rssi_windows_window,
                     m_ble_tx_tracker.disconnect_window);
    BLE_DYN_LOG_INFO("BLE tracker: recov_try=%u recov_ok=%u",
                     m_ble_tx_tracker.recovery_attempts_window,
                     m_ble_tx_tracker.recovery_success_window);
    BLE_DYN_LOG_INFO("BLE tracker: last_disc=0x%X gatt_to=%u no_tx_ok_ms=%u tx_dbm=%d rssi=%d",
                     m_ble_last_disconnect_reason,
                     m_ble_tx_tracker.gatt_timeout_window,
                     m_ble_tx_tracker.no_tx_ok_ms,
                     m_ble_param_tx_power_dbm,
                     m_ble_param_rssi_dbm);

    m_ble_tx_tracker.hvn_tx_complete_pkts_window = 0;
    m_ble_tx_tracker.boosts_window = 0;
    m_ble_tx_tracker.low_rssi_windows_window = 0;
    m_ble_tx_tracker.disconnect_window = 0;
    m_ble_tx_tracker.gatt_timeout_window = 0;
    m_ble_tx_tracker.recovery_attempts_window = 0;
    m_ble_tx_tracker.recovery_success_window = 0;
    m_ble_tx_tracker.log_elapsed_ms = 0;
}
#endif

static void ble_param_rssi_accumulate(int8_t rssi_dbm)
{
    m_ble_param_rssi_accum += (int32_t)rssi_dbm;
    if (m_ble_param_rssi_samples < 0xFFFFu)
    {
        m_ble_param_rssi_samples++;
    }
}

static uint8_t ble_param_tx_power_level_find(int8_t dbm)
{
    uint8_t best_idx = 0;
    int16_t best_err = 32767;
    for (uint8_t i = 0; i < (uint8_t)(sizeof(m_ble_tx_power_levels) / sizeof(m_ble_tx_power_levels[0])); i++)
    {
        int16_t err = (int16_t)m_ble_tx_power_levels[i] - (int16_t)dbm;
        if (err < 0)
        {
            err = (int16_t)(-err);
        }
        if (err < best_err)
        {
            best_err = err;
            best_idx = i;
        }
    }
    return best_idx;
}

static uint8_t ble_phy_target_from_tx_dbm(int8_t tx_dbm)
{
    /* Higher TX power budget can sustain higher PHY throughput.
     * As TX power is reduced, fall back to 1 Mbps for robustness. */
    if (tx_dbm > BLE_PHY_2MBPS_TX_DBM_MAX)
    {
        return BLE_GAP_PHY_2MBPS;
    }
    return BLE_GAP_PHY_1MBPS;
}

static void ble_phy_apply_from_tx_power(int8_t tx_dbm)
{
    ret_code_t err_code;
    uint8_t target_phy;
    ble_gap_phys_t phys;

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    target_phy = ble_phy_target_from_tx_dbm(tx_dbm);
    if (target_phy == m_ble_phy_current)
    {
        return;
    }

    memset(&phys, 0, sizeof(phys));
    phys.rx_phys = target_phy;
    phys.tx_phys = target_phy;
    err_code = sd_ble_gap_phy_update(m_conn_handle, &phys);
    if (err_code == NRF_SUCCESS)
    {
        m_ble_phy_current = target_phy;
        BLE_DYN_LOG_INFO("PHY policy: TX=%d dBm -> %s",
                         tx_dbm,
                         (target_phy == BLE_GAP_PHY_2MBPS) ? "2 Mbps" : "1 Mbps");
    }
    else if ((err_code != NRF_ERROR_BUSY) &&
             (err_code != NRF_ERROR_INVALID_STATE) &&
             (err_code != NRF_ERROR_NOT_SUPPORTED))
    {
        BLE_DYN_LOG_WARNING("PHY policy update failed: 0x%X", err_code);
    }
}

static ret_code_t ble_param_tx_power_apply_index(uint8_t level_idx)
{
    ret_code_t err_code;
    int8_t tx_dbm;
    uint8_t max_idx = (uint8_t)(sizeof(m_ble_tx_power_levels) / sizeof(m_ble_tx_power_levels[0])) - 1u;
    uint8_t min_idx = ble_param_tx_power_level_find(BLE_CONN_TX_POWER_MIN_DBM);

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (level_idx > max_idx)
    {
        level_idx = max_idx;
    }
    if (level_idx < min_idx)
    {
        level_idx = min_idx;
    }

    tx_dbm = m_ble_tx_power_levels[level_idx];
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, tx_dbm);
    if (err_code == NRF_SUCCESS)
    {
        m_ble_tx_power_level_idx = level_idx;
        m_ble_param_tx_power_dbm = tx_dbm;
        ble_phy_apply_from_tx_power(tx_dbm);
    }
    else
    {
        BLE_DYN_LOG_WARNING("Conn TX power set failed: err=0x%X req_dbm=%d idx=%u conn=0x%X",
                            err_code, tx_dbm, level_idx, m_conn_handle);
    }
    return err_code;
}

static void ble_param_tx_power_adapt(int8_t rssi_avg_dbm)
{
#if (BLE_ADAPTIVE_TX_POWER_ENABLE == 1)
    bool super_good = (rssi_avg_dbm >= BLE_TX_POWER_SUPER_GOOD_RSSI_DBM);
    bool above_acceptable = (rssi_avg_dbm >= BLE_TX_POWER_ACCEPTABLE_RSSI_DBM);
    bool low = (rssi_avg_dbm <= BLE_TX_POWER_LOW_RSSI_DBM);

    if (rssi_avg_dbm <= BLE_TX_POWER_LOW_RSSI_DBM)
    {
        if (m_ble_tx_low_ms < 0xFFFFFFFFu - BLE_PARAM_UPDATE_INTERVAL_MS)
        {
            m_ble_tx_low_ms += BLE_PARAM_UPDATE_INTERVAL_MS;
        }
#if (BLE_TX_TRACKER_ENABLE == 1)
        m_ble_tx_tracker.low_rssi_windows_total++;
        m_ble_tx_tracker.low_rssi_windows_window++;
#endif
    }
    else
    {
        m_ble_tx_low_ms = 0;
    }

    if (above_acceptable)
    {
        if (m_ble_tx_high_ms < 0xFFFFFFFFu - BLE_PARAM_UPDATE_INTERVAL_MS)
        {
            m_ble_tx_high_ms += BLE_PARAM_UPDATE_INTERVAL_MS;
        }
    }
    else
    {
        m_ble_tx_high_ms = 0;
    }

    if (low &&
        (m_ble_tx_low_ms >= BLE_TX_POWER_LOW_HOLD_MS) &&
        (m_ble_tx_power_level_idx + 1u < (uint8_t)(sizeof(m_ble_tx_power_levels) / sizeof(m_ble_tx_power_levels[0]))))
    {
        ble_conn_tx_power_boost_steps(BLE_TX_POWER_LOW_BOOST_STEPS);
        m_ble_tx_low_ms = 0;
        m_ble_tx_high_ms = 0;
        m_ble_tx_recovery_cooldown_ms = BLE_TX_RECOVERY_COOLDOWN_MS;
        BLE_DYN_LOG_INFO("Adaptive TX power UP (+%u step) -> %d dBm (RSSI avg %d dBm)",
                         BLE_TX_POWER_LOW_BOOST_STEPS,
                         m_ble_param_tx_power_dbm,
                         rssi_avg_dbm);
    }
    else if ((m_ble_tx_recovery_cooldown_ms == 0u) &&
             (m_ble_tx_stable_ok_ms >= BLE_TX_DOWN_REQUIRES_STABLE_TX_MS) &&
             super_good &&
             above_acceptable &&
             (m_ble_tx_high_ms >= BLE_TX_POWER_SUPER_GOOD_STEP_MS) &&
             (m_ble_tx_power_level_idx > 0u))
    {
        // Aggressively step down power while RSSI is super good to minimize current draw.
        ble_param_tx_power_apply_index((uint8_t)(m_ble_tx_power_level_idx - 1u));
        m_ble_tx_high_ms = 0;
        m_ble_tx_low_ms = 0;
        BLE_DYN_LOG_INFO("Adaptive TX power DOWN (super good) -> %d dBm (RSSI avg %d dBm)",
                         m_ble_param_tx_power_dbm, rssi_avg_dbm);
    }
    else if ((m_ble_tx_recovery_cooldown_ms == 0u) &&
             (m_ble_tx_stable_ok_ms >= BLE_TX_DOWN_REQUIRES_STABLE_TX_MS) &&
             above_acceptable &&
             (m_ble_tx_high_ms >= BLE_TX_POWER_HIGH_HOLD_MS) &&
             (m_ble_tx_power_level_idx > 0u))
    {
        ble_param_tx_power_apply_index((uint8_t)(m_ble_tx_power_level_idx - 1u));
        m_ble_tx_high_ms = 0;
        m_ble_tx_low_ms = 0;
        BLE_DYN_LOG_INFO("Adaptive TX power DOWN -> %d dBm (RSSI avg %d dBm)", m_ble_param_tx_power_dbm, rssi_avg_dbm);
    }
#else
    UNUSED_PARAMETER(rssi_avg_dbm);
#endif
}

static void ble_param_rssi_refresh_from_softdevice(void)
{
    int8_t current_rssi = m_ble_param_rssi_dbm;
    ret_code_t err_code;

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    err_code = sd_ble_gap_rssi_get(m_conn_handle, &current_rssi, &m_ble_param_rssi_channel_idx);
    if (err_code == NRF_SUCCESS)
    {
        m_ble_param_rssi_dbm = current_rssi;
        ble_param_rssi_accumulate(current_rssi);
    }
}

static void ble_param_rssi_update_and_notify(bool notify)
{
    ret_code_t err_code;
    uint16_t   hvx_len;
    ble_gatts_value_t   gatts_value;
    ble_gatts_hvx_params_t hvx_params;

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    memset(&gatts_value, 0, sizeof(gatts_value));
    m_ble_param_payload[0] = m_ble_param_rssi_dbm;
    m_ble_param_payload[1] = m_ble_param_tx_power_dbm;
    gatts_value.len = sizeof(m_ble_param_payload);
    gatts_value.p_value = (uint8_t *)m_ble_param_payload;
    err_code = sd_ble_gatts_value_set(m_conn_handle,
                                      m_ble_param_rssi_char_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return;
    }

    if (!notify || !ble_param_rssi_notify_is_enabled())
    {
        return;
    }

    hvx_len = sizeof(m_ble_param_payload);
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = m_ble_param_rssi_char_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.p_data = (uint8_t *)m_ble_param_payload;
    hvx_params.p_len  = &hvx_len;

    err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static bool ble_conn_tx_power_boost_steps(uint8_t steps)
{
#if (EXPOSE_BLE_PARAM == 1)
#if (BLE_ADAPTIVE_TX_POWER_ENABLE == 1)
    uint8_t max_idx = (uint8_t)(sizeof(m_ble_tx_power_levels) / sizeof(m_ble_tx_power_levels[0])) - 1u;
    uint8_t target_idx;
    ret_code_t err_code;

    if ((m_conn_handle == BLE_CONN_HANDLE_INVALID) || (steps == 0u))
    {
        return false;
    }

    target_idx = m_ble_tx_power_level_idx;
    while ((steps-- > 0u) && (target_idx < max_idx))
    {
        target_idx++;
    }

    if (target_idx == m_ble_tx_power_level_idx)
    {
        return false;
    }

    err_code = ble_param_tx_power_apply_index(target_idx);
    if (err_code == NRF_SUCCESS)
    {
        m_ble_tx_low_ms = 0;
        m_ble_tx_high_ms = 0;
        m_ble_tx_stable_ok_ms = 0;
        m_ble_tx_recovery_cooldown_ms = BLE_TX_RECOVERY_COOLDOWN_MS;
#if (BLE_TX_TRACKER_ENABLE == 1)
        m_ble_tx_tracker.boosts_total++;
        m_ble_tx_tracker.boosts_window++;
#endif
        BLE_DYN_LOG_WARNING("Emergency TX power boost -> %d dBm (idx %u)", m_ble_param_tx_power_dbm, m_ble_tx_power_level_idx);
        ble_param_rssi_update_and_notify(false);
        return true;
    }
    return false;
#else
    UNUSED_PARAMETER(steps);
    return false;
#endif
#else
    UNUSED_PARAMETER(steps);
    return false;
#endif
}

#if (BLE_TX_TRACKER_ENABLE == 1)
static bool ble_conn_tx_power_is_max(void)
{
#if (EXPOSE_BLE_PARAM == 1)
#if (BLE_ADAPTIVE_TX_POWER_ENABLE == 1)
    uint8_t max_idx = (uint8_t)(sizeof(m_ble_tx_power_levels) / sizeof(m_ble_tx_power_levels[0])) - 1u;
    return (m_ble_tx_power_level_idx >= max_idx);
#else
    return true;
#endif
#else
    return true;
#endif
}
#endif

#if (BLE_TX_TRACKER_ENABLE == 1)
static void ble_conn_tx_recovery_try(void)
{
    bool boosted;

    m_ble_tx_tracker.recovery_attempts_total++;
    m_ble_tx_tracker.recovery_attempts_window++;
    boosted = ble_conn_tx_power_boost_steps(BLE_TX_POWER_LOW_BOOST_STEPS);
    m_ble_tx_recovery_cooldown_ms = BLE_TX_RECOVERY_COOLDOWN_MS;
    if (boosted)
    {
        m_ble_tx_tracker.recovery_success_total++;
        m_ble_tx_tracker.recovery_success_window++;
    }
    if (boosted || ble_conn_tx_power_is_max())
    {
        m_ble_tx_tracker.no_tx_ok_ms = 0;
    }
}
#endif

static void ble_param_timeout_handler(void * p_context)
{
    int8_t rssi_avg_dbm;

    UNUSED_PARAMETER(p_context);
    ble_param_rssi_refresh_from_softdevice();

    if (m_ble_param_rssi_samples > 0u)
    {
        rssi_avg_dbm = (int8_t)(m_ble_param_rssi_accum / (int32_t)m_ble_param_rssi_samples);
        m_ble_param_rssi_accum = 0;
        m_ble_param_rssi_samples = 0;
        m_ble_param_rssi_dbm = rssi_avg_dbm;
    }
    else
    {
        rssi_avg_dbm = m_ble_param_rssi_dbm;
    }

    ble_param_tx_power_adapt(rssi_avg_dbm);
    BLE_DYN_LOG_INFO("RSSI avg(%d ms): %d dBm (ch %u), TX: %d dBm",
                     BLE_PARAM_UPDATE_INTERVAL_MS,
                     m_ble_param_rssi_dbm,
                     m_ble_param_rssi_channel_idx,
                     m_ble_param_tx_power_dbm);
    ble_param_rssi_update_and_notify(true);
    /* Keep adaptive-stability timing active regardless of tracker compile flag.
     * Otherwise TX power never steps down when BLE_TX_TRACKER_ENABLE == 0. */
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        if (m_ble_tx_hvn_seen_since_tick)
        {
            if (m_ble_tx_stable_ok_ms < 0xFFFFFFFFu - BLE_PARAM_UPDATE_INTERVAL_MS)
            {
                m_ble_tx_stable_ok_ms += BLE_PARAM_UPDATE_INTERVAL_MS;
            }
#if (BLE_TX_TRACKER_ENABLE == 1)
            m_ble_tx_tracker.no_tx_ok_ms = 0;
#endif
        }
        else
        {
            m_ble_tx_stable_ok_ms = 0;
#if (BLE_TX_TRACKER_ENABLE == 1)
            if (m_ble_tx_tracker.no_tx_ok_ms < 0xFFFFFFFFu - BLE_PARAM_UPDATE_INTERVAL_MS)
            {
                m_ble_tx_tracker.no_tx_ok_ms += BLE_PARAM_UPDATE_INTERVAL_MS;
            }
#endif
        }
        m_ble_tx_hvn_seen_since_tick = false;

        if (m_ble_tx_recovery_cooldown_ms > BLE_PARAM_UPDATE_INTERVAL_MS)
        {
            m_ble_tx_recovery_cooldown_ms -= BLE_PARAM_UPDATE_INTERVAL_MS;
        }
        else
        {
            m_ble_tx_recovery_cooldown_ms = 0;
        }

#if (BLE_TX_TRACKER_ENABLE == 1)
        if ((m_ble_tx_tracker.no_tx_ok_ms >= BLE_TX_RECOVERY_NO_TXOK_MS) &&
            (m_ble_tx_recovery_cooldown_ms == 0u))
        {
            BLE_DYN_LOG_WARNING("BLE tracker recovery: no tx_ok for %u ms, trying TX boost.",
                                BLE_TX_RECOVERY_NO_TXOK_MS);
            ble_conn_tx_recovery_try();
        }
#endif
    }

#if (BLE_TX_TRACKER_ENABLE == 1)
    if (m_ble_tx_tracker.log_elapsed_ms < 0xFFFFFFFFu - BLE_PARAM_UPDATE_INTERVAL_MS)
    {
        m_ble_tx_tracker.log_elapsed_ms += BLE_PARAM_UPDATE_INTERVAL_MS;
    }
    ble_tx_tracker_log_if_due();
#endif
}

void ble_param_service_init(void)
{
    ret_code_t err_code;
    ble_uuid_t service_uuid;
    ble_add_char_params_t add_char_params;

    BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_BLE_PARAM_SERVICE);
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &m_ble_param_service_handle);
    APP_ERROR_CHECK(err_code);

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_BLE_PARAM_RSSI_CHAR;
    add_char_params.uuid_type         = BLE_UUID_TYPE_BLE;
    m_ble_param_payload[0] = m_ble_param_rssi_dbm;
    m_ble_param_payload[1] = m_ble_param_tx_power_dbm;
    add_char_params.init_len          = sizeof(m_ble_param_payload);
    add_char_params.max_len           = sizeof(m_ble_param_payload);
    add_char_params.p_init_value      = (uint8_t *)m_ble_param_payload;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;
    add_char_params.read_access       = SEC_OPEN;

    err_code = characteristic_add(m_ble_param_service_handle,
                                  &add_char_params,
                                  &m_ble_param_rssi_char_handles);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_ble_param_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                ble_param_timeout_handler);
    APP_ERROR_CHECK(err_code);
}
#endif /* #if (EXPOSE_BLE_PARAM == 1) */


static void ads1299_reinit(void) {
    // Power up ADS1299 device
    ads1299_powerup();
    for (uint8_t idx = 0; idx < NUM_OF_ADS1299; idx++)
    {
        ble_eeg_t *p_m_eeg = &m_eeg[idx];        
        ads1299_wake(p_m_eeg);
        nrf_delay_ms(10);
        ads1299_stop_rdatac(p_m_eeg);
        ads1299_init_regs_default(p_m_eeg);
        p_m_eeg->eeg_ch1_count = 0;
        ads1299_soft_start_conversion(p_m_eeg);
        ads1299_start_rdatac(p_m_eeg);
        //NRF_LOG_FLUSH();
    }
}

static void ads1299_deinit(void) {
    // Power down ADS1299 device
    ads1299_powerdn();
}

/*! @brief Function for handling BLE events.
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 * @return None.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    // Central BLE event dispatcher: connect/disconnect actions + sensor power state management.
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
#if (PEER_MANAGER_ENABLED == 1)
            (void)app_timer_stop(m_adv_filter_fallback_timer_id);
#endif
#if (EXPOSE_BLE_PARAM == 1)
            (void)sd_ble_gap_rssi_stop(p_ble_evt->evt.gap_evt.conn_handle);
            (void)app_timer_stop(m_ble_param_timer_id);
            m_ble_param_rssi_accum = 0;
            m_ble_param_rssi_samples = 0;
            m_ble_tx_low_ms = 0;
            m_ble_tx_high_ms = 0;
            m_ble_tx_recovery_cooldown_ms = 0;
            m_ble_tx_stable_ok_ms = 0;
            m_ble_tx_hvn_seen_since_tick = false;
            m_ble_phy_current = BLE_GAP_PHY_1MBPS;
#if (BLE_TX_TRACKER_ENABLE == 1)
            m_ble_tx_tracker.no_tx_ok_ms = 0;
            m_ble_last_disconnect_reason = p_ble_evt->evt.gap_evt.params.disconnected.reason;
            m_ble_tx_tracker.disconnect_total++;
            m_ble_tx_tracker.disconnect_window++;
#endif
#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_connected = false;
            NRF_LOG_INFO("Disconnected. Reason: 0x%X", p_ble_evt->evt.gap_evt.params.disconnected.reason);
#if defined(LED_CONTROL)
            led_set_intensity(0u, 0u);
#endif
#if defined(ADS1299)
            // Stop ADS1299 sampling
            ads1299_deinit();
#endif
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
#if (PEER_MANAGER_ENABLED == 1)
            (void)app_timer_stop(m_adv_filter_fallback_timer_id);
#endif
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            m_connected = true;
#if defined(LED_CONTROL)
            led_set_intensity(LED_DEFAULT_RED_PERCENT, LED_DEFAULT_IR_PERCENT);
#endif
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
            {
                APP_ERROR_CHECK(err_code);
            }
#if (EXPOSE_BLE_PARAM == 1)
            m_ble_tx_power_level_idx = ble_param_tx_power_level_find(BLE_CONN_TX_POWER_INIT_DBM);
            ble_param_tx_power_apply_index(m_ble_tx_power_level_idx);
            m_ble_tx_recovery_cooldown_ms = 0;
            m_ble_tx_stable_ok_ms = 0;
            m_ble_tx_hvn_seen_since_tick = false;
#if (BLE_TX_TRACKER_ENABLE == 1)
            m_ble_tx_tracker.no_tx_ok_ms = 0;
#endif
            err_code = sd_ble_gap_rssi_start(m_conn_handle, 1, 0);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
            {
                NRF_LOG_WARNING("RSSI start failed: 0x%X", err_code);
            }
            err_code = app_timer_start(m_ble_param_timer_id, BLE_PARAM_UPDATE_INTERVAL, NULL);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
            {
                APP_ERROR_CHECK(err_code);
            }
            ble_param_rssi_refresh_from_softdevice();
            ble_param_rssi_update_and_notify(false);
#endif

            /* Request maximum data length to increase notification throughput. */
            {
                ble_gap_data_length_params_t const dl_params = {
                    .max_tx_octets = NRF_SDH_BLE_GAP_DATA_LENGTH,
                    .max_rx_octets = NRF_SDH_BLE_GAP_DATA_LENGTH,
                    .max_tx_time_us = BLE_GAP_DATA_LENGTH_AUTO,
                    .max_rx_time_us = BLE_GAP_DATA_LENGTH_AUTO,
                };
                err_code = sd_ble_gap_data_length_update(m_conn_handle, &dl_params, NULL);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_INVALID_STATE))
                {
                    NRF_LOG_WARNING("Data length update request failed: 0x%X", err_code);
                }
            }
#if (PEER_MANAGER_ENABLED == 1)
    #if (REQUEST_SECURITY_ON_CONNECT == 1)
            // Request link security/bonding on connect (optional).
            err_code = pm_conn_secure(m_conn_handle, false);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != NRF_ERROR_INVALID_STATE))
            {
                NRF_LOG_WARNING("pm_conn_secure failed: 0x%X", err_code);
            }
    #endif
#endif /* #if (PEER_MANAGER_ENABLED == 1) */
#if defined(ADS1299)
            ads1299_reinit();
#endif
            NRF_LOG_INFO("Connected.");
            break;

#if (EXPOSE_BLE_PARAM == 1)
        case BLE_GAP_EVT_RSSI_CHANGED:
            m_ble_param_rssi_dbm = p_ble_evt->evt.gap_evt.params.rssi_changed.rssi;
            ble_param_rssi_accumulate(m_ble_param_rssi_dbm);
            break;
#endif

#if (EXPOSE_BLE_PARAM == 1)
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
#if (BLE_TX_TRACKER_ENABLE == 1)
            m_ble_tx_tracker.hvn_tx_complete_pkts_total += p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count;
            m_ble_tx_tracker.hvn_tx_complete_pkts_window += p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count;
            m_ble_tx_tracker.no_tx_ok_ms = 0;
#endif
            m_ble_tx_hvn_seen_since_tick = true;
            break;
#endif

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys = {
                .tx_phys = BLE_GAP_PHY_AUTO,
                .rx_phys = BLE_GAP_PHY_AUTO
            };
            NRF_LOG_INFO("PHY update request (AUTO).");
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != NRF_ERROR_INVALID_STATE))
            {
                NRF_LOG_WARNING("PHY update request handling failed: 0x%X", err_code);
            }
        }
        break;

        case BLE_GAP_EVT_PHY_UPDATE:
            m_ble_phy_current = p_ble_evt->evt.gap_evt.params.phy_update.tx_phy;
            NRF_LOG_INFO("PHY updated: tx=%u rx=%u",
                         p_ble_evt->evt.gap_evt.params.phy_update.tx_phy,
                         p_ble_evt->evt.gap_evt.params.phy_update.rx_phy);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // Required before sending notifications/indications when system attributes are absent.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 NULL,
                                                 0,
                                                 0);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("System attributes set to defaults.");
            break;

        case BLE_GATTC_EVT_TIMEOUT:
#if (EXPOSE_BLE_PARAM == 1) && (BLE_TX_TRACKER_ENABLE == 1)
            m_ble_tx_tracker.gatt_timeout_total++;
            m_ble_tx_tracker.gatt_timeout_window++;
#endif
            // Disconnect on GATT Client timeout event.
            NRF_LOG_INFO("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
#if (EXPOSE_BLE_PARAM == 1) && (BLE_TX_TRACKER_ENABLE == 1)
            m_ble_tx_tracker.gatt_timeout_total++;
            m_ble_tx_tracker.gatt_timeout_window++;
#endif
            // Disconnect on GATT Server timeout event.
            NRF_LOG_INFO("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }

    // Always route BLE events to EEG services so conn_handle is updated on connect/disconnect.
#if defined(ADS1299)
    extern ble_eeg_t m_eeg[];
    for (uint8_t i = 0; i < NUM_OF_ADS1299; i++)
    {
        ble_eeg_on_ble_evt(&m_eeg[i], (ble_evt_t *)p_ble_evt);
    }
#endif

#if defined(AS7341)
    extern ble_as7341_t m_as7341;
    ble_as7341_on_ble_evt(&m_as7341, (ble_evt_t *)p_ble_evt);
#endif
}

// Register a handler for BLE events.
NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

/*! @brief Function for initializing the BLE stack.
 * @details Initializes the SoftDevice and the BLE event interrupt.
 * @return None.
 */
void ble_stack_init(void)
{
    // Initializes the SoftDevice (BLE stack) and configures roles/MTU/event length before enabling BLE.
    ret_code_t err_code;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Variable to add configurations for BLE
    ble_cfg_t ble_cfg;

    // Configure number of custom UUIDS.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = NRF_SDH_BLE_VS_UUID_COUNT; // Set to 2
    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
    // Configure the maximum number of connections.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT; //is 1
    
    /* The following lines saves RAM since the nRF board is a peripheral and not a central BLE device. 
    The nRF device by itself will not scan for device(s) to connect/pair. 
    That is the responsibility of central BE devices. */
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0; 
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count = 0;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

     // Configure the max ATT MTU?
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum event length
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320; // NRF_SDH_BLE_GAP_EVENT_LENGTH;
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count = BLE_GAP_CONN_COUNT_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the number of packets per connection event:
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = NRF_SDH_BLE_HVN_QUEUE_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(err_code);
}

#if (PEER_MANAGER_ENABLED == 1)
/*! @brief Function for the Peer Manager initialization.
 * @return None.
 */
void peer_manager_init(void)
{
    // Peer Manager handles pairing/bonding and stores keys in flash (FDS).
    // main() currently comments out peer_manager_init(), so bonding is disabled unless you enable that call.
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/*! @brief Clear bond information from persistent storage.
 * @return None.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("Erase bonds!");
    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}
#endif /* #if (PEER_MANAGER_ENABLED == 1) */

/*! @brief Function for starting advertising.
 * @param[in] erase_bonds  True to erase existing bonds before advertising.
 * @return None.
 */
void advertising_start(bool erase_bonds)
{
    // Starts advertising. If erase_bonds=true, it first clears stored bonds/peers (see delete_bonds/pm_evt_handler).
#if (PEER_MANAGER_ENABLED == 1)
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
#else
    UNUSED_PARAMETER(erase_bonds);
#endif /* PEER MANAGER */
    {
        ret_code_t err_code;
#if (PEER_MANAGER_ENABLED == 1)
        adv_filter_configure_for_last_peer();
#endif
        advertising_payload_encode(m_adv_params.filter_policy == BLE_GAP_ADV_FP_FILTER_CONNREQ);
        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
        APP_ERROR_CHECK(err_code);

        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,
                                           (uint16_t)m_adv_handle,
                                           BLE_ADV_TX_POWER_DBM);
        APP_ERROR_CHECK(err_code);

        err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_CONN_COUNT))
        {
            APP_ERROR_CHECK(err_code);
        }

#if (PEER_MANAGER_ENABLED == 1)
        if (err_code == NRF_SUCCESS)
        {
            (void)app_timer_stop(m_adv_filter_fallback_timer_id);
            if (m_adv_params.filter_policy == BLE_GAP_ADV_FP_FILTER_CONNREQ)
            {
                err_code = app_timer_start(m_adv_filter_fallback_timer_id,
                                           APP_TIMER_TICKS(BLE_LAST_PEER_FILTER_MS),
                                           NULL);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
        }
#endif
    }
}


/*! @brief Function for initializing the Advertising functionality.
 * @return None.
 */
void advertising_init(void)
{
    ret_code_t err_code;
    advertising_payload_encode(false);

    // Set advertising parameters (fast advertising via APP_ADV_INTERVAL / APP_ADV_DURATION).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    m_adv_params.duration        = APP_ADV_DURATION;
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = MSEC_TO_UNITS(40, UNIT_0_625_MS);   // fast - 40 ms advertisment;
    // NOTE: You can set this to longer duration to save battery

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
    // This line can be used to limit or extend the maximum BLE transmission power
    // TODO to be removed and/or reduced to save power
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, (uint16_t)m_adv_handle, BLE_ADV_TX_POWER_DBM);
    APP_ERROR_CHECK(err_code);

    if (!m_adv_filter_fallback_timer_created)
    {
        err_code = app_timer_create(&m_adv_filter_fallback_timer_id,
                                    APP_TIMER_MODE_SINGLE_SHOT,
                                    adv_filter_fallback_timeout_handler);
        APP_ERROR_CHECK(err_code);
        m_adv_filter_fallback_timer_created = true;
    }
}
