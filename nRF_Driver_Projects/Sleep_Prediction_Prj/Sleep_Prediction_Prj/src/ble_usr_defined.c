#include "ble_usr_defined.h"
#include "peer_manager_types.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "icm20649.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_log.h"
#include "usr_config.h"
#include "ads1299-x.h"
#include "nrf_log_ctrl.h"
#include "usr_config.h"
#include "nrf_delay.h"
#include <string.h>
#if defined(ICM20649)
#include "ble_icm.h"
#endif
#if defined(MAX30102_PRESENT) && (MAX30102_PRESENT == 1)
#include "ble_spo.h"
#include "max30102.h"
extern ble_spo_t m_spo;
#endif

/* PREDEFINED MACROS */
#define APP_ADV_INTERVAL                              64                                      /**< Fast advertising interval (in units of 0.625 ms; 64 = 40 ms). */
#define APP_ADV_DURATION                              18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO                         3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                          1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define MIN_CONN_INTERVAL                             MSEC_TO_UNITS(7.5, UNIT_1_25_MS)         /**< Minimum acceptable connection interval for high-throughput notifications. */
#define MAX_CONN_INTERVAL                             MSEC_TO_UNITS(15, UNIT_1_25_MS)          /**< Maximum acceptable connection interval for high-throughput notifications. */
#define SLAVE_LATENCY                                 0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                              MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY                APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY                 APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT                  3                                       /**< Number of attempts before giving up the connection parameter negotiation. */
#define SEC_PARAM_BOND                                1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                                0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                                0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                            0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES                     BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                                 0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE                        7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE                        16                                      /**< Maximum encryption key size. */

/* FUNCTION MACROS */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */

/* GLOBAL VARIABLES */
// m_conn_handle is used by some BLE modules (e.g., conn params). BLE_CONN_HANDLE_INVALID means "not connected".
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing encoded advertising data. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing encoded scan response data. */

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
/* YOUR_JOB: Declare all services structure your application is using BLE_XYZ_DEF(m_xyz); */
// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
// The UUIDs in m_adv_uuids are placed into the advertising payload so scanners can quickly see which services this peripheral exposes.
{
#if defined(ADS1299)
    {BLE_UUID_BIOPOTENTIAL_ECG_MEASUREMENT_SERVICE, BLE_UUID_TYPE_BLE}, // For ADS1299
#endif
#if defined(ICM20649)
    {BLE_UUID_BIOPOTENTIAL_IMU_MEASUREMENT_SERVICE, BLE_UUID_TYPE_BLE}, // For MAX30102
#endif   
#if defined(MAX30102_PRESENT) && (MAX30102_PRESENT == 1)
    {BLE_UUID_BIOPOTENTIAL_SPO_MEASUREMENT_SERVICE, BLE_UUID_TYPE_BLE}, // For MAX30102
#endif
#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
#endif
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


#if (PEER_MANAGER_ENABLED == 1)
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
        default:
            break;
    }
}
#endif /* #if (PEER_MANAGER_ENABLED == 1) */


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
APP_ERROR_CHECK(err_code);
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
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
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
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_connected = false;
#if defined(ICM20649)
            NRF_LOG_INFO("ICM sampling stopped (timer-based)");
            for (uint8_t i = 0; i < NUM_OF_ICM_DEVICES; i++)
            {
                m_icm[i].icm_sample_count = 0u;
            }
#endif
#if defined(MAX30102_PRESENT) && (MAX30102_PRESENT == 1)
            (void)max30102_reset();
            m_spo.conn_handle = BLE_CONN_HANDLE_INVALID;
            m_spo.spo_ch1_count = 0u;
            m_spo.spo_ch2_count = 0u;
#endif
#if defined(ADS1299)
            // Stop ADS1299 sampling
            // ADS1299 device in operating mode to supply clock to the other device(s) in daisy chain, so stop RDATAC mode to stop sampling on all devices in the daisy chain.
            ads1299_stop_rdatac(&m_eeg[0]); 
#if (NUM_OF_ADS1299 == 2)
            ads1299_stop_rdatac(&m_eeg[0]);
            // ads1299_standby(); // TODO put ADS1299 device 2 only in standby to save power
#endif /* #if (NUM_OF_ADS1299 == 2) */
            ads1299_stop_conv(); // Save power
#endif
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            m_connected = true;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
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
            // Pair with the device
            pm_conn_secure(m_conn_handle, false);
#endif /* #if (PEER_MANAGER_ENABLED == 1) */
#if defined(ICM20649)
            // Initialize ICM20649 with default 5 ms settings (timer-based sampling).
            err_code = icm20649_init(&m_icm20649[0], &m_twi, ICM20649_I2C_ADDR1);
            if (err_code == NRF_SUCCESS)
            {
                for (uint8_t i = 0; i < NUM_OF_ICM_DEVICES; i++)
                {
                    m_icm[i].icm_sample_count = 0u;
                }
                NRF_LOG_INFO("ICM20649 initialized (timer mode, 5 ms sample period)");
            }
            else
            {
                NRF_LOG_WARNING("ICM20649 init failed on connect: 0x%X", err_code);
            }
#endif
#if defined(MAX30102_PRESENT) && (MAX30102_PRESENT == 1)
            m_spo.conn_handle = m_conn_handle;
            err_code = max30102_init();
            if (err_code == NRF_SUCCESS)
            {
                err_code = max30102_clear_interrupts();
            }
            if (err_code == NRF_SUCCESS)
            {
                m_spo.spo_ch1_count = 0u;
                m_spo.spo_ch2_count = 0u;
                ble_spo_update_config(&m_spo, false);
                NRF_LOG_INFO("MAX30102 BLE initialized.");
            }
            else
            {
                NRF_LOG_WARNING("MAX30102 init failed on connect: 0x%X", err_code);
            }
#endif
#if defined(ADS1299)
#if (NUM_OF_ADS1299 == 2)
            ads1299_start_conv();
            ads1299_start_rdatac(&m_eeg[0]);
            ads1299_start_rdatac(&m_eeg[1]);
#else
            ads1299_soft_start_conversion(&m_eeg[0]);
            nrf_delay_us(10);
#endif
            ads1299_start_rdatac(&m_eeg[0]);
#endif
            NRF_LOG_INFO("Connected.");
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_2MBPS,
                .tx_phys = BLE_GAP_PHY_2MBPS,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            if (err_code != NRF_ERROR_NOT_SUPPORTED) {
                APP_ERROR_CHECK(err_code);
            }
        }
        break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 NULL,
                                                 0,
                                                 0);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("GATT system attributes initialized.");
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            m_connected = false;
            // Disconnect on GATT Client timeout event.
            NRF_LOG_INFO("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            m_connected = false;
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

    // Always route BLE events to ICM services so conn_handle is updated on connect/disconnect.
#if defined(ICM20649)
    extern ble_icm_t m_icm[];
    for (uint8_t i = 0; i < NUM_OF_ICM_DEVICES; i++)
    {
        ble_icm_on_ble_evt(&m_icm[i], (ble_evt_t *)p_ble_evt);
    }
#endif

#if defined(MAX30102_PRESENT) && (MAX30102_PRESENT == 1)
    extern ble_spo_t m_spo;
    ble_spo_on_ble_evt(&m_spo, (ble_evt_t *)p_ble_evt);
#endif
}
#define NRF_SDH_BLE_HVN_QUEUE_SIZE                    32
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
        ret_code_t err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
}


/*! @brief Function for initializing the Advertising functionality.
 * @return None.
 */
void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = m_adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters (fast advertising via APP_ADV_INTERVAL / APP_ADV_DURATION).
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION; // 18000
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = MSEC_TO_UNITS(40, UNIT_0_625_MS);   // fast - 40 ms advertisment;
    // NOTE: You can set this to longer duration to save battery

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
    // This line can be used to limit or extend the maximum BLE transmission power
    // TODO to be removed and/or reduced to save power
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, (uint16_t)m_adv_handle, 4);
    APP_ERROR_CHECK(err_code);
}
