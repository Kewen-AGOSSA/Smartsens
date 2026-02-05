#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_ble_scan.h"
#include "ble_db_discovery.h"
#include "ble_nus_c.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "bsp.h"
#include "nrf_delay.h"
#include <string.h>
#include "bmp180.h"
#include "nrf_ble_gatt.h"
#include "max30102.h"
#include "nrfx_twi.h"
#include "MLX90614.h"

#define TARGET_NAME "My_BLE_PROJECT"
#define APP_BLE_CONN_CFG_TAG 1
#define APP_BLE_OBSERVER_PRIO 3

NRF_BLE_SCAN_DEF(m_scan);
BLE_NUS_C_DEF(m_nus_c);
BLE_DB_DISCOVERY_DEF(m_db_disc);

static bool m_nus_ready = false;
static bool m_ble_tx_ready = true;
char msg1[32], msg2[32];
static bool msg1_sent = false;

// Paramètres de connexion BLE acceptables pour éviter les déconnexions
static ble_gap_conn_params_t conn_params = {
    .min_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS), // 50 ms
    .max_conn_interval = MSEC_TO_UNITS(75, UNIT_1_25_MS), // 100 ms
    .slave_latency = 0,
    .conn_sup_timeout = MSEC_TO_UNITS(4000, UNIT_10_MS) // 4 s
};

NRF_BLE_GATT_DEF(m_gatt);

void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

void nus_c_evt_handler(ble_nus_c_t *p_nus, ble_nus_c_evt_t const *p_evt)
{
    NRF_LOG_INFO("NUS Client Event received.");
    bsp_board_led_on(BSP_BOARD_LED_1);

    switch (p_evt->evt_type)
    {
    case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
        NRF_LOG_INFO("NUS Discovery Complete");
        bsp_board_led_on(BSP_BOARD_LED_2);
        ble_nus_c_handles_assign(&m_nus_c, p_evt->conn_handle, &p_evt->handles);
        ble_nus_c_tx_notif_enable(&m_nus_c);

        m_nus_ready = true;
        NRF_LOG_INFO("NUS Notification enabled, ready to send.");

        break;

    case BLE_NUS_C_EVT_DISCONNECTED:
        NRF_LOG_WARNING("Disconnected from peripheral.");
        m_nus_ready = false;
        bsp_board_led_on(BSP_BOARD_LED_0);
        break;

    /*case BLE_NUS_C_EVT_NUS_TX_EVT:
        if (msg1_sent)
        {
            ret_code_t err = ble_nus_c_string_send(&m_nus_c, (uint8_t *)msg2, strlen(msg2));
            if (err == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Message 2 envoyé : %s", msg2);
                msg1_sent = false;
            }
            else
            {
                NRF_LOG_ERROR("Erreur envoi msg2 : %d", err);
            }
        }
        else
        {
            m_ble_tx_ready = true;
        }
        break;*/

    default:
        NRF_LOG_INFO("Unhandled NUS event type: %d", p_evt->evt_type);
        break;
    }
}

bool adv_report_has_target_name(const ble_gap_evt_adv_report_t *p_adv_report)
{
    const uint8_t *p_data = p_adv_report->data.p_data;
    uint8_t len = p_adv_report->data.len;

    for (uint32_t i = 0; i < len;)
    {
        uint8_t field_length = p_data[i];
        uint8_t field_type = p_data[i + 1];
        if (field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME)
        {
            if (memcmp(&p_data[i + 2], TARGET_NAME, strlen(TARGET_NAME)) == 0)
            {
                NRF_LOG_INFO("Target device name matched.");
                return true;
            }
        }
        i += field_length + 1;
    }

    return false;
}

void scan_evt_handler(scan_evt_t const *p_scan_evt)
{
    switch (p_scan_evt->scan_evt_id)
    {
    case NRF_BLE_SCAN_EVT_NOT_FOUND:
        bsp_board_led_on(BSP_BOARD_LED_0);
        if (adv_report_has_target_name(p_scan_evt->params.p_not_found))
        {
            NRF_LOG_INFO("Connecting to target device...");
            sd_ble_gap_connect(&p_scan_evt->params.p_not_found->peer_addr,
                               &m_scan.scan_params,
                               &conn_params,
                               APP_BLE_CONN_CFG_TAG);
        }
        break;

    case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        NRF_LOG_ERROR("Connection error during scan.");
        bsp_board_led_on(BSP_BOARD_LED_3);
        break;

    default:
        break;
    }
}

void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ble_db_discovery_on_ble_evt(p_ble_evt, &m_db_disc);
    ble_nus_c_on_ble_evt(p_ble_evt, &m_nus_c);

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected to peripheral.");
        bsp_board_led_off(BSP_BOARD_LED_0);
        bsp_board_led_on(BSP_BOARD_LED_1);
        ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_WARNING("Disconnected from peripheral.");
        bsp_board_led_on(BSP_BOARD_LED_0);

        break;
    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
    {
        NRF_LOG_INFO("Requête de mise à jour des paramètres reçue : min %d, max %d, latence %d, timeout %d",
                     p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.min_conn_interval,
                     p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.max_conn_interval,
                     p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.slave_latency,
                     p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.conn_sup_timeout);

        const ble_gap_evt_t *gap_evt = &p_ble_evt->evt.gap_evt;
        sd_ble_gap_conn_param_update(gap_evt->conn_handle, &gap_evt->params.conn_param_update_request.conn_params);
    }
    break;

    default:
        NRF_LOG_INFO("Unhandled BLE event: 0x%X", p_ble_evt->header.evt_id);
        break;
    }
}

void ble_stack_init(void)
{
    NRF_LOG_INFO("Initializing BLE Stack...");
    uint32_t ram_start = 0;
    nrf_sdh_enable_request();
    nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    nrf_sdh_ble_enable(&ram_start);
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

void scan_init(void)
{
    NRF_LOG_INFO("Initializing Scan Module...");

    nrf_ble_scan_init_t scan_init = {0};
    memcpy(&m_scan.conn_params, &conn_params, sizeof(ble_gap_conn_params_t));

    scan_init.connect_if_match = false;
    scan_init.p_scan_param = NULL;
    nrf_ble_scan_init(&m_scan, &scan_init, scan_evt_handler);
}

void nus_c_init(void)
{
    NRF_LOG_INFO("Initializing NUS Client...");
    ble_nus_c_init_t init = {0};
    init.evt_handler = nus_c_evt_handler;
    ble_nus_c_init(&m_nus_c, &init);
}

void ble_nus_c_db_disc_handler(ble_db_discovery_evt_t *p_evt)
{
    NRF_LOG_INFO("DB Discovery Event received.");
    ble_nus_c_on_db_disc_evt(&m_nus_c, p_evt);
}

int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_leds_off();
    nrf_delay_ms(100);

    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("==== CENTRAL START ====");
    NRF_LOG_FLUSH();

    ble_stack_init();
    scan_init();
    ble_db_discovery_init(ble_nus_c_db_disc_handler);
    nus_c_init();

    // --- FIX ajout UUID Vendor et enregistrement du service NUS ---
    static const ble_uuid128_t nus_base_uuid = {
        .uuid128 = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E}};
    uint8_t nus_uuid_type;
    ret_code_t err_code;

    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &nus_uuid_type);
    APP_ERROR_CHECK(err_code);

    ble_uuid_t nus_uuid;
    nus_uuid.type = nus_uuid_type;
    nus_uuid.uuid = BLE_UUID_NUS_SERVICE;

    err_code = ble_db_discovery_evt_register(&nus_uuid);
    APP_ERROR_CHECK(err_code);
    // ---------------------------------------------------------------

    NRF_LOG_INFO("Starting scan...");
    nrf_ble_scan_start(&m_scan);
    bmp180_init(); // Initialisation du capteur BMP180
    max30102_init();
    mlx90614_init();

    uint16_t ir, red;
    float amb, obj;

    while (true)
    {
        NRF_LOG_FLUSH();
        __WFE();

        if (m_nus_ready &&
            m_ble_tx_ready &&
            m_nus_c.conn_handle != BLE_CONN_HANDLE_INVALID &&
            m_nus_c.handles.nus_rx_handle != BLE_GATT_HANDLE_INVALID)
        {
            bool ok = max30102_read_fifo(&ir, &red);
            if (!ok)
            {
                NRF_LOG_WARNING("Lecture MAX30102 échouée");
                continue;
            }
            if (mlx90614_read_ambient_temp(&amb))
                NRF_LOG_INFO("Température ambiante : " NRF_LOG_FLOAT_MARKER "°C", NRF_LOG_FLOAT(amb));
            if (mlx90614_read_object_temp(&obj))
                NRF_LOG_INFO("Température objet IR : " NRF_LOG_FLOAT_MARKER "°C", NRF_LOG_FLOAT(obj));

            int32_t temp, press;
            bmp180_get_temperature_pressure(&temp, &press);

            // Formatage des deux messages
            snprintf(msg1, sizeof(msg1), "T:%ld.%dC P:%ldhPa",
                     temp / 10, abs(temp % 10), press / 100);
            
                     snprintf(msg2, sizeof(msg2), "IR:%u RED:%u", ir, red);

            ret_code_t err = ble_nus_c_string_send(&m_nus_c, (uint8_t *)msg1, strlen(msg1));
            nrf_delay_ms(7000); // tempo entre chaque lecture complète

            if (err == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Message 1 envoyé : %s", msg1);

                // Envoie directement le 2e message ici :
                err = ble_nus_c_string_send(&m_nus_c, (uint8_t *)msg2, strlen(msg2));
                if (err == NRF_SUCCESS)
                {
                    NRF_LOG_INFO("Message 2 envoyé : %s", msg2);
                }
                else
                {
                    NRF_LOG_ERROR("Erreur envoi msg2 : %d", err);
                }

                m_ble_tx_ready = false;
            }

        nrf_delay_ms(7000); // tempo entre chaque lecture complète
    }
}
}

