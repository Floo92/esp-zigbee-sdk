/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee Sleepy end device Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 *
 *
 * Beschreibung: 
 * Das Programm erstellt mit einen ESP32 H2 (DevKit) ein über Zigbee erreichbares Licht-Gerät als End Device.
 * Der Controller befindet sich die meiste Zeit im light-sleep Modus und wacht zyklisch auf.
 * Wird ein Einschaltbefehl empfangen wird GPIO 12 high gesetzt. Bei einem Ausschaltbefehl wird GPIO 12 low gesetzt.
 * Zusammen mit einem FET als Low-Side-Schalter an GPIO 12 kann damit eine Last wie z.B. eine Lichterkette geschaltet werden. Alternatic kann direkt an GPIO 12 ein externer LED-Treiber betrieben werden.
 * Es sollten Li-Ion Zellen (AA) verwendet werden, da diese eine Konstante Spannung (1,5V) über die gesamte Entladekurve der Batterie liefern. 
 * Bei anderen Zellen würde die Spannung mit der Zeit langsam absinken und der Controller würde nicht mehr arbeiten können bevor die Batterien ganz leer sind.
 * 
 *
 * Versionsverlauf:
 * 13.12.2024, Florian, Erstellt 
 * 24.09.2025, Florian, LEDC-Modul für PWM hinzugefügt
 *
 *
 *
 *
 *
 */
#include "esp_check.h"
#include "hal/gpio_types.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_sleepy_end_device.h"
#include "switch_driver.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#endif
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define SLEEP_ENABLE            // aktiviert light sleep
#define GPIO_CONTROL            // verwendet GPIO high/low anstatt PWM

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (12) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 12) * 100% = 4096
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 4 kHz

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2 (rev < 1.2), ESP32P4 targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */

/**
 * @note Make sure set idf.py menuconfig in zigbee component as zigbee end device!
*/
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_SLEEP";

static switch_func_pair_t button_func_pair[] = {{CONFIG_GPIO_EXT1_WAKEUP_SOURCE, SWITCH_ONOFF_TOGGLE_CONTROL}};

static void ieee_cb(esp_zb_zdp_status_t zdo_status, esp_zb_zdo_ieee_addr_rsp_t *resp, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Response IEEE address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", resp->ieee_addr[7],
                 resp->ieee_addr[6], resp->ieee_addr[5], resp->ieee_addr[4], resp->ieee_addr[3], resp->ieee_addr[2],
                 resp->ieee_addr[1], resp->ieee_addr[0]);
    }
}

static void zb_buttons_handler(switch_func_pair_t* button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        /* Get the switch ieee address */
        esp_zb_zdo_ieee_addr_req_param_t ieee_req;
        ieee_req.addr_of_interest = 0x0;
        ieee_req.dst_nwk_addr = 0x0;
        ieee_req.request_type = 0;
        ieee_req.start_index = 0;
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zdo_ieee_addr_req(&ieee_req, ieee_cb, NULL);
        esp_zb_lock_release();
        ESP_EARLY_LOGI(TAG, "Send 'ieee_addr req' command");
    }
}

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    if (!is_inited) {
        ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler),
                            ESP_FAIL, TAG, "Failed to initialize switch driver");
    /* Configure RTC IO wake up:
    The configuration mode depends on your hardware design.
    Since the BOOT button is connected to a pull-up resistor, the wake-up mode is configured as LOW.
    */
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(BIT(CONFIG_GPIO_EXT1_WAKEUP_SOURCE), ESP_EXT1_WAKEUP_ANY_LOW));

#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_init(CONFIG_GPIO_EXT1_WAKEUP_SOURCE);
        rtc_gpio_pulldown_dis(CONFIG_GPIO_EXT1_WAKEUP_SOURCE);
        rtc_gpio_pullup_en(CONFIG_GPIO_EXT1_WAKEUP_SOURCE);
#else
        gpio_pulldown_dis(CONFIG_GPIO_EXT1_WAKEUP_SOURCE);
        gpio_pullup_en(CONFIG_GPIO_EXT1_WAKEUP_SOURCE);
#endif
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
      case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
#ifdef SLEEP_ENABLE
        ESP_LOGI(TAG, "Zigbee can sleep");
        esp_zb_sleep_now();
#endif
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                if(light_state == 1)                        // wenn Befehl Licht ein
                {
#ifdef GPIO_CONTROL
                    gpio_hold_dis(12);                      // GPIO Spannung halten deaktivieren
                    gpio_set_level(12, 1);                  // GPIO 12 high (FET bzw. Licht einschalten)
                    gpio_hold_en(12);                       // GPIO Spannung halten aktivieren
                    ESP_LOGI(TAG, "GPIO 12 high");
#else
                    // Set duty to X%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 4095));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    ESP_LOGI(TAG, "PWM higher");
#endif
                }
                else                                        // wenn Befehl Licht aus
                {
#ifdef GPIO_CONTROL
                    gpio_hold_dis(12);                      // GPIO Spannung halten deaktivieren
                    gpio_set_level(12, 0);                  // GPIO 12 high (FET bzw. Licht ausschalten)
                    gpio_hold_en(12);                       // GPIO Spannung halten aktivieren
                    ESP_LOGI(TAG, "GPIO 12 low");
#else
                    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
                    ESP_LOGI(TAG, "PWM inaktiv");
#endif
                }

                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack with Zigbee end-device config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
#ifdef SLEEP_ENABLE
    /* Enable zigbee light sleep */
    esp_zb_sleep_enable(true);
#endif
    esp_zb_init(&zb_nwk_cfg);
    /* set the on-off light device config */
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);
    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };

    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_on_off_light_ep, HA_ESP_LIGHT_ENDPOINT, &info);
    esp_zb_device_register(esp_zb_on_off_light_ep);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

esp_reset_reason_t reason = esp_reset_reason();

    printf("Reset reason: ");
    switch (reason) {
        case ESP_RST_POWERON:
            printf("Power-on reset\n");
            break;
        case ESP_RST_EXT:
            printf("External reset\n");
            break;
        case ESP_RST_SW:
            printf("Software reset\n");
            break;
        case ESP_RST_PANIC:
            printf("Exception/panic reset\n");
            break;
        case ESP_RST_INT_WDT:
            printf("Interrupt watchdog reset\n");
            break;
        case ESP_RST_TASK_WDT:
            printf("Task watchdog reset\n");
            break;
        case ESP_RST_WDT:
            printf("Other watchdog reset\n");
            break;
        case ESP_RST_BROWNOUT:
            printf("Brownout reset\n");
            break;
        case ESP_RST_SDIO:
            printf("SDIO reset\n");
            break;
        default:
            printf("Unknown reason\n");
            break;
    }

#ifdef GPIO_CONTROL
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL<<12;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(12, 0);
    gpio_hold_en(12);
    ESP_LOGI(TAG, "GPIO 12 low init");
#else
    // Set the LEDC peripheral configuration
    ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
#endif
    ESP_ERROR_CHECK(nvs_flash_init());
    /* esp zigbee light sleep initialization*/
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    /* load Zigbee platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
