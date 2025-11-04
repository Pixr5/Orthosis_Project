#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <stdint.h>
#include <stdlib.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_mac.h"
#include "nvs_flash.h"

// Motor PWM configuration
#define MOTOR_PIN     6    // PWM output 
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT  // 10-bit resolution (0-1023) cant go any higher at 20kHz
#define LEDC_FREQUENCY  200  // 200 Hz pwm frequency 
#define SECURITY_MARGIN  10   // Safety margin for remanant noise after low pass filter

// Solenoid control pin
#define SOLENOID_PIN    18

#define ADC_INPUT_PIN   4 // GPIO4
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_4   
#define ADC_ATTEN       ADC_ATTEN_DB_12

// Motor control constants
#define MAX_PWM        1023    // Maximum PWM duty cycle (10-bit)
#define ADC_MAX        4095    // Maximum ADC value (12-bit)


// UART configuration for receiving commands
#define UART_NUM        UART_NUM_0
#define UART_BUF_SIZE   1024

// BLE definitions
#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4

#define DEVICE_NAME          "ESP32 DO NOT CONNECT"
#define GATTS_TAG            "GATTS"

static uint8_t adv_config_done = 0;
static uint16_t gatts_if_global = 0;
static uint16_t conn_id_global = 0;
static uint16_t char_handle_global = 0;
static bool ble_connected = false;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

    // PID control parameters for later use

// float kp = 0.25f;              // Proportional gain
// float ki = 0.05f;              // Integral gain
// float error_sum = 0.0f;        // Integral term
// float last_error = 0.0f;       // Previous error
// float anti_windup_gain = 0.1f; // Anti-windup feedback gain


static const char *TAG = "motor_control";

// Global setpoint variable (shared between tasks)
static volatile int g_setpoint = 0;  // Default setpoint in ADC units (0-4095)

// Control mode: true = ADC passthrough, false = manual setpoint
static volatile bool g_use_adc = true;

// BLE GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~0x01);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed");
        } else {
            ESP_LOGI(GATTS_TAG, "Advertising started");
        }
        break;
    default:
        break;
    }
}

// BLE GATTS event handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server registered, app_id %d", param->reg.app_id);
        gatts_if_global = gatts_if;
        
        // Set device name
        esp_ble_gap_set_device_name(DEVICE_NAME);
        
        // Configure advertising data
        esp_ble_adv_data_t adv_data = {
            .set_scan_rsp = false,
            .include_name = true,
            .include_txpower = true,
            .min_interval = 0x0006,
            .max_interval = 0x0010,
            .appearance = 0x00,
            .manufacturer_len = 0,
            .p_manufacturer_data = NULL,
            .service_data_len = 0,
            .p_service_data = NULL,
            .service_uuid_len = 0,
            .p_service_uuid = NULL,
            .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
        };
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= 0x01;
        
        // Create GATT service
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id.inst_id = 0,
            .id.uuid.len = ESP_UUID_LEN_16,
            .id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID,
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        break;
        
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "Service created");
        esp_ble_gatts_start_service(param->create.service_handle);
        
        // Add characteristic
        esp_bt_uuid_t char_uuid;
        char_uuid.len = ESP_UUID_LEN_16;
        char_uuid.uuid.uuid16 = GATTS_CHAR_UUID;
        
        esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_ble_gatts_add_char(param->create.service_handle, &char_uuid,
                               ESP_GATT_PERM_READ, char_property, NULL, NULL);
        break;
        

    case ESP_GATTS_ADD_CHAR_EVT: {
    ESP_LOGI(GATTS_TAG, "Characteristic added, handle %d", param->add_char.attr_handle);
    char_handle_global = param->add_char.attr_handle;

    // Add the Client Characteristic Configuration Descriptor (CCCD)
    esp_bt_uuid_t descr_uuid;
    descr_uuid.len = ESP_UUID_LEN_16;
    descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

    esp_err_t err = esp_ble_gatts_add_char_descr(param->add_char.service_handle,&descr_uuid,ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to add CCCD descriptor: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(GATTS_TAG, "CCCD descriptor added");
    }
    break;
}
        
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Client connected, conn_id %d", param->connect.conn_id);
        conn_id_global = param->connect.conn_id;
        ble_connected = true;
        break;
        
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Client disconnected");
        ble_connected = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;
        
    default:
        break;
    }
}

// Function to send telemetry data via BLE
void send_ble_telemetry(const char *data)
{
    if (ble_connected && char_handle_global != 0) {
        esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global, char_handle_global,strlen(data), (uint8_t *)data, false);
    }
}

// Task to read UART commands
void uart_read_task(void *pvParameters)
{
    uint8_t data[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';  // Null terminate
            char *line = (char *)data;
            
            // Parse command: "SET,<adc_value>\n" or "SETP,<percent>\n" or "MODE,<ADC|MAN|1|0>\n"
            if (strncmp(line, "SET,", 4) == 0) {
                int new_setpoint = atoi(line + 4);
                // Clamp to valid range
                if (new_setpoint < 0) new_setpoint = 0;
                if (new_setpoint > ADC_MAX) new_setpoint = ADC_MAX;
                
                g_setpoint = new_setpoint;
                ESP_LOGI(TAG, "Setpoint updated to %d", g_setpoint);
            } else if (strncmp(line, "SETP,", 5) == 0) {
                int pct = atoi(line + 5);
                if (pct < 0) pct = 0;
                if (pct > 100) pct = 100;
                int new_setpoint = (pct * ADC_MAX) / 100;
                g_setpoint = new_setpoint;
                ESP_LOGI(TAG, "Setpoint (%%) updated to %d%% -> ADC %d", pct, g_setpoint);
            } else if (strncmp(line, "MODE,", 5) == 0) {
                char *mode = line + 5;
                if (strncmp(mode, "ADC", 3) == 0 || strncmp(mode, "1", 1) == 0) {
                    g_use_adc = true;
                    ESP_LOGI(TAG, "Mode set to ADC passthrough");
                } else if (strncmp(mode, "MAN", 3) == 0 || strncmp(mode, "0", 1) == 0) {
                    g_use_adc = false;
                    ESP_LOGI(TAG, "Mode set to Manual setpoint");
                }
            }
        }
    }
}

void app_main(void)
{
    // Initialize NVS (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    // Register callbacks
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    
    ESP_LOGI(TAG, "BLE initialized, device name: %s", DEVICE_NAME);


    // Configure UART for command reception
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    // Create UART read task
    xTaskCreate(uart_read_task, "uart_read_task", 2048, NULL, 10, NULL);
    // Configure solenoid control pin
    gpio_reset_pin(SOLENOID_PIN);
    gpio_set_direction(SOLENOID_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SOLENOID_PIN, 0);
    ESP_LOGI(TAG, "Solenoid configured on GPIO %d", SOLENOID_PIN);

    // Configure ADC oneshot driver for analog input
    
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT,};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {.bitwidth = ADC_BITWIDTH_12,.atten = ADC_ATTEN,};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));
    ESP_LOGI(TAG, "ADC configured on GPIO %d", ADC_INPUT_PIN);

    // Configure the LEDC timer for motor PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return;
    }

    // Configure the LEDC channel for motor PWM
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_PIN,
        .duty = 0,
        .hpoint = 0
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Motor PWM on GPIO %d, Solenoid on GPIO %d, ADC on GPIO %d", MOTOR_PIN, SOLENOID_PIN, ADC_INPUT_PIN);

    // Main control loop
    while (1) {
        // Read analog value from ADC
        int adc_reading = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_reading));

        // Control mode: either follow ADC reading or manual setpoint from GUI
        int setpoint_value = g_use_adc ? adc_reading : g_setpoint;
        
        // Direct setpoint to PWM mapping (12-bit setpoint to 10-bit PWM)
        int control_signal = (setpoint_value * MAX_PWM) / ADC_MAX;
        
        // Update motor PWM duty cycle (commanded)
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, control_signal));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        // Read back the actual duty from LEDC hardware (reflects GPIO6 output)
        int pwm_duty_read = (int)ledc_get_duty(LEDC_MODE, LEDC_CHANNEL);

        // Solenoid control: HIGH if motor is on, LOW if motor is off
        int solenoid_state = 0;
        if (pwm_duty_read > SECURITY_MARGIN) {
            gpio_set_level(SOLENOID_PIN, 1);
            solenoid_state = 1;
        } else {
            gpio_set_level(SOLENOID_PIN, 0);
            solenoid_state = 0;
        }

        // Telemetry over UART (CSV) with security prefix: TEL,timestamp_ms,setpoint,adc_reading,control_signal,solenoid_state
        int64_t t_us = esp_timer_get_time();
        long long t_ms = (long long)(t_us / 1000);
        // Telemetry reports measured PWM duty (pwm_duty_read) 
        char telemetry[128];
        snprintf(telemetry, sizeof(telemetry), "TEL,%lld,%d,%d,%d,%d\n", 
                t_ms, setpoint_value, adc_reading, pwm_duty_read, solenoid_state);
        
        printf("%s", telemetry);
        ESP_LOGI(TAG, "Setpoint=%d ADC=%d PWM_RD=%d SOL=%d", setpoint_value, adc_reading, pwm_duty_read, solenoid_state);
        
        // Send telemetry over BLE
        send_ble_telemetry(telemetry);


        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms = 100 Hz sample rate

   
    }
}