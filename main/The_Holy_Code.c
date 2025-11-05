#include <stdio.h>
#include <string.h>
#include <math.h>
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
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
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
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT  // 13-bit resolution (0-8191)
#define SECURITY_MARGIN  20   // Safety margin for remanant noise after low pass filter

// Solenoid control pin
#define SOLENOID_PIN    18

#define ADC_INPUT_PIN   4 // GPIO4
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_4   
#define ADC_ATTEN       ADC_ATTEN_DB_12

// Continuous ADC configuration optimized for 5Hz analog filter
#define ADC_READ_LEN            256     // DMA buffer length
#define ADC_CONV_FRAME_SIZE     64      // Conversions per frame  
#define ADC_SAMPLE_FREQ_HZ      20000   // 20kHz sampling (minimum for continuous mode)

// Motor control constants
#define MAX_PWM        8191    // Maximum PWM duty cycle (13-bit)
#define ADC_MAX        4095    // Maximum ADC value (12-bit)


// UART configuration for receiving commands
#define UART_NUM        UART_NUM_0
#define UART_BUF_SIZE   1024

// Multi-rate control system frequencies
#define MOTOR_CONTROL_FREQ_HZ   1000    // 1kHz for fast motor response
#define ADC_SAMPLING_FREQ_HZ    500     // 500Hz for ADC readings
#define TELEMETRY_FREQ_HZ       200     // 200Hz for smoother telemetry (reduced step appearance)
#define BLE_UPDATE_FREQ_HZ      50      // 50Hz for BLE notifications
#define LEDC_FREQUENCY  1000  // 1 kHz pwm frequency for smooth control

// Convert to tick periods
#define MOTOR_CONTROL_PERIOD_MS (1000 / MOTOR_CONTROL_FREQ_HZ)
#define ADC_SAMPLING_PERIOD_MS  (1000 / ADC_SAMPLING_FREQ_HZ)
#define TELEMETRY_PERIOD_MS     (1000 / TELEMETRY_FREQ_HZ)
#define BLE_UPDATE_PERIOD_MS    (1000 / BLE_UPDATE_FREQ_HZ)

// Configurable frequencies structure
typedef struct {
    uint16_t motor_control_hz;
    uint16_t adc_sampling_hz;
    uint16_t telemetry_hz;
    uint16_t ble_update_hz;
    bool auto_adjust;
} frequency_config_t;

// Shared data structure with thread-safe access
typedef struct {
    float adc_reading;
    float setpoint_value;
    int pwm_output;
    int solenoid_state;
    bool new_adc_data;
    bool new_control_output;
    bool adc_mode;  // true = ADC passthrough, false = manual setpoint
    SemaphoreHandle_t data_mutex;
} shared_data_t;

// BLE definitions
#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4

#define DEVICE_NAME          "ESP32_Motor"
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

// PI Controller Structure (No Derivative for Motor Control)
typedef struct {
    float kp;                 // Proportional gain
    float ki;                 // Integral gain  
    float setpoint;           // Desired value
    float integral;           // Integral accumulator
    float output_min;         // Minimum output limit
    float output_max;         // Maximum output limit
    float dt;                 // Nominal sample time (seconds)
    bool anti_windup_enabled; // Anti-windup protection
    uint64_t last_update_us;  // Last update timestamp for adaptive dt
} pi_controller_t;

// Global PI controller instance
static pi_controller_t motor_pi;

// Global frequency configuration
static frequency_config_t g_freq_config = {
    .motor_control_hz = MOTOR_CONTROL_FREQ_HZ,
    .adc_sampling_hz = ADC_SAMPLING_FREQ_HZ,
    .telemetry_hz = TELEMETRY_FREQ_HZ,
    .ble_update_hz = BLE_UPDATE_FREQ_HZ,
    .auto_adjust = false
};

// Global shared data
static shared_data_t g_shared_data;

// Continuous ADC global variables for DMA operation
static adc_continuous_handle_t adc_handle = NULL;
static volatile int32_t g_latest_adc_reading = 0;
static volatile bool g_adc_data_ready = false;

// Task handles for frequency control
static TaskHandle_t adc_task_handle = NULL;
static TaskHandle_t motor_task_handle = NULL;
static TaskHandle_t telemetry_task_handle = NULL;
static TaskHandle_t ble_task_handle = NULL;

// Function declarations
void send_ble_telemetry(const char *data);

// Continuous ADC callback for DMA conversion completion placed in IRAM for speed
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    
    // Process the ADC data buffer
    uint8_t *buffer = edata->conv_frame_buffer;
    uint32_t length = edata->size;
    
    // Parse ADC results from DMA buffer
    for (int i = 0; i < length; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&buffer[i];
        
        // Check if this is our target channel (ESP32-C3 uses type2 format)
        if (p->type2.channel == ADC_CHANNEL && p->type2.unit == ADC_UNIT) {
            uint32_t raw_data = p->type2.data;
            if (raw_data > 0) {  // Valid reading
                g_latest_adc_reading = raw_data;
                g_adc_data_ready = true;
            }
        }
    }
    
    return mustYield == pdTRUE;
}

// PI Controller Functions
void pi_init(pi_controller_t *pi, float kp, float ki, float dt, float output_min, float output_max) {
    pi->kp = kp;
    pi->ki = ki;
    pi->dt = dt;
    pi->output_min = output_min;
    pi->output_max = output_max;
    pi->setpoint = 0.0f;
    pi->integral = 0.0f;
    pi->anti_windup_enabled = true;
    pi->last_update_us = 0;  // Initialize timing
}

float pi_update(pi_controller_t *pi, float current_value, float actual_dt) {
    // Calculate error
    float error = pi->setpoint - current_value;
    
    // Deadband around zero setpoint to help PWM reach true zero
    // Only activate deadband when setpoint is truly zero and we're very close
    if (pi->setpoint <= 5.0f && fabs(error) < 3.0f && current_value < 10.0f) {
        // Force output to zero when very close to zero setpoint
        pi->integral = 0.0f;  // Reset integral to prevent windup
        return 0.0f;
    }
    
    // Integral decay for large persistent errors (runaway protection)
    if (fabs(error) > 500.0f) {  // Large error suggests controller isn't working
        pi->integral *= 0.95f;  // Decay integral by 5% each cycle
    }
    
    // Proportional term
    float p_term = pi->kp * error;
    
    // Calculate preliminary output (P + existing I)
    float i_term = pi->ki * pi->integral;
    float prelim_output = p_term + i_term;
    
    // Conditional Integration - only integrate if not saturated
    bool saturated = (prelim_output >= pi->output_max) || (prelim_output <= pi->output_min);
    bool error_reducing = false;
    
    if (saturated) {
        // Check if error is reducing (moving away from saturation)
        if ((prelim_output >= pi->output_max && error < 0) ||
            (prelim_output <= pi->output_min && error > 0)) {
            error_reducing = true;
        }
    }
    
    // Only integrate if not saturated OR if error is reducing saturation
    if (!saturated || error_reducing) {
        pi->integral += error * actual_dt;
    }
    
    // Aggressive integral clamping to prevent windup
    float max_integral = (pi->output_max * 0.8f) / pi->ki;  // 80% of max output
    float min_integral = (pi->output_min * 0.8f) / pi->ki;  // 80% of min output
    
    // Also limit integral to reasonable range based on typical errors
    float error_based_limit = fabs(pi->output_max - pi->output_min) / (2.0f * pi->ki);
    if (max_integral > error_based_limit) max_integral = error_based_limit;
    if (min_integral < -error_based_limit) min_integral = -error_based_limit;
    
    if (pi->integral > max_integral) pi->integral = max_integral;
    if (pi->integral < min_integral) pi->integral = min_integral;
    
    // Recalculate with updated integral
    i_term = pi->ki * pi->integral;
    float output = p_term + i_term;
    
    // Final output clamping
    if (output > pi->output_max) {
        output = pi->output_max;
    } else if (output < pi->output_min) {
        output = pi->output_min;
    }
    
    return output;
}

void pi_set_gains(pi_controller_t *pi, float kp, float ki) {
    pi->kp = kp;
    pi->ki = ki;
}

void pi_set_setpoint(pi_controller_t *pi, float setpoint) {
    pi->setpoint = setpoint;
}

void pi_reset(pi_controller_t *pi) {
    pi->integral = 0.0f;
}

static const char *TAG = "motor_control";



// High-frequency ADC sampling task using continuous ADC with DMA
void adc_sampling_task(void *) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "ADC sampling task started at %d Hz (DMA continuous mode)", g_freq_config.adc_sampling_hz);
    
    while (1) {
        // Check if new ADC data is available from DMA
        if (g_adc_data_ready) {
            int adc_raw = g_latest_adc_reading;
            g_adc_data_ready = false;  // Clear flag
            
            // Apply filtering/threshold
            if (adc_raw < 10) adc_raw = 0;
            
            // Thread-safe data update
            if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(1))) {
                g_shared_data.adc_reading = (float)adc_raw;
                g_shared_data.new_adc_data = true;
                xSemaphoreGive(g_shared_data.data_mutex);
            }
        }
        
        // Calculate dynamic period with minimum tick protection
        uint32_t period_ms = 1000 / g_freq_config.adc_sampling_hz;
        if (period_ms == 0) period_ms = 1;  // Minimum 1ms period
        TickType_t delay_ticks = pdMS_TO_TICKS(period_ms);
        if (delay_ticks == 0) delay_ticks = 1;  // Ensure at least 1 tick
        vTaskDelayUntil(&last_wake_time, delay_ticks);
    }
}

// High-frequency motor control task  
void motor_control_task(void *) {
    TickType_t last_wake_time = xTaskGetTickCount();
    float local_adc = 0, local_setpoint = 0;
    
    ESP_LOGI(TAG, "Motor control task started at %d Hz", g_freq_config.motor_control_hz);
    
    while (1) {
        // Get latest data (non-blocking)
        bool use_adc_mode = false;
        if (xSemaphoreTake(g_shared_data.data_mutex, 0)) {
            if (g_shared_data.new_adc_data) {
                local_adc = g_shared_data.adc_reading;
                g_shared_data.new_adc_data = false;
            }
            local_setpoint = g_shared_data.setpoint_value;
            use_adc_mode = g_shared_data.adc_mode;
            xSemaphoreGive(g_shared_data.data_mutex);
        }
        
        int pwm_output;
        
        if (use_adc_mode) {
            // ADC Passthrough Mode: Direct scaling from ADC to PWM
            // Scale ADC reading (0-4095) directly to PWM (0-8191)
            pwm_output = (int)((local_adc / (float)ADC_MAX) * (float)MAX_PWM);
            
            // Clamp to valid PWM range
            if (pwm_output < 0) pwm_output = 0;
            if (pwm_output > MAX_PWM) pwm_output = MAX_PWM;
            
        } else {
            // Manual Mode: Use PI controller
            // Update PI controller setpoint
            pi_set_setpoint(&motor_pi, local_setpoint);
            
            // Calculate adaptive dt for PI controller
            uint64_t current_time_us = esp_timer_get_time();
            float actual_dt = motor_pi.dt;  // Default to nominal
            
            if (motor_pi.last_update_us != 0) {
                actual_dt = (current_time_us - motor_pi.last_update_us) / 1000000.0f;
                
                // Sanity check: clamp dt to reasonable range (0.1ms to 100ms)
                if (actual_dt < 0.0001f) actual_dt = 0.0001f;
                if (actual_dt > 0.1f) actual_dt = 0.1f;
            }
            motor_pi.last_update_us = current_time_us;
            
            // Calculate control output with adaptive dt
            float control_signal = pi_update(&motor_pi, local_adc, actual_dt);
            pwm_output = (int)control_signal;
        }
        
        // Update motor PWM
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pwm_output));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        
        // Store output for telemetry 
        if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(1))) {
            g_shared_data.pwm_output = pwm_output;  
            g_shared_data.new_control_output = true;
            xSemaphoreGive(g_shared_data.data_mutex);
        }
        
        // Calculate dynamic period with minimum tick protection
        uint32_t period_ms = 1000 / g_freq_config.motor_control_hz;
        if (period_ms == 0) period_ms = 1;  // Minimum 1ms period
        TickType_t delay_ticks = pdMS_TO_TICKS(period_ms);
        if (delay_ticks == 0) delay_ticks = 1;  // Ensure at least 1 tick
        vTaskDelayUntil(&last_wake_time, delay_ticks);
    }
}

// Medium-frequency telemetry task
void telemetry_task(void *) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Telemetry task started at %d Hz", g_freq_config.telemetry_hz);
    
    while (1) {
        // Initialize variables to safe defaults
        float adc_val = 0.0f, setpoint_val = 0.0f;
        int pwm_val = 0, solenoid_state = 0;
        
        // Collect all data atomically
        if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(5))) {
            adc_val = g_shared_data.adc_reading;
            setpoint_val = g_shared_data.setpoint_value;
            pwm_val = g_shared_data.pwm_output;
            xSemaphoreGive(g_shared_data.data_mutex);
        }
        
        // Solenoid control
        solenoid_state = (pwm_val > SECURITY_MARGIN) ? 1 : 0;
        gpio_set_level(SOLENOID_PIN, solenoid_state);
        
        // Update shared solenoid state
        if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(1))) {
            g_shared_data.solenoid_state = solenoid_state;
            xSemaphoreGive(g_shared_data.data_mutex);
        }
        
        // Generate telemetry
        int64_t t_ms = esp_timer_get_time() / 1000;
        char telemetry[128];
        snprintf(telemetry, sizeof(telemetry), "TEL,%lld,%.0f,%.0f,%d,%d\n", 
                 t_ms, setpoint_val, adc_val, pwm_val, solenoid_state);
        
        // Send via UART
        printf("%s", telemetry);
        
        // Calculate dynamic period with minimum tick protection
        uint32_t period_ms = 1000 / g_freq_config.telemetry_hz;
        if (period_ms == 0) period_ms = 1;  // Minimum 1ms period
        TickType_t delay_ticks = pdMS_TO_TICKS(period_ms);
        if (delay_ticks == 0) delay_ticks = 1;  // Ensure at least 1 tick
        vTaskDelayUntil(&last_wake_time, delay_ticks);
    }
}

// Low-frequency BLE update task
void ble_update_task(void *) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "BLE update task started at %d Hz", g_freq_config.ble_update_hz);
    
    while (1) {
        // Initialize variables to safe defaults
        float adc_val = 0.0f, setpoint_val = 0.0f;
        int pwm_val = 0, solenoid_state = 0;
        
        // Get telemetry data
        if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(5))) {
            adc_val = g_shared_data.adc_reading;
            setpoint_val = g_shared_data.setpoint_value;
            pwm_val = g_shared_data.pwm_output;
            solenoid_state = g_shared_data.solenoid_state;
            xSemaphoreGive(g_shared_data.data_mutex);
        }
        
        // Generate BLE telemetry
        int64_t t_ms = esp_timer_get_time() / 1000;
        char ble_telemetry[128];
        snprintf(ble_telemetry, sizeof(ble_telemetry), "TEL,%lld,%.0f,%.0f,%d,%d\n", 
                 t_ms, setpoint_val, adc_val, pwm_val, solenoid_state);
        
        // Send BLE telemetry
        send_ble_telemetry(ble_telemetry);
        
        // Calculate dynamic period with minimum tick protection
        uint32_t period_ms = 1000 / g_freq_config.ble_update_hz;
        if (period_ms == 0) period_ms = 1;  // Minimum 1ms period
        TickType_t delay_ticks = pdMS_TO_TICKS(period_ms);
        if (delay_ticks == 0) delay_ticks = 1;  // Ensure at least 1 tick
        vTaskDelayUntil(&last_wake_time, delay_ticks);
    }
}

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
void uart_read_task(void *)
{
    uint8_t data[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';  // Null terminate
            char *line = (char *)data;
            
            // Parse command: "SET,<adc_value>" or "SETP,<percent>" or "MODE,<ADC|MAN>" or "PI,<kp>,<ki>" or "FREQ,<subsystem>,<hz>"
            if (strncmp(line, "SET,", 4) == 0) {
                int new_setpoint = atoi(line + 4);
                // Clamp to valid range
                if (new_setpoint < 0) new_setpoint = 0;
                if (new_setpoint > ADC_MAX) new_setpoint = ADC_MAX;
                
                // Reset PI controller integral on significant setpoint changes
                if (abs(new_setpoint - g_setpoint) > 100 || new_setpoint <= 10) {
                    pi_reset(&motor_pi);
                    ESP_LOGI(TAG, "PI reset due to setpoint change: %d -> %d", g_setpoint, new_setpoint);
                }
                g_setpoint = new_setpoint;
                // Update shared data
                if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(10))) {
                    g_shared_data.setpoint_value = (float)new_setpoint;
                    xSemaphoreGive(g_shared_data.data_mutex);
                }
                ESP_LOGI(TAG, "Setpoint updated to %d", g_setpoint);
            } else if (strncmp(line, "SETP,", 5) == 0) {
                int pct = atoi(line + 5);
                if (pct < 0) pct = 0;
                if (pct > 100) pct = 100;
                int new_setpoint = (pct * ADC_MAX) / 100;
                // Reset PI controller integral on significant setpoint changes
                if (abs(new_setpoint - g_setpoint) > 100 || new_setpoint <= 10) {
                    pi_reset(&motor_pi);
                    ESP_LOGI(TAG, "PI reset due to setpoint change: %d -> %d", g_setpoint, new_setpoint);
                }
                g_setpoint = new_setpoint;
                // Update shared data
                if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(10))) {
                    g_shared_data.setpoint_value = (float)new_setpoint;
                    xSemaphoreGive(g_shared_data.data_mutex);
                }
                ESP_LOGI(TAG, "Setpoint (%%) updated to %d%% -> ADC %d", pct, g_setpoint);
            } else if (strncmp(line, "MODE,", 5) == 0) {
                char *mode = line + 5;
                if (strncmp(mode, "ADC", 3) == 0 || strncmp(mode, "1", 1) == 0) {
                    g_use_adc = true;
                    pi_reset(&motor_pi);  // Reset PI when switching modes
                    // Update shared data
                    if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(10))) {
                        g_shared_data.adc_mode = true;
                        xSemaphoreGive(g_shared_data.data_mutex);
                    }
                    ESP_LOGI(TAG, "Mode set to ADC passthrough");
                } else if (strncmp(mode, "MAN", 3) == 0 || strncmp(mode, "0", 1) == 0) {
                    g_use_adc = false;
                    pi_reset(&motor_pi);  // Reset PI when switching modes
                    // Update shared data
                    if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(10))) {
                        g_shared_data.adc_mode = false;
                        xSemaphoreGive(g_shared_data.data_mutex);
                    }
                    ESP_LOGI(TAG, "Mode set to Manual setpoint");
                }
            } else if (strncmp(line, "PI,", 3) == 0) {
                // Parse PI parameters: "PI,kp,ki"
                char *token = strtok(line + 3, ",");
                if (token != NULL) {
                    float kp = atof(token);
                    token = strtok(NULL, ",");
                    if (token != NULL) {
                        float ki = atof(token);
                        pi_set_gains(&motor_pi, kp, ki);
                        pi_reset(&motor_pi);  // Reset integral term
                        ESP_LOGI(TAG, "PI gains updated: Kp=%.3f, Ki=%.3f", kp, ki);
                    }
                }
            } else if (strncmp(line, "PIGET", 5) == 0) {
                // Get current PI parameters
                printf("PI: Kp=%.3f, Ki=%.3f\n", motor_pi.kp, motor_pi.ki);
            } else if (strncmp(line, "FREQ,", 5) == 0) {
                // Format: "FREQ,MOTOR,1000" or "FREQ,ADC,500" 
                char *subsystem = strtok(line + 5, ",");
                char *freq_str = strtok(NULL, ",");
                
                if (subsystem && freq_str) {
                    uint16_t new_freq = (uint16_t)atoi(freq_str);
                    
                    // Validate frequency range (10 Hz to 10 kHz)
                    if (new_freq < 10) new_freq = 10;
                    if (new_freq > 10000) new_freq = 10000;
                    
                    if (strncmp(subsystem, "MOTOR", 5) == 0) {
                        g_freq_config.motor_control_hz = new_freq;
                        ESP_LOGI(TAG, "Motor control frequency set to %d Hz", new_freq);
                        
                    } else if (strncmp(subsystem, "ADC", 3) == 0) {
                        g_freq_config.adc_sampling_hz = new_freq;
                        ESP_LOGI(TAG, "ADC sampling frequency set to %d Hz", new_freq);
                        
                    } else if (strncmp(subsystem, "TEL", 3) == 0) {
                        g_freq_config.telemetry_hz = new_freq;
                        ESP_LOGI(TAG, "Telemetry frequency set to %d Hz", new_freq);
                        
                    } else if (strncmp(subsystem, "BLE", 3) == 0) {
                        g_freq_config.ble_update_hz = new_freq;
                        ESP_LOGI(TAG, "BLE update frequency set to %d Hz", new_freq);
                    }
                }
            } else if (strncmp(line, "FREQGET", 7) == 0) {
                printf("FREQ: MOTOR=%dHz, ADC=%dHz, TEL=%dHz, BLE=%dHz\n",
                       g_freq_config.motor_control_hz,
                       g_freq_config.adc_sampling_hz, 
                       g_freq_config.telemetry_hz,
                       g_freq_config.ble_update_hz);
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
    xTaskCreate(uart_read_task, "uart_read_task", 2560, NULL, 10, NULL);
    // Configure solenoid control pin
    gpio_reset_pin(SOLENOID_PIN);
    gpio_set_direction(SOLENOID_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SOLENOID_PIN, 0);
    ESP_LOGI(TAG, "Solenoid configured on GPIO %d", SOLENOID_PIN);

    // Configure continuous ADC with DMA for better performance
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_READ_LEN * 4,
        .conv_frame_size = ADC_CONV_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));
    
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    
    adc_digi_pattern_config_t adc_pattern[1] = {
        {
            .atten = ADC_ATTEN,
            .channel = ADC_CHANNEL,
            .unit = ADC_UNIT,
            .bit_width = ADC_BITWIDTH_12,
        }
    };
    dig_cfg.pattern_num = 1;
    dig_cfg.adc_pattern = adc_pattern;
    
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    
    // Register callback and start continuous conversion
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    
    ESP_LOGI(TAG, "Continuous ADC configured on GPIO %d at %d Hz", ADC_INPUT_PIN, ADC_SAMPLE_FREQ_HZ);

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

    // Initialize shared data mutex
    g_shared_data.data_mutex = xSemaphoreCreateMutex();
    if (g_shared_data.data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return;
    }
    
    // Initialize shared data
    g_shared_data.adc_reading = 0.0f;
    g_shared_data.setpoint_value = 0.0f;
    g_shared_data.pwm_output = 0;
    g_shared_data.solenoid_state = 0;
    g_shared_data.new_adc_data = false;
    g_shared_data.new_control_output = false;
    g_shared_data.adc_mode = true;  // Default to ADC passthrough mode
    
    // Initialize PI controller with reduced Ki to prevent windup
    // Sample time = 0.001s (1000Hz), Output range 0-8191 (13-bit PWM)
    pi_init(&motor_pi, 1.2f, 0.02f, 0.001f, 0.0f, (float)MAX_PWM);
    ESP_LOGI(TAG, "PI controller initialized: Kp=%.2f, Ki=%.2f ", 
             motor_pi.kp, motor_pi.ki);

    // Create multi-rate control tasks
    ESP_LOGI(TAG, "Starting multi-rate control system:");
    ESP_LOGI(TAG, "  - ADC sampling: %d Hz", g_freq_config.adc_sampling_hz);
    ESP_LOGI(TAG, "  - Motor control: %d Hz", g_freq_config.motor_control_hz);
    ESP_LOGI(TAG, "  - Telemetry: %d Hz", g_freq_config.telemetry_hz);
    ESP_LOGI(TAG, "  - BLE updates: %d Hz", g_freq_config.ble_update_hz);
    
    // Create ADC sampling task (highest priority) - continuous ADC runs in background  
    xTaskCreate(adc_sampling_task, "adc_task", 2560, NULL, configMAX_PRIORITIES - 1, &adc_task_handle);
    
    // Create motor control task (high priority) - optimized stack for PI calculations
    xTaskCreate(motor_control_task, "motor_task", 1792, NULL, configMAX_PRIORITIES - 2, &motor_task_handle);
    
    // Create telemetry task (medium priority) - increased stack for string operations
    xTaskCreate(telemetry_task, "telemetry_task", 3072, NULL, configMAX_PRIORITIES - 3, &telemetry_task_handle);
    
    // Create BLE update task (low priority) - increased stack for BLE and string operations
    xTaskCreate(ble_update_task, "ble_task", 3072, NULL, configMAX_PRIORITIES - 4, &ble_task_handle);
    
    ESP_LOGI(TAG, "Multi-rate control system started successfully");
    
    // Main task becomes idle - just monitor system
    while (1) {
        // In manual mode, update setpoint from g_setpoint
        // In ADC mode, motor task handles direct passthrough
        bool adc_mode_active = false;
        if (xSemaphoreTake(g_shared_data.data_mutex, pdMS_TO_TICKS(10))) {
            adc_mode_active = g_shared_data.adc_mode;
            if (!adc_mode_active) {
                g_shared_data.setpoint_value = (float)g_setpoint;
            }
            xSemaphoreGive(g_shared_data.data_mutex);
        }
        
        // Sleep for 100ms (this task is now low priority)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}