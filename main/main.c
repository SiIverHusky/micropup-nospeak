/**
 * @file main.c
 * @brief STS3032 Servo Driver - Basic Example
 * 
 * This example demonstrates the minimal code required to use the STS3032 servo driver.
 * It shows initialization and basic servo control operations.
 * 
 * For additional examples, see the examples/ directory.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// STS3032 Driver - include the single driver header provided by the component
#include "sts3032_driver.h"

static const char *TAG = "EXAMPLE";

// ═══════════════════════════════════════════════════════
// HARDWARE CONFIGURATION
// ═══════════════════════════════════════════════════════
// Adjust these pins to match your hardware setup

#define SERVO_UART_NUM      UART_NUM_1
#define SERVO_TX_PIN        GPIO_NUM_10
#define SERVO_RX_PIN        GPIO_NUM_11
#define SERVO_TXEN_PIN      GPIO_NUM_3
#define SERVO_BAUD_RATE     1000000

// ═══════════════════════════════════════════════════════
// EXAMPLE APPLICATION
// ═══════════════════════════════════════════════════════

void app_main(void)
{
    ESP_LOGI(TAG, "STS3032 Servo Driver - Basic Example");
    ESP_LOGI(TAG, "=====================================");
    
    // ───────────────────────────────────────────────────────
    // STEP 1: Initialize the driver
    // ───────────────────────────────────────────────────────
    // This is the only required initialization
    
    sts_protocol_config_t config = {
        .uart_num = SERVO_UART_NUM,
        .tx_pin = SERVO_TX_PIN,
        .rx_pin = SERVO_RX_PIN,
        .txen_pin = SERVO_TXEN_PIN,
        .baud_rate = SERVO_BAUD_RATE,
    };
    
    esp_err_t ret = sts_protocol_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STS3032 driver");
        return;
    }
    
    ESP_LOGI(TAG, "Driver initialized successfully");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // ───────────────────────────────────────────────────────
    // STEP 2: Connect to a servo
    // ───────────────────────────────────────────────────────
    
    uint8_t servo_id = 3;  // Change this to match your servo's ID
    
    ESP_LOGI(TAG, "Connecting to servo ID %d...", servo_id);
    if (!sts_servo_ping(servo_id)) {
        ESP_LOGE(TAG, "Servo ID %d not found. Check connections and servo ID.", servo_id);
        return;
    }
    
    ESP_LOGI(TAG, "Servo ID %d connected", servo_id);
    
    // ───────────────────────────────────────────────────────
    // STEP 3: Enable torque
    // ───────────────────────────────────────────────────────
    
    sts_servo_enable_torque(servo_id, true);
    ESP_LOGI(TAG, "Torque enabled");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // ───────────────────────────────────────────────────────
    // STEP 4: Control the servo
    // ───────────────────────────────────────────────────────
    
    ESP_LOGI(TAG, "Starting basic movement sequence...");
    
    while (1) {
        // Move to center position (90 degrees)
        ESP_LOGI(TAG, "Moving to 90°");
        sts_servo_set_angle(servo_id, 0.0, SPEED_MAX);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Move to minimum position (45 degrees)
        ESP_LOGI(TAG, "Moving to 45°");
        sts_servo_set_angle(servo_id, 360.0, SPEED_MAX + 100);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Move to maximum position (135 degrees)
        ESP_LOGI(TAG, "Moving to 135°");
        sts_servo_set_angle(servo_id, 180.0, SPEED_MAX + 200);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Read current position
        float current_angle;
        if (sts_servo_get_angle(servo_id, &current_angle)) {
            ESP_LOGI(TAG, "Current position: %.1f°", current_angle);
        }
    }

}
