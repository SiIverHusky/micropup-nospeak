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
    // STEP 2: Connect to servos 1..4 and move them to 90°
    // ───────────────────────────────────────────────────────

    ESP_LOGI(TAG, "Scanning servos 1..4 and moving to 90°");

    for (uint8_t id = 1; id <= 4; id++) {
        ESP_LOGI(TAG, "Pinging servo ID %d...", id);
        if (!sts_servo_ping(id)) {
            ESP_LOGW(TAG, "Servo ID %d not found; skipping", id);
            continue;
        }

        ESP_LOGI(TAG, "Servo ID %d connected", id);
        sts_servo_enable_torque(id, true);
        vTaskDelay(pdMS_TO_TICKS(50));

        float target = (id == 1 || id == 4) ? 270.0f : 90.0f;
        ESP_LOGI(TAG, "Setting servo %d to %.1f°", id, target);
        sts_servo_set_angle(id, target, SPEED_MAX);
        vTaskDelay(pdMS_TO_TICKS(200));

        float angle = 0.0f;
        if (sts_servo_get_angle(id, &angle)) {
            ESP_LOGI(TAG, "Servo %d reported angle: %.1f°", id, angle);
        } else {
            ESP_LOGW(TAG, "Failed to read angle from servo %d", id);
        }
    }

    // After initial positioning, periodically report positions
    while (1) {
        for (uint8_t id = 1; id <= 4; id++) {
            float angle = 0.0f;
            if (sts_servo_get_angle(id, &angle)) {
                ESP_LOGI(TAG, "Servo %d: %.1f°", id, angle);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
