/**
 * @file main.c
 * @brief Crawl Gait Demo
 * 
 * Demonstrates the crawl gait walking algorithm for a quadruped robot.
 * The crawl gait uses a wave-like ripple pattern from back to front.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// STS3032 Driver
#include "sts3032_driver.h"

// Crawl gait algorithm
#include "gait_common.h"
#include "crawl_gait.h"

static const char *TAG = "CRAWL_DEMO";

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
// DEMO APPLICATION
// ═══════════════════════════════════════════════════════

void app_main(void)
{
    ESP_LOGI(TAG, "Crawl Gait Demo - Wave Pattern Walking");
    ESP_LOGI(TAG, "=======================================");
    
    // ───────────────────────────────────────────────────────
    // STEP 1: Initialize the servo driver
    // ───────────────────────────────────────────────────────
    
    sts_protocol_config_t protocol_config = {
        .uart_num = SERVO_UART_NUM,
        .tx_pin = SERVO_TX_PIN,
        .rx_pin = SERVO_RX_PIN,
        .txen_pin = SERVO_TXEN_PIN,
        .baud_rate = SERVO_BAUD_RATE,
    };
    
    esp_err_t ret = sts_protocol_init(&protocol_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STS3032 driver");
        return;
    }
    
    ESP_LOGI(TAG, "Servo driver initialized");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // ───────────────────────────────────────────────────────
    // STEP 2: Initialize crawl gait
    // ───────────────────────────────────────────────────────
    
    crawl_gait_config_t crawl_config = {
        .stance_angle_fr = 270.0f,
        .stance_angle_fl = 90.0f,
        .stance_angle_br = 90.0f,
        .stance_angle_bl = 270.0f,
        .swing_amplitude = 25.0f,
        .step_duration_ms = 250,
        .servo_speed = SPEED_VERY_FAST,
    };
    
    if (!crawl_gait_init(&crawl_config)) {
        ESP_LOGW(TAG, "Some servos not responding, but continuing...");
    }
    
    ESP_LOGI(TAG, "Crawl gait initialized");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // ───────────────────────────────────────────────────────
    // STEP 3: Demo FORWARD crawling (6 seconds)
    // ───────────────────────────────────────────────────────
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> FORWARD - alternating sides for straight motion");
    crawl_gait_start(GAIT_DIRECTION_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    // ───────────────────────────────────────────────────────
    // STEP 4: Demo TURN RIGHT (6 seconds)
    // ───────────────────────────────────────────────────────
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> TURN RIGHT - same-side consecutive pattern");
    crawl_gait_set_direction(GAIT_DIRECTION_TURN_RIGHT);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    // ───────────────────────────────────────────────────────
    // STEP 5: Demo TURN LEFT (6 seconds)
    // ───────────────────────────────────────────────────────
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> TURN LEFT - mirrored same-side pattern");
    crawl_gait_set_direction(GAIT_DIRECTION_TURN_LEFT);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    // ───────────────────────────────────────────────────────
    // STEP 6: Back to FORWARD (6 seconds)
    // ───────────────────────────────────────────────────────
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> FORWARD again");
    crawl_gait_set_direction(GAIT_DIRECTION_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    // Stop and return to stance
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Stopping crawl gait");
    crawl_gait_stop();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Demo complete!");
    ESP_LOGI(TAG, "  - FORWARD:    BL -> FR -> BR -> FL (alternating sides)");
    ESP_LOGI(TAG, "  - TURN RIGHT: BL -> BR -> FL -> FR (same-side consecutive)");
    ESP_LOGI(TAG, "  - TURN LEFT:  BR -> BL -> FR -> FL (mirror of turn right)");
    
    // ───────────────────────────────────────────────────────
    // Idle
    // ───────────────────────────────────────────────────────
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Idle - reset to run again");
    }
}
