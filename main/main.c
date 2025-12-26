/**
 * @file main.c
 * @brief Robot Control with Minimal BLE Servo Protocol
 * 
 * Supports two modes:
 *   1. BLE Control Mode - Control servos via Web Bluetooth for gait development
 *   2. Demo Mode - Run automated gait demonstrations
 * 
 * BLE Commands (JSON format):
 *   - Single move:   {"s":[fr,fl,br,bl,speed,delay_ms]}
 *   - Multi move:    {"m":[[fr,fl,br,bl,speed,delay],[...]]}
 *   - Stance:        {"c":"stance"}
 *   - Ping:          {"c":"ping"}
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

// Dog configuration (pins, servo IDs, angle reversal)
#include "dog_config.h"

// Minimal BLE servo control
#include "ble/ble_servo.h"

// Crawl gait algorithm
#include "gait_common.h"
#include "crawl_gait.h"

static const char *TAG = "ROBOT_MAIN";

// ═══════════════════════════════════════════════════════
// BLE SERVO CALLBACKS
// ═══════════════════════════════════════════════════════

/**
 * @brief Called when BLE receives a servo move command
 */
static void on_servo_move(float fr, float fl, float br, float bl,
                          uint16_t speed, uint16_t delay_ms) {
    ESP_LOGI(TAG, "Move: FR=%.0f FL=%.0f BR=%.0f BL=%.0f spd=%u dly=%u",
             fr, fl, br, bl, speed, delay_ms);
    
    // Move servos (dog_config handles right-side reversal automatically)
    dog_servo_move_all(fr, fl, br, bl, speed);
    
    // Apply delay if specified
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/**
 * @brief Called when BLE receives a stance command
 */
static void on_stance(void) {
    ESP_LOGI(TAG, "Stance command received");
    // Stop any running gait
    if (crawl_gait_is_running()) {
        crawl_gait_stop();
    }
    dog_goto_stance();
}

/**
 * @brief Called on BLE connection state change
 */
static void on_connect(bool connected) {
    if (connected) {
        ESP_LOGI(TAG, "=== BLE Client Connected ===");
        // Stop gait and go to stance on new connection
        if (crawl_gait_is_running()) {
            crawl_gait_stop();
        }
        dog_goto_stance();
    } else {
        ESP_LOGI(TAG, "=== BLE Client Disconnected ===");
    }
}

// ═══════════════════════════════════════════════════════
// DEMO MODE (Optional)
// ═══════════════════════════════════════════════════════

#define RUN_DEMO_MODE 0  // Set to 1 to run demo instead of BLE mode

static void run_demo_mode(void) {
    ESP_LOGI(TAG, "Running Demo Mode");
    
    // Initialize crawl gait
    crawl_gait_config_t crawl_config = {
        .stance_angle_fr = DOG_STANCE_FRONT,
        .stance_angle_fl = DOG_STANCE_FRONT,
        .stance_angle_br = DOG_STANCE_BACK,
        .stance_angle_bl = DOG_STANCE_BACK,
        .swing_amplitude = DOG_SWING_AMPLITUDE,
        .step_duration_ms = 250,
        .servo_speed = DOG_SPEED_VERY_FAST,
    };
    crawl_gait_init(&crawl_config);
    
    // Demo: Forward 6s -> Right 6s -> Left 6s -> Forward 6s -> Stop
    ESP_LOGI(TAG, ">>> FORWARD");
    crawl_gait_start(GAIT_DIRECTION_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    ESP_LOGI(TAG, ">>> TURN RIGHT");
    crawl_gait_set_direction(GAIT_DIRECTION_TURN_RIGHT);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    ESP_LOGI(TAG, ">>> TURN LEFT");
    crawl_gait_set_direction(GAIT_DIRECTION_TURN_LEFT);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    ESP_LOGI(TAG, ">>> FORWARD");
    crawl_gait_set_direction(GAIT_DIRECTION_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(6000));
    
    crawl_gait_stop();
    ESP_LOGI(TAG, "Demo complete!");
}

// ═══════════════════════════════════════════════════════
// MAIN APPLICATION
// ═══════════════════════════════════════════════════════

void app_main(void)
{
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      MicroPupper Robot Control        ║");
    ESP_LOGI(TAG, "║      BLE + Web Bluetooth Ready        ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
    // ───────────────────────────────────────────────────────
    // STEP 1: Initialize NVS (required for BLE)
    // ───────────────────────────────────────────────────────
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
    // ───────────────────────────────────────────────────────
    // STEP 2: Initialize dog hardware
    // ───────────────────────────────────────────────────────
    
    if (!dog_init(NULL)) {
        ESP_LOGW(TAG, "Some servos not responding, but continuing...");
    }
    ESP_LOGI(TAG, "Dog hardware initialized");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // ───────────────────────────────────────────────────────
    // STEP 3: Initialize crawl gait (for text commands)
    // ───────────────────────────────────────────────────────
    
    crawl_gait_config_t crawl_config = {
        .stance_angle_fr = DOG_STANCE_FRONT,
        .stance_angle_fl = DOG_STANCE_FRONT,
        .stance_angle_br = DOG_STANCE_BACK,
        .stance_angle_bl = DOG_STANCE_BACK,
        .swing_amplitude = DOG_SWING_AMPLITUDE,
        .step_duration_ms = 250,
        .servo_speed = DOG_SPEED_VERY_FAST,
    };
    crawl_gait_init(&crawl_config);
    ESP_LOGI(TAG, "Crawl gait initialized");
    
#if RUN_DEMO_MODE
    // ───────────────────────────────────────────────────────
    // Demo Mode
    // ───────────────────────────────────────────────────────
    run_demo_mode();
#else
    // ───────────────────────────────────────────────────────
    // STEP 4: Initialize BLE Servo Control
    // ───────────────────────────────────────────────────────
    
    // Initialize BLE with callbacks
    if (!ble_servo_init(on_servo_move, on_stance, on_connect)) {
        ESP_LOGE(TAG, "Failed to initialize BLE!");
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  BLE Ready - Connect via Web Bluetooth ║");
    ESP_LOGI(TAG, "║  Device: MicroPupper                   ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Commands:");
    ESP_LOGI(TAG, "  Move:   {\"s\":[fr,fl,br,bl,speed,delay]}");
    ESP_LOGI(TAG, "  Multi:  {\"m\":[[fr,fl,br,bl,spd,dly],[...]]}");
    ESP_LOGI(TAG, "  Stance: {\"c\":\"stance\"}");
    ESP_LOGI(TAG, "  Ping:   {\"c\":\"ping\"}");
    ESP_LOGI(TAG, "");
#endif
    
    // ───────────────────────────────────────────────────────
    // Main loop
    // ───────────────────────────────────────────────────────
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        if (ble_servo_is_connected()) {
            ESP_LOGD(TAG, "BLE connected, waiting for commands...");
        } else {
            ESP_LOGD(TAG, "Waiting for BLE connection...");
        }
    }
}
