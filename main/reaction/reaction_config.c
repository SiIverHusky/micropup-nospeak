/**
 * @file reaction_config.c
 * @brief Reaction system implementation
 * 
 * Monitors IMU data and triggers animations when thresholds are met.
 */

#include "reaction_config.h"
#include "walk_forward_reaction.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "REACTION";

// ═══════════════════════════════════════════════════════
// STATE TRACKING
// ═══════════════════════════════════════════════════════

static TickType_t last_reaction_time = 0;
static bool initialized = false;

// ═══════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════

/**
 * @brief Check if enough time has passed since last reaction
 */
static bool is_cooldown_expired(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ms = pdTICKS_TO_MS(now - last_reaction_time);
    return elapsed_ms >= REACTION_COOLDOWN_MS;
}

/**
 * @brief Update the last reaction timestamp
 */
static void update_reaction_time(void)
{
    last_reaction_time = xTaskGetTickCount();
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void reaction_init(void)
{
    ESP_LOGI(TAG, "Reaction system initialized");
    ESP_LOGI(TAG, "Front push threshold: +%.1f m/s²", REACTION_PUSH_FRONT_THRESHOLD);
    ESP_LOGI(TAG, "Back push threshold: %.1f m/s²", REACTION_PUSH_BACK_THRESHOLD);
    ESP_LOGI(TAG, "Cooldown: %d ms", REACTION_COOLDOWN_MS);
    
    initialized = true;
}

void reaction_process_imu(const qmi8658a_data_t *data)
{
    if (!initialized) {
        return;
    }
    
    // Check cooldown
    if (!is_cooldown_expired()) {
        return;
    }
    
    // Check for front push (+X direction)
    if (data->accel_x >= REACTION_PUSH_FRONT_THRESHOLD) {
        ESP_LOGI(TAG, "Front push detected! (%.2f m/s²)", data->accel_x);
        update_reaction_time();
        
        // Play walk forward animation for 3 cycles
        walk_forward_play(3);
        
        return;
    }
    
    // Check for back push (-X direction)
    if (data->accel_x <= REACTION_PUSH_BACK_THRESHOLD) {
        ESP_LOGI(TAG, "Back push detected! (%.2f m/s²)", data->accel_x);
        update_reaction_time();
        
        // TODO: Add backward reaction animation here
        ESP_LOGW(TAG, "Back push reaction not yet implemented");
        
        return;
    }
}
