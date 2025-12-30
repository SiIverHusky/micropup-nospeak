/**
 * @file reaction_config.c
 * @brief Reaction system implementation
 * 
 * Monitors IMU data and triggers animations when thresholds are met.
 * Includes gyro-based stabilization for keeping legs facing ground.
 */

#include "reaction_config.h"
#include "walk_forward_reaction.h"
#include "dog_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "REACTION";

// ═══════════════════════════════════════════════════════
// STATE TRACKING
// ═══════════════════════════════════════════════════════

static TickType_t last_reaction_time = 0;
static bool initialized = false;

// Previous acceleration reading for delta calculation
static float prev_accel_x = 0.0f;
static bool has_prev_reading = false;

// ═══════════════════════════════════════════════════════
// GYRO STABILIZATION STATE
// ═══════════════════════════════════════════════════════

static bool gyro_stabilize_enabled = REACTION_GYRO_STABILIZE_ENABLED_DEFAULT;
static float gyro_filtered_y = 0.0f;         // Low-pass filtered gyro Y
static float accumulated_angle = 0.0f;       // Integrated angle offset
static float prev_accumulated_angle = 0.0f;  // Previous angle for delta calculation
static TickType_t last_stabilize_time = 0;

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
// GYRO STABILIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Apply gyro-based stabilization to keep legs facing ground
 * 
 * Uses gyro Y axis (pitch rate) to compute angle correction.
 * Adjusts all leg servos to compensate for body tilt.
 */
static void apply_gyro_stabilization(const qmi8658a_data_t *data)
{
    if (!gyro_stabilize_enabled) {
        return;
    }
    
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ms = pdTICKS_TO_MS(now - last_stabilize_time);
    
    // Rate limit updates
    if (elapsed_ms < REACTION_GYRO_UPDATE_INTERVAL_MS) {
        return;
    }
    last_stabilize_time = now;
    
    // Get gyro Y reading (pitch rate in degrees/second)
    float gyro_y = data->gyro_y;
    
    // Apply deadzone
    if (fabsf(gyro_y) < REACTION_GYRO_DEADZONE) {
        gyro_y = 0.0f;
    }
    
    // Low-pass filter for smoothing
    gyro_filtered_y = (REACTION_GYRO_SMOOTHING * gyro_y) + 
                      ((1.0f - REACTION_GYRO_SMOOTHING) * gyro_filtered_y);
    
    // Calculate correction angle
    float correction = gyro_filtered_y * REACTION_GYRO_GAIN;
    
    // Integrate to accumulate angle (with decay to prevent drift)
    accumulated_angle = accumulated_angle * 0.98f + correction * 0.02f;
    
    // Clamp correction to maximum allowed
    if (accumulated_angle > REACTION_GYRO_MAX_CORRECTION) {
        accumulated_angle = REACTION_GYRO_MAX_CORRECTION;
    } else if (accumulated_angle < -REACTION_GYRO_MAX_CORRECTION) {
        accumulated_angle = -REACTION_GYRO_MAX_CORRECTION;
    }
    
    // Apply correction to all legs
    // Front legs: stance is 90°, correction adds/subtracts
    // Back legs: stance is 270°, correction adds/subtracts (inverted)
    float front_correction = accumulated_angle;
    float back_correction = accumulated_angle;  
    
    float angle_fl = DOG_STANCE_FRONT + front_correction;
    float angle_fr = DOG_STANCE_FRONT + front_correction;
    float angle_bl = DOG_STANCE_BACK + back_correction;
    float angle_br = DOG_STANCE_BACK + back_correction;
    
    // Calculate dynamic speed based on how much the angle is CHANGING
    // Small movements = slow speed, large movements = fast speed
    float angle_delta = fabsf(accumulated_angle - prev_accumulated_angle);
    prev_accumulated_angle = accumulated_angle;
    
    float speed_ratio = angle_delta / REACTION_GYRO_SPEED_THRESHOLD;
    if (speed_ratio > 1.0f) {
        speed_ratio = 1.0f;
    }
    
    // Apply power curve to bias toward lower speeds
    // ratio^2.5 means: 50% input -> 17% output, 75% input -> 49% output
    speed_ratio = powf(speed_ratio, REACTION_GYRO_SPEED_CURVE);
    
    uint16_t dynamic_speed = (uint16_t)(REACTION_GYRO_SPEED_MIN + 
                              speed_ratio * (REACTION_GYRO_SPEED_MAX - REACTION_GYRO_SPEED_MIN));
    
    // Move servos with dynamic speed
    dog_servo_move_all(angle_fr, angle_fl, angle_br, angle_bl, dynamic_speed);
}

void reaction_gyro_stabilize_enable(bool enable)
{
    if (enable && !gyro_stabilize_enabled) {
        // Resetting state when enabling
        gyro_filtered_y = 0.0f;
        accumulated_angle = 0.0f;
        prev_accumulated_angle = 0.0f;
        last_stabilize_time = xTaskGetTickCount();
        ESP_LOGI(TAG, "Gyro stabilization ENABLED");
    } else if (!enable && gyro_stabilize_enabled) {
        // Return to stance when disabling
        dog_goto_stance();
        ESP_LOGI(TAG, "Gyro stabilization DISABLED - returning to stance");
    }
    gyro_stabilize_enabled = enable;
}

bool reaction_gyro_stabilize_is_enabled(void)
{
    return gyro_stabilize_enabled;
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

void reaction_init(void)
{
    ESP_LOGI(TAG, "Reaction system initialized (delta-based detection)");
    ESP_LOGI(TAG, "Delta threshold: %.1f m/s², Min accel: %.1f m/s²",
             REACTION_DELTA_THRESHOLD, REACTION_MIN_ACCEL);
    ESP_LOGI(TAG, "Cooldown: %d ms", REACTION_COOLDOWN_MS);
    ESP_LOGI(TAG, "Gyro stabilization: %s (max correction: %.1f°, gain: %.2f)",
             REACTION_GYRO_STABILIZE_ENABLED_DEFAULT ? "ENABLED" : "DISABLED",
             REACTION_GYRO_MAX_CORRECTION, REACTION_GYRO_GAIN);
    
    prev_accel_x = 0.0f;
    has_prev_reading = false;
    gyro_filtered_y = 0.0f;
    accumulated_angle = 0.0f;
    last_stabilize_time = xTaskGetTickCount();
    
    initialized = true;
}

void reaction_process_imu(const qmi8658a_data_t *data)
{
    if (!initialized) {
        return;
    }
    
    // Apply gyro stabilization if enabled (runs at its own rate)
    apply_gyro_stabilization(data);
    
    float current_accel_x = data->accel_x;
    
    // Need a previous reading to calculate delta
    if (!has_prev_reading) {
        prev_accel_x = current_accel_x;
        has_prev_reading = true;
        return;
    }
    
    // Calculate delta (change from previous reading)
    float delta = current_accel_x - prev_accel_x;
    
    // Store current as previous for next iteration
    prev_accel_x = current_accel_x;
    
    // Check cooldown
    if (!is_cooldown_expired()) {
        return;
    }
    
    // Front push: large positive delta AND current reading is positive
    // (acceleration suddenly increased in +X direction)
    if (delta >= REACTION_DELTA_THRESHOLD && current_accel_x >= REACTION_MIN_ACCEL) {
        ESP_LOGI(TAG, "Front push detected! (delta: +%.2f, accel: %.2f m/s²)",
                 delta, current_accel_x);
        update_reaction_time();
        walk_forward_play(3);
        return;
    }
    
    // Back push: large negative delta AND current reading is negative
    // (acceleration suddenly increased in -X direction)
    if (delta <= -REACTION_DELTA_THRESHOLD && current_accel_x <= -REACTION_MIN_ACCEL) {
        ESP_LOGI(TAG, "Back push detected! (delta: %.2f, accel: %.2f m/s²)",
                 delta, current_accel_x);
        update_reaction_time();
        // TODO: Add backward reaction animation here
        ESP_LOGW(TAG, "Back push reaction not yet implemented");
        return;
    }
}
