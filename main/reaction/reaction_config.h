/**
 * @file reaction_config.h
 * @brief Configuration for user interaction reactions
 * 
 * Defines thresholds and parameters for detecting user pushes
 * and triggering appropriate animations.
 */

#ifndef REACTION_CONFIG_H
#define REACTION_CONFIG_H

#include <stdbool.h>
#include "qmi8658a.h"

// ═══════════════════════════════════════════════════════
// DELTA-BASED PUSH DETECTION
// ═══════════════════════════════════════════════════════
// Detects sudden changes in acceleration (impulse) rather than
// sustained thresholds. A push causes a large delta between readings.

// Minimum delta (change) in accel_x to detect a push (m/s² per sample)
// This is the difference between current and previous reading
#define REACTION_DELTA_THRESHOLD        50.0f    // m/s² change

// Minimum absolute acceleration to consider (filters out noise deltas)
#define REACTION_MIN_ACCEL              3.0f    // m/s²

// Minimum time between reactions (debounce)
#define REACTION_COOLDOWN_MS            2000    // 2 seconds

// Animation timing adjustment (added to each keyframe delay)
// Increase if animation is too fast, decrease if too slow
#define REACTION_TIMING_OFFSET_MS       100     // milliseconds

// ═══════════════════════════════════════════════════════
// GYRO STABILIZATION (Ground-Facing Mode)
// ═══════════════════════════════════════════════════════
// Uses gyro Y axis to keep legs perpendicular to ground.
// Since legs are 1 DoF, only pitch (Y axis) compensation is possible.

// Enable/disable gyro stabilization at startup (can be toggled at runtime)
#define REACTION_GYRO_STABILIZE_ENABLED_DEFAULT     true

// Maximum leg angle adjustment from neutral (degrees)
// Limits how much the stabilization can deviate from stance
#define REACTION_GYRO_MAX_CORRECTION    90.0f   // degrees

// Gyro deadzone - ignore small rotations (degrees/second)
#define REACTION_GYRO_DEADZONE          0.5f    // dps

// Proportional gain for gyro response
// Higher = more aggressive correction, but may oscillate
#define REACTION_GYRO_GAIN              1.6f    // degrees correction per dps

// Low-pass filter coefficient (0.0-1.0)
// Lower = smoother but slower response, Higher = faster but jittery
#define REACTION_GYRO_SMOOTHING         0.3f

// Update rate for stabilization (ms)
#define REACTION_GYRO_UPDATE_INTERVAL_MS    50  // 20Hz

// Dynamic speed configuration
// Speed scales based on correction magnitude with configurable curve
#define REACTION_GYRO_SPEED_MIN         150       // Speed for small corrections
#define REACTION_GYRO_SPEED_MAX         2000    // Speed for large corrections
#define REACTION_GYRO_SPEED_THRESHOLD   10.0f    // Correction angle for max speed (degrees)

// Speed curve exponent - controls how quickly speed ramps up
// 1.0 = linear, 2.0 = quadratic (slower ramp), 3.0 = cubic (even slower)
// Higher values keep speed low for small/medium changes
#define REACTION_GYRO_SPEED_CURVE       1.2f

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize the reaction system
 */
void reaction_init(void);

/**
 * @brief Process IMU data and trigger reactions if thresholds met
 * @param data Current IMU sensor data
 */
void reaction_process_imu(const qmi8658a_data_t *data);

// ═══════════════════════════════════════════════════════
// GYRO STABILIZATION CONTROL
// ═══════════════════════════════════════════════════════

/**
 * @brief Enable or disable gyro-based ground stabilization
 * @param enable true to enable, false to disable
 */
void reaction_gyro_stabilize_enable(bool enable);

/**
 * @brief Check if gyro stabilization is currently enabled
 * @return true if enabled
 */
bool reaction_gyro_stabilize_is_enabled(void);

#endif // REACTION_CONFIG_H
