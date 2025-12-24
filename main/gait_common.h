/**
 * @file gait_common.h
 * @brief Common definitions for all gait algorithms
 * 
 * Shared types and constants used across trot, walk, creep, and crawl gaits.
 * 
 * Servo ID Layout:
 *   Front: [1] Right  [2] Left
 *   Back:  [3] Right  [4] Left
 * 
 * Positive angle direction (clockwise) means:
 *   - Right legs (1, 3) move backward
 *   - Left legs (2, 4) move forward
 */

#ifndef GAIT_COMMON_H
#define GAIT_COMMON_H

#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════
// SERVO ID DEFINITIONS
// ═══════════════════════════════════════════════════════

#define SERVO_FRONT_RIGHT   1
#define SERVO_FRONT_LEFT    2
#define SERVO_BACK_RIGHT    3
#define SERVO_BACK_LEFT     4

// ═══════════════════════════════════════════════════════
// COMMON TYPES
// ═══════════════════════════════════════════════════════

/**
 * @brief Direction of walking (common to all gaits)
 */
typedef enum {
    GAIT_DIRECTION_FORWARD,
    GAIT_DIRECTION_BACKWARD,
    GAIT_DIRECTION_TURN_LEFT,
    GAIT_DIRECTION_TURN_RIGHT,
    GAIT_DIRECTION_STOP
} gait_direction_t;

/**
 * @brief Base configuration shared by all gaits
 */
typedef struct {
    float stance_angle_fr;      ///< Front-right neutral angle (degrees)
    float stance_angle_fl;      ///< Front-left neutral angle (degrees)
    float stance_angle_br;      ///< Back-right neutral angle (degrees)
    float stance_angle_bl;      ///< Back-left neutral angle (degrees)
    float swing_amplitude;      ///< Maximum angle deviation from stance (degrees)
    uint16_t step_duration_ms;  ///< Duration of each step phase (milliseconds)
    uint16_t servo_speed;       ///< Servo movement speed (0-4095)
} gait_config_t;

/**
 * @brief Default stance configuration
 * Stance angles: FR=270°, FL=90°, BR=90°, BL=270°
 */
#define GAIT_DEFAULT_STANCE() { \
    .stance_angle_fr = 270.0f,  \
    .stance_angle_fl = 90.0f,   \
    .stance_angle_br = 90.0f,   \
    .stance_angle_bl = 270.0f,  \
    .swing_amplitude = 30.0f,   \
    .step_duration_ms = 250,    \
    .servo_speed = 1000         \
}

#endif // GAIT_COMMON_H
