/**
 * @file ble_robot.h
 * @brief C-compatible BLE Robot Control Interface
 * 
 * Provides a C interface for the C++ BLE protocol class,
 * allowing main.c to use BLE for robot control.
 */

#ifndef BLE_ROBOT_H
#define BLE_ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════
// CALLBACK TYPES
// ═══════════════════════════════════════════════════════

/**
 * @brief Callback for moving all servos
 * @param angle_fr Front-right angle (unified, before reversal)
 * @param angle_fl Front-left angle
 * @param angle_br Back-right angle (unified, before reversal)
 * @param angle_bl Back-left angle
 * @param speed Servo speed
 * @param delay_ms Delay after movement (ms)
 */
typedef void (*ble_servo_all_cb_t)(float angle_fr, float angle_fl,
                                    float angle_br, float angle_bl,
                                    uint16_t speed, uint16_t delay_ms);

/**
 * @brief Callback for moving a single servo
 * @param id Servo ID (1-4)
 * @param angle Angle (unified, before reversal)
 * @param speed Servo speed
 * @param delay_ms Delay after movement (ms)
 */
typedef void (*ble_servo_single_cb_t)(uint8_t id, float angle,
                                       uint16_t speed, uint16_t delay_ms);

/**
 * @brief Callback for connection state changes
 * @param connected true if connected, false if disconnected
 */
typedef void (*ble_connection_cb_t)(bool connected);

/**
 * @brief Callback for generic messages
 * @param message The message text
 */
typedef void (*ble_message_cb_t)(const char* message);

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize and start the BLE robot control interface
 * @return true if successful
 */
bool ble_robot_init(void);

/**
 * @brief Stop the BLE interface
 */
void ble_robot_stop(void);

/**
 * @brief Check if BLE is connected
 * @return true if a device is connected
 */
bool ble_robot_is_connected(void);

// ═══════════════════════════════════════════════════════
// CALLBACK REGISTRATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Register callback for all-servo commands
 */
void ble_robot_on_servo_all(ble_servo_all_cb_t callback);

/**
 * @brief Register callback for single-servo commands
 */
void ble_robot_on_servo_single(ble_servo_single_cb_t callback);

/**
 * @brief Register callback for connection state changes
 */
void ble_robot_on_connection(ble_connection_cb_t callback);

/**
 * @brief Register callback for messages
 */
void ble_robot_on_message(ble_message_cb_t callback);

// ═══════════════════════════════════════════════════════
// SENDING DATA TO CLIENT
// ═══════════════════════════════════════════════════════

/**
 * @brief Send servo state to connected client
 * @param fr Front-right angle
 * @param fl Front-left angle
 * @param br Back-right angle
 * @param bl Back-left angle
 * @return true if sent successfully
 */
bool ble_robot_send_servo_state(float fr, float fl, float br, float bl);

/**
 * @brief Send a text response to connected client
 * @param response The response text
 * @return true if sent successfully
 */
bool ble_robot_send_response(const char* response);

/**
 * @brief Get the BLE device name
 * @return Device name string
 */
const char* ble_robot_get_device_name(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_ROBOT_H
