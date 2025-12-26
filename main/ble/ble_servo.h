/**
 * @file ble_servo.h
 * @brief Minimal BLE Servo Controller
 * 
 * Simple BLE interface for servo control - designed for gait animation iteration.
 * Receives servo commands via Web Bluetooth and executes them.
 * 
 * Commands (JSON):
 *   {"s":[fr,fl,br,bl,speed,delay]}  - Move all 4 servos
 *   {"m":[[fr,fl,br,bl,speed,delay], ...]}  - Sequence of moves
 *   {"p":1}  - Ping (returns {"p":1})
 *   {"r":1}  - Return to stance
 */

#ifndef BLE_SERVO_H
#define BLE_SERVO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ═══════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════

#define BLE_SERVO_DEVICE_NAME   "MicroPupper"

// ═══════════════════════════════════════════════════════
// CALLBACKS
// ═══════════════════════════════════════════════════════

/**
 * @brief Callback for servo movement command
 * @param fr Front-right angle (unified, auto-reversed for right side)
 * @param fl Front-left angle  
 * @param br Back-right angle (unified, auto-reversed for right side)
 * @param bl Back-left angle
 * @param speed Servo speed (0-4095)
 * @param delay_ms Delay after this move before next command
 */
typedef void (*ble_servo_move_cb_t)(float fr, float fl, float br, float bl, 
                                     uint16_t speed, uint16_t delay_ms);

/**
 * @brief Callback for return to stance
 */
typedef void (*ble_servo_stance_cb_t)(void);

/**
 * @brief Callback for connection state changes
 */
typedef void (*ble_servo_connect_cb_t)(bool connected);

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

/**
 * @brief Initialize and start BLE servo controller
 * @param move_cb Callback when servo move command received
 * @param stance_cb Callback to return to stance
 * @param connect_cb Callback for connection state (optional, can be NULL)
 * @return true if successful
 */
bool ble_servo_init(ble_servo_move_cb_t move_cb, 
                    ble_servo_stance_cb_t stance_cb,
                    ble_servo_connect_cb_t connect_cb);

/**
 * @brief Check if BLE is connected
 */
bool ble_servo_is_connected(void);

/**
 * @brief Send current servo positions back to client (for feedback)
 */
bool ble_servo_send_state(float fr, float fl, float br, float bl);

/**
 * @brief Send a simple response message
 */
bool ble_servo_send_response(const char* msg);

#ifdef __cplusplus
}
#endif

#endif // BLE_SERVO_H
