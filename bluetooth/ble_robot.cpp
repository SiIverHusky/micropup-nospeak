/**
 * @file ble_robot.cpp
 * @brief C-compatible BLE Robot Control Implementation
 * 
 * Bridges the C++ BleProtocol class to C code in main.c
 */

#include "ble_robot.h"
#include "ble_protocol.h"
#include "esp_log.h"
#include <memory>

static const char* TAG = "BLE_ROBOT";

// Global BLE protocol instance
static std::unique_ptr<BleProtocol> g_ble_protocol;

// C callback storage
static ble_servo_all_cb_t g_servo_all_cb = nullptr;
static ble_servo_single_cb_t g_servo_single_cb = nullptr;
static ble_connection_cb_t g_connection_cb = nullptr;
static ble_message_cb_t g_message_cb = nullptr;

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

bool ble_robot_init(void) {
    ESP_LOGI(TAG, "Initializing BLE robot control");
    
    if (g_ble_protocol) {
        ESP_LOGW(TAG, "BLE already initialized");
        return true;
    }
    
    g_ble_protocol = std::make_unique<BleProtocol>();
    
    // Register C++ callbacks that forward to C callbacks
    g_ble_protocol->OnServoAll([](const ServoAllCommand& cmd) {
        ESP_LOGI(TAG, "Servo all command received");
        if (g_servo_all_cb) {
            g_servo_all_cb(cmd.angle_fr, cmd.angle_fl, 
                           cmd.angle_br, cmd.angle_bl,
                           cmd.speed, cmd.delay_ms);
        }
    });
    
    g_ble_protocol->OnServoSingle([](const ServoSingleCommand& cmd) {
        ESP_LOGI(TAG, "Servo single command received");
        if (g_servo_single_cb) {
            g_servo_single_cb(cmd.id, cmd.angle, cmd.speed, cmd.delay_ms);
        }
    });
    
    g_ble_protocol->OnConnectionState([](bool connected) {
        ESP_LOGI(TAG, "BLE connection state: %s", connected ? "connected" : "disconnected");
        if (g_connection_cb) {
            g_connection_cb(connected);
        }
    });
    
    g_ble_protocol->OnMessage([](const std::string& msg) {
        ESP_LOGI(TAG, "Message received: %s", msg.c_str());
        if (g_message_cb) {
            g_message_cb(msg.c_str());
        }
    });
    
    // Start the BLE protocol
    if (!g_ble_protocol->Start()) {
        ESP_LOGE(TAG, "Failed to start BLE protocol");
        g_ble_protocol.reset();
        return false;
    }
    
    ESP_LOGI(TAG, "BLE robot control initialized - Device: %s", 
             BleProtocol::GetDeviceName());
    return true;
}

void ble_robot_stop(void) {
    if (g_ble_protocol) {
        g_ble_protocol->Stop();
        g_ble_protocol.reset();
        ESP_LOGI(TAG, "BLE robot control stopped");
    }
}

bool ble_robot_is_connected(void) {
    if (g_ble_protocol) {
        return g_ble_protocol->IsConnected();
    }
    return false;
}

// ═══════════════════════════════════════════════════════
// CALLBACK REGISTRATION
// ═══════════════════════════════════════════════════════

void ble_robot_on_servo_all(ble_servo_all_cb_t callback) {
    g_servo_all_cb = callback;
    ESP_LOGI(TAG, "Registered servo all callback");
}

void ble_robot_on_servo_single(ble_servo_single_cb_t callback) {
    g_servo_single_cb = callback;
    ESP_LOGI(TAG, "Registered servo single callback");
}

void ble_robot_on_connection(ble_connection_cb_t callback) {
    g_connection_cb = callback;
    ESP_LOGI(TAG, "Registered connection callback");
}

void ble_robot_on_message(ble_message_cb_t callback) {
    g_message_cb = callback;
    ESP_LOGI(TAG, "Registered message callback");
}

// ═══════════════════════════════════════════════════════
// SENDING DATA TO CLIENT
// ═══════════════════════════════════════════════════════

bool ble_robot_send_servo_state(float fr, float fl, float br, float bl) {
    if (g_ble_protocol) {
        return g_ble_protocol->SendServoState(fr, fl, br, bl);
    }
    return false;
}

bool ble_robot_send_response(const char* response) {
    if (g_ble_protocol && response) {
        return g_ble_protocol->SendResponse(std::string(response));
    }
    return false;
}

const char* ble_robot_get_device_name(void) {
    return BleProtocol::GetDeviceName();
}
