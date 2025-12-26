/**
 * @file dog_config.c
 * @brief Dog Hardware Configuration Implementation
 * 
 * Handles hardware initialization and servo control with automatic
 * angle reversal for right-side servos.
 */

#include "dog_config.h"
#include "sts3032_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DOG";

// ═══════════════════════════════════════════════════════
// INTERNAL STATE
// ═══════════════════════════════════════════════════════

static dog_config_t s_config;
static bool s_initialized = false;

// ═══════════════════════════════════════════════════════
// INTERNAL HELPERS
// ═══════════════════════════════════════════════════════

/**
 * @brief Apply angle reversal for right-side servos
 * @param servo_id The servo ID
 * @param angle Input angle (from unified left-side perspective)
 * @return Actual angle to send to servo
 */
static float apply_reversal(uint8_t servo_id, float angle)
{
    if (DOG_IS_RIGHT_SIDE(servo_id)) {
        return DOG_REVERSE_ANGLE(angle);
    }
    return angle;
}

/**
 * @brief Get base stance angle for a servo (before reversal)
 */
static float get_base_stance(uint8_t servo_id)
{
    if (DOG_IS_FRONT_LEG(servo_id)) {
        return s_config.stance_front;
    } else {
        return s_config.stance_back;
    }
}

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

bool dog_init(const dog_config_t *config)
{
    ESP_LOGI(TAG, "Initializing dog hardware");
    
    // Load configuration
    if (config != NULL) {
        s_config = *config;
    } else {
        dog_config_t default_config = DOG_DEFAULT_CONFIG();
        s_config = default_config;
    }
    
    ESP_LOGI(TAG, "UART: %d, TX: %d, RX: %d, TXEN: %d, Baud: %lu",
             s_config.uart_num, s_config.tx_pin, s_config.rx_pin,
             s_config.txen_pin, (unsigned long)s_config.baud_rate);
    
    // Initialize the servo protocol
    sts_protocol_config_t protocol_config = {
        .uart_num = s_config.uart_num,
        .tx_pin = s_config.tx_pin,
        .rx_pin = s_config.rx_pin,
        .txen_pin = s_config.txen_pin,
        .baud_rate = s_config.baud_rate,
    };
    
    esp_err_t ret = sts_protocol_init(&protocol_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize servo protocol: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Servo protocol initialized");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check all servos
    bool all_ok = dog_check_servos();
    if (!all_ok) {
        ESP_LOGW(TAG, "Some servos not responding, continuing anyway...");
    }
    
    // Enable torque on all servos
    dog_set_torque(true);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Go to stance position
    dog_goto_stance();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    s_initialized = true;
    
    ESP_LOGI(TAG, "Dog initialized - Stance: Front=%.0f° Back=%.0f° Amplitude=%.0f°",
             s_config.stance_front, s_config.stance_back, s_config.swing_amplitude);
    
    return all_ok;
}

const dog_config_t* dog_get_config(void)
{
    return &s_config;
}

// ═══════════════════════════════════════════════════════
// SERVO CONTROL
// ═══════════════════════════════════════════════════════

void dog_servo_move(uint8_t servo_id, float angle, uint16_t speed)
{
    float actual_angle = apply_reversal(servo_id, angle);
    sts_servo_set_angle(servo_id, actual_angle, speed);
}

void dog_servo_move_all(float angle_fr, float angle_fl, 
                        float angle_br, float angle_bl, uint16_t speed)
{
    // Apply reversal to right-side servos
    float actual_fr = apply_reversal(DOG_SERVO_FR, angle_fr);
    float actual_br = apply_reversal(DOG_SERVO_BR, angle_br);
    
    // Left side uses angles directly
    float actual_fl = angle_fl;
    float actual_bl = angle_bl;
    
    // Move all servos
    sts_servo_set_angle(DOG_SERVO_FR, actual_fr, speed);
    sts_servo_set_angle(DOG_SERVO_FL, actual_fl, speed);
    sts_servo_set_angle(DOG_SERVO_BR, actual_br, speed);
    sts_servo_set_angle(DOG_SERVO_BL, actual_bl, speed);
}

void dog_goto_stance(void)
{
    ESP_LOGI(TAG, "Moving to stance position");
    
    // Use unified angles - reversal happens automatically
    dog_servo_move_all(
        s_config.stance_front,  // FR
        s_config.stance_front,  // FL
        s_config.stance_back,   // BR
        s_config.stance_back,   // BL
        s_config.default_speed
    );
}

float dog_get_stance_angle(uint8_t servo_id)
{
    float base = get_base_stance(servo_id);
    return apply_reversal(servo_id, base);
}

float dog_get_swing_forward_angle(uint8_t servo_id)
{
    float base = get_base_stance(servo_id);
    float forward = base + s_config.swing_amplitude;  // Forward = positive offset
    return apply_reversal(servo_id, forward);
}

float dog_get_push_back_angle(uint8_t servo_id)
{
    float base = get_base_stance(servo_id);
    float back = base - s_config.swing_amplitude;  // Back = negative offset
    return apply_reversal(servo_id, back);
}

bool dog_check_servos(void)
{
    bool all_ok = true;
    
    ESP_LOGI(TAG, "Checking servos...");
    
    for (uint8_t id = 1; id <= DOG_SERVO_COUNT; id++) {
        if (sts_servo_ping(id)) {
            const char *name;
            switch (id) {
                case DOG_SERVO_FR: name = "Front-Right"; break;
                case DOG_SERVO_FL: name = "Front-Left"; break;
                case DOG_SERVO_BR: name = "Back-Right"; break;
                case DOG_SERVO_BL: name = "Back-Left"; break;
                default: name = "Unknown"; break;
            }
            ESP_LOGI(TAG, "  ✓ Servo %d (%s) OK", id, name);
        } else {
            ESP_LOGE(TAG, "  ✗ Servo %d NOT responding", id);
            all_ok = false;
        }
    }
    
    return all_ok;
}

void dog_set_torque(bool enable)
{
    for (uint8_t id = 1; id <= DOG_SERVO_COUNT; id++) {
        sts_servo_enable_torque(id, enable);
    }
    ESP_LOGI(TAG, "Torque %s on all servos", enable ? "enabled" : "disabled");
}
