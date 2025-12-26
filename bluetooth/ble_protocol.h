#ifndef _BLE_PROTOCOL_H
#define _BLE_PROTOCOL_H

#include "protocol.h"
#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include "esp_err.h"

// Robot BLE Protocol UUIDs
// Service UUID: 0d9be2a0-4757-43d9-83df-704ae274b8df
// Characteristic UUID: 8116d8c0-d45d-4fdf-998e-33ab8c471d59
#define ROBOT_SERVICE_UUID_128 \
    0xdf, 0xb8, 0x74, 0xe2, 0x4a, 0x70, 0xdf, 0x83, \
    0xd9, 0x43, 0x57, 0x47, 0xa0, 0xe2, 0x9b, 0x0d

#define ROBOT_CHARACTERISTIC_UUID_128 \
    0x59, 0x1d, 0x47, 0x8c, 0xab, 0x33, 0x8e, 0x99, \
    0xdf, 0x4f, 0x5d, 0xd4, 0xc0, 0xd8, 0x16, 0x81

// ═══════════════════════════════════════════════════════
// SERVO COMMAND TYPES
// ═══════════════════════════════════════════════════════

/**
 * @brief Command to move all 4 servos at once
 * JSON format: {"cmd":"servos","fr":90,"fl":90,"br":270,"bl":270,"speed":1000,"delay":100}
 */
struct ServoAllCommand {
    float angle_fr;
    float angle_fl;
    float angle_br;
    float angle_bl;
    uint16_t speed;
    uint16_t delay_ms;  // Delay before next command
};

/**
 * @brief Command to move a single servo
 * JSON format: {"cmd":"servo","id":1,"angle":90,"speed":1000,"delay":100}
 */
struct ServoSingleCommand {
    uint8_t id;
    float angle;
    uint16_t speed;
    uint16_t delay_ms;
};

/**
 * @brief MCP tool call command
 * JSON format: {"cmd":"mcp","tool":"tool_name","params":{...}}
 */
struct McpCommand {
    std::string tool_name;
    cJSON* params;  // Caller owns memory
};

// ═══════════════════════════════════════════════════════
// BLE PROTOCOL CLASS
// ═══════════════════════════════════════════════════════

class BleProtocol : public Protocol {
public:
    using CommandCallback = std::function<void(const std::string&)>;
    using ConnectionStateCallback = std::function<void(bool)>;
    using ServoAllCallback = std::function<void(const ServoAllCommand&)>;
    using ServoSingleCallback = std::function<void(const ServoSingleCommand&)>;
    using McpCallback = std::function<void(const McpCommand&)>;
    using MessageCallback = std::function<void(const std::string&)>;

    BleProtocol();
    ~BleProtocol();

    // Protocol interface implementation
    bool Start() override;
    void Stop();
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel() override;
    bool IsAudioChannelOpened() const override;

    // BLE specific methods
    bool IsConnected() const;
    
    // Register callbacks
    void OnCommand(CommandCallback callback);
    void OnConnectionState(ConnectionStateCallback callback);
    
    // Servo command callbacks
    void OnServoAll(ServoAllCallback callback);
    void OnServoSingle(ServoSingleCallback callback);
    
    // MCP and message callbacks (for porting to other projects)
    void OnMcpCommand(McpCallback callback);
    void OnMessage(MessageCallback callback);

    // Send response back to browser (supports chunking for large payloads)
    bool SendResponse(const std::string& response);
    
    // Send servo commands (for sending state back to client)
    bool SendServoState(float fr, float fl, float br, float bl);
    
    // Handle internal command (for MCP tool execution via BLE thread)
    bool HandleInternalCommand(const std::string& command);
    
    // Get the device name from configuration
    static const char* GetDeviceName();
    
    // Helper methods for processing commands (called from static callbacks)
    void ProcessJsonCommand(const cJSON* json);
    void ProcessTextCommand(const std::string& text);

protected:
    bool SendText(const std::string& text) override;

private:
    void* impl_;
    std::atomic<bool> audio_channel_opened_{false};
    
    // Callbacks
    CommandCallback command_callback_;
    ConnectionStateCallback connection_callback_;
    ServoAllCallback servo_all_callback_;
    ServoSingleCallback servo_single_callback_;
    McpCallback mcp_callback_;
    MessageCallback message_callback_;
    
    // Send large response in chunks
    bool SendChunkedResponse(const std::string& response);
    
    // Parse servo commands from JSON
    void ParseServoCommand(const cJSON* json);
    void ParseMcpCommand(const cJSON* json);
    
    // Constants for chunking
    static const size_t MAX_CHUNK_SIZE = 120;
};

#endif // _BLE_PROTOCOL_H