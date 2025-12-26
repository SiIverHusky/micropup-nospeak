/**
 * @file protocol.h
 * @brief Base Protocol Interface
 * 
 * Provides a base interface for communication protocols (BLE, WiFi, etc.)
 * This is a simplified version for the robot control project.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <string>
#include <functional>
#include <memory>
#include <cJSON.h>

/**
 * @brief Audio stream packet (placeholder for compatibility)
 */
struct AudioStreamPacket {
    std::vector<uint8_t> data;
};

/**
 * @brief Base protocol class for communication
 */
class Protocol {
public:
    using JsonCallback = std::function<void(const cJSON*)>;
    using VoidCallback = std::function<void()>;

    virtual ~Protocol() = default;

    // Core protocol methods
    virtual bool Start() = 0;
    virtual bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) { return false; }
    virtual bool OpenAudioChannel() { return false; }
    virtual void CloseAudioChannel() {}
    virtual bool IsAudioChannelOpened() const { return false; }

    // Register callbacks
    void OnIncomingJson(JsonCallback callback) { on_incoming_json_ = callback; }
    void OnAudioChannelOpened(VoidCallback callback) { on_audio_channel_opened_ = callback; }
    void OnAudioChannelClosed(VoidCallback callback) { on_audio_channel_closed_ = callback; }

protected:
    virtual bool SendText(const std::string& text) = 0;

    JsonCallback on_incoming_json_;
    VoidCallback on_audio_channel_opened_;
    VoidCallback on_audio_channel_closed_;
};

#endif // PROTOCOL_H
