#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>

#include "usbipdcpp/virtual_device/VirtualInterfaceHandler.h"
#include "usbipdcpp/Device.h"
#include "usbipdcpp/Session.h"

#define USB_AUDIO_CHANNEL_L    (1 << 0)  // Left Front
#define USB_AUDIO_CHANNEL_R    (1 << 1)  // Right Front
#define USB_AUDIO_CHANNEL_C    (1 << 2)  // Center Front
#define USB_AUDIO_CHANNEL_LFE  (1 << 3)  // Low Frequency Enhancement
#define USB_AUDIO_CHANNEL_LS   (1 << 4)  // Left Surround
#define USB_AUDIO_CHANNEL_RS   (1 << 5)  // Right Surround
#define USB_AUDIO_CHANNEL_LC   (1 << 6)  // Left of Center
#define USB_AUDIO_CHANNEL_RC   (1 << 7)  // Right of Center
#define USB_AUDIO_CHANNEL_S    (1 << 8)  // Surround
#define USB_AUDIO_CHANNEL_SL   (1 << 9)  // Side Left
#define USB_AUDIO_CHANNEL_SR   (1 << 10) // Side Right
#define USB_AUDIO_CHANNEL_T    (1 << 11) // Top

namespace usbipdcpp {

constexpr std::uint8_t AUDIO_CONTROL = 0x01;
constexpr std::uint8_t AUDIO_STREAMING = 0x02;

constexpr std::uint8_t AUDIO_IP_VERSION_00_00 = 0x01;

enum class UacRole : std::uint8_t {
    Microphone = 1 << 0,
    Speaker    = 1 << 1,
    Both       = Microphone | Speaker
};

class IsoThrottler {
private:
    std::chrono::steady_clock::time_point next_release_time_;
    std::chrono::microseconds interval_us_;
    std::mutex mtx_;

public:
    explicit IsoThrottler(uint32_t packets_per_sec) 
        : interval_us_(1000000 / packets_per_sec) {
        next_release_time_ = std::chrono::steady_clock::now();
    }

    void wait_for_slot() {
        std::lock_guard<std::mutex> lock(mtx_);
        auto now = std::chrono::steady_clock::now();
        
        if (now < next_release_time_) {
            std::this_thread::sleep_until(next_release_time_);
        } else {
            next_release_time_ = now;
        }
        
        next_release_time_ += interval_us_;
    }
};

class UacAudioStreamingHandler;

class AudioSource {
public:
    virtual ~AudioSource() = default;
    virtual std::size_t read_audio(std::uint8_t* buffer, std::size_t max_bytes) = 0;
    virtual std::uint32_t sample_rate() const { return 48000; }
    virtual std::uint8_t channels() const { return 1; }
    virtual std::uint16_t channels_config() {
        std::uint16_t config = 0;
        if (channels() >= 1) config |= USB_AUDIO_CHANNEL_L;
        if (channels() >= 2) config |= USB_AUDIO_CHANNEL_R;
        return config;
    }
};

class AudioSink {
public:
    explicit AudioSink() = default;
    virtual ~AudioSink() = default;
    virtual void write_audio(const std::uint8_t* buffer, std::size_t bytes) = 0;
    virtual std::uint32_t sample_rate() const { return 48000; }
    virtual std::uint8_t channels() const { return 1; }
    virtual std::uint16_t calculate_expected_bytes_per_packet() const {
        // freq * s16le * channels
        return ((sample_rate() / 1000) + 1) * sizeof(std::int16_t) * channels();
    }
    virtual std::uint16_t channels_config() {
        std::uint16_t config = 0;
        if (channels() >= 1) config |= USB_AUDIO_CHANNEL_L;
        if (channels() >= 2) config |= USB_AUDIO_CHANNEL_R;
        return config;
    }
};

class UacAudioControlHandler : public VirtualInterfaceHandler {
public:
    explicit UacAudioControlHandler(UsbInterface &handle_interface, StringPool &string_pool, UacRole role);
    
    void on_setup_interface_handlers() override;
    void build_class_descriptor();
    data_type get_class_specific_descriptor() override;
    
    void handle_non_standard_request_type_control_urb(
        std::uint32_t seqnum, const UsbEndpoint &ep, std::uint32_t transfer_flags,
        std::uint32_t transfer_buffer_length, const SetupPacket &setup_packet,
        TransferHandle transfer, std::error_code &ec) override;
        
    data_type request_get_descriptor(std::uint8_t type, std::uint8_t language_id,
                                     std::uint16_t descriptor_length, std::uint32_t *p_status) override;

    void request_clear_feature(std::uint16_t feature_selector, std::uint32_t *p_status) override { *p_status = 0; }
    void request_endpoint_clear_feature(std::uint16_t feature_selector, std::uint8_t ep_address, std::uint32_t *p_status) override { *p_status = 0; }
    std::uint8_t request_get_interface(std::uint32_t *p_status) override { *p_status = 0; return 0; }
    void request_set_interface(std::uint16_t alternate_setting, std::uint32_t *p_status) override { *p_status = 0; }
    std::uint16_t request_get_status(std::uint32_t *p_status) override { return 0; }
    std::uint16_t request_endpoint_get_status(std::uint8_t ep_address, std::uint32_t *p_status) override { return 0; }
    void request_set_feature(std::uint16_t feature_selector, std::uint32_t *p_status) override { *p_status = 0; }
    void request_endpoint_set_feature(std::uint16_t feature_selector, std::uint8_t ep_address, std::uint32_t *p_status) override { *p_status = 0; }
    void handle_unlink_seqnum(std::uint32_t unlink_seqnum, std::uint32_t cmd_seqnum) override;

    void set_ass_handler(UacAudioStreamingHandler *handler) {
        ass_handler_ = handler;
    }
    void set_asm_handler(UacAudioStreamingHandler *handler) {
        asm_handler_ = handler;
    }

private:
    UacRole role_;
    data_type class_desc_;
    bool desc_built_ = false;
    
    bool mic_mute_ = false;
    std::int16_t mic_volume_ = -3072; // -12 dB
    
    bool spk_mute_ = false;
    std::int16_t spk_volume_ = -3072; // -12 dB
    UacAudioStreamingHandler *asm_handler_ = nullptr;
    UacAudioStreamingHandler *ass_handler_ = nullptr;
};

struct PendingResponse {
    std::uint32_t seqnum;
    std::uint32_t actual_length;
    std::uint32_t num_iso_packets;
    TransferHandle transfer;
    std::chrono::steady_clock::time_point send_time;
};

class UacAudioStreamingHandler : public VirtualInterfaceHandler {
public:
    explicit UacAudioStreamingHandler(UsbInterface &handle_interface, StringPool &string_pool, 
                                      bool is_input, 
                                      std::shared_ptr<AudioSource> source, 
                                      std::shared_ptr<AudioSink> sink);
    
    void on_setup_interface_handlers() override;
    void build_class_descriptor();
    data_type get_class_specific_descriptor() override;
    
    void handle_non_standard_request_type_control_urb(
        std::uint32_t seqnum, const UsbEndpoint &ep, std::uint32_t transfer_flags,
        std::uint32_t transfer_buffer_length, const SetupPacket &setup_packet,
        TransferHandle transfer, std::error_code &ec) override;
        
    void handle_isochronous_transfer(
        std::uint32_t seqnum, const UsbEndpoint &ep, std::uint32_t transfer_flags,
        std::uint32_t transfer_buffer_length, TransferHandle transfer, int num_iso_packets,
        std::error_code &ec) override;
        
    void request_set_interface(std::uint16_t alternate_setting, std::uint32_t *p_status) override;
    std::uint8_t request_get_interface(std::uint32_t *p_status) override;
    data_type request_get_descriptor(std::uint8_t type, std::uint8_t language_id,
                                     std::uint16_t descriptor_length, std::uint32_t *p_status) override;
                                     
    void on_disconnection(std::error_code &ec) override;

    void request_clear_feature(std::uint16_t feature_selector, std::uint32_t *p_status) override { *p_status = 0; }
    void request_endpoint_clear_feature(std::uint16_t feature_selector, std::uint8_t ep_address, std::uint32_t *p_status) override { *p_status = 0; }
    std::uint16_t request_get_status(std::uint32_t *p_status) override { return 0; }
    std::uint16_t request_endpoint_get_status(std::uint8_t ep_address, std::uint32_t *p_status) override { return 0; }
    void request_set_feature(std::uint16_t feature_selector, std::uint32_t *p_status) override { *p_status = 0; }
    void request_endpoint_set_feature(std::uint16_t feature_selector, std::uint8_t ep_address, std::uint32_t *p_status) override { *p_status = 0; }
    void handle_unlink_seqnum(std::uint32_t unlink_seqnum, std::uint32_t cmd_seqnum) override;

    std::shared_ptr<AudioSource> source() const { return source_; }
    std::shared_ptr<AudioSink> sink() const { return sink_; }
    ~UacAudioStreamingHandler() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            thread_running_ = false;
        }
        queue_cv.notify_all();
        if (runner_thread.joinable()) {
            runner_thread.join();
        }
    }
private:
    bool is_input_;
    std::shared_ptr<AudioSource> source_;
    std::shared_ptr<AudioSink> sink_;
    data_type class_desc_;
    bool desc_built_ = false;
    bool streaming_ = false;
    std::uint32_t sample_rate_ = 48000;
    std::uint32_t channels_ = 1;

    std::chrono::steady_clock::time_point stream_start_time_;
    std::uint32_t initial_frame_offset_;
    std::unique_ptr<IsoThrottler> throttler_;

    std::queue<PendingResponse> response_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::thread runner_thread;
    bool thread_running_ = true;

    std::chrono::steady_clock::time_point last_planned_time = std::chrono::steady_clock::now();
    std::mutex time_mutex;
    std::vector<std::uint32_t> unlinked_seqnums_;
};

class UacDeviceHelper {
public:
    static void setup(std::shared_ptr<UsbDevice> device, StringPool &string_pool,
                      UacRole role, std::shared_ptr<AudioSource> source, std::shared_ptr<AudioSink> sink, std::uint8_t start_interface_index);
};

} // namespace usbipdcpp