#include "UacVirtualInterfaceHandler.h"

#include <cstring>
#include <algorithm>
#include <stdexcept>

#include "usbipdcpp/Device.h"
#include "usbipdcpp/Session.h"
#include "usbipdcpp/protocol.h"
#include "spdlog/spdlog.h"
#include "usbipdcpp/virtual_device/SimpleVirtualDeviceHandler.h"
#include <numeric>
#include <thread>

namespace usbipdcpp {

constexpr std::uint8_t CS_INTERFACE = 0x24;
constexpr std::uint8_t CS_ENDPOINT  = 0x25;

constexpr std::uint8_t AC_HEADER          = 0x01;
constexpr std::uint8_t AC_INPUT_TERMINAL  = 0x02;
constexpr std::uint8_t AC_OUTPUT_TERMINAL = 0x03;
constexpr std::uint8_t AC_FEATURE_UNIT    = 0x06;

constexpr std::uint8_t AS_GENERAL     = 0x01;
constexpr std::uint8_t AS_FORMAT_TYPE = 0x02;
constexpr std::uint8_t EP_GENERAL     = 0x01;

constexpr std::uint8_t GET_CUR  = 0x81;
constexpr std::uint8_t SET_CUR  = 0x01;
constexpr std::uint8_t GET_MIN  = 0x82;
constexpr std::uint8_t GET_MAX  = 0x83;
constexpr std::uint8_t GET_RES  = 0x84;
constexpr std::uint8_t GET_INFO = 0x86;

// ==================== UacAudioControlHandler ====================

UacAudioControlHandler::UacAudioControlHandler(UsbInterface &handle_interface, StringPool &string_pool, UacRole role) :
    VirtualInterfaceHandler(handle_interface, string_pool), role_(role) {
}

void UacAudioControlHandler::on_setup_interface_handlers() {
    build_class_descriptor();
}

void UacAudioControlHandler::build_class_descriptor() {
    if (desc_built_) return;
    desc_built_ = true;

    data_type d;
    std::uint8_t num_streaming_ifs = 0;
    if ((static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Microphone))) num_streaming_ifs++;
    if ((static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Speaker))) num_streaming_ifs++;

    std::uint16_t bcdADC = 0x0100;
    std::uint16_t total_ac_len = 9;
    if ((static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Microphone))) total_ac_len += 30;
    if ((static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Speaker))) total_ac_len += 30;

    // 1. AC Header
    d.insert(d.end(), {
        0x09, CS_INTERFACE, AC_HEADER,
        static_cast<std::uint8_t>(bcdADC & 0xFF), static_cast<std::uint8_t>((bcdADC >> 8) & 0xFF),
        static_cast<std::uint8_t>(total_ac_len & 0xFF), static_cast<std::uint8_t>((total_ac_len >> 8) & 0xFF),
        num_streaming_ifs
    });
    
    std::uint8_t current_if = 1;
    if ((static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Microphone))) d.push_back(current_if++);
    if ((static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Speaker))) d.push_back(current_if++);

    // --- Microphone ---
    if ((static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Microphone))) {
        auto channels_config = asm_handler_ ? asm_handler_->source()->channels_config() : 0x0000;
        d.insert(d.end(), {
            0x0C, CS_INTERFACE, AC_INPUT_TERMINAL,
            0x01,       // bTerminalID
            0x01, 0x02, // wTerminalType: Microphone (0x0201)
            0x00,       // bAssocTerminal
            static_cast<std::uint8_t>(asm_handler_ ? asm_handler_->source()->channels() : 0x01),       // bNrChannels
            static_cast<std::uint8_t>(channels_config & 0xFF),       // wChannelConfig (low byte)
            static_cast<std::uint8_t>((channels_config >> 8) & 0xFF), // wChannelConfig (high byte)
            0x00,       // iChannelNames
            0x00        // iTerminal
        });
        d.insert(d.end(), {
            0x09, CS_INTERFACE, AC_FEATURE_UNIT,
            0x02,       // bUnitID
            0x01,       // bSourceID
            0x01,       // bControlSize
            0x01, 0x02, // bmaControls
            0x00        // iFeature
        });
        d.insert(d.end(), {
            0x09, CS_INTERFACE, AC_OUTPUT_TERMINAL,
            0x03,       // bTerminalID
            0x01, 0x01, // wTerminalType: USB Streaming (0x0101)
            0x00,       // bAssocTerminal
            0x02,       // bSourceID
            0x00        // iTerminal
        });
    }

    // --- Speaker ---
    if ((static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Speaker))) {
        auto channels_config = ass_handler_ ? ass_handler_->sink()->channels_config() : 0x0000;
        d.insert(d.end(), {
            0x0C, CS_INTERFACE, AC_INPUT_TERMINAL, 
            0x04,       // bTerminalID
            0x01, 0x01, // wTerminalType: USB Streaming (0x0101)
            0x00,       // bAssocTerminal
            static_cast<std::uint8_t>(ass_handler_ ? ass_handler_->sink()->channels() : 0x01),       // bNrChannels
            static_cast<std::uint8_t>(channels_config & 0xFF),       // wChannelConfig (low byte)
            static_cast<std::uint8_t>((channels_config >> 8) & 0xFF), // wChannelConfig (high byte)
            0x00,       // iChannelNames
            0x00        // iTerminal
        });
        d.insert(d.end(), {
            0x09, CS_INTERFACE, AC_FEATURE_UNIT, 
            0x05,       // bUnitID
            0x04,       // bSourceID
            0x01,       // bControlSize
            0x01, 0x02, // bmaControls
            0x00        // iFeature
        });
        d.insert(d.end(), {
            0x09, CS_INTERFACE, AC_OUTPUT_TERMINAL, 
            0x06,       // bTerminalID
            0x01, 0x03, // wTerminalType: Speaker (0x0301)
            0x00,       // bAssocTerminal
            0x05,       // bSourceID
            0x00        // iTerminal
        });
    }

    class_desc_ = std::move(d);
}

data_type UacAudioControlHandler::get_class_specific_descriptor() {
    return class_desc_;
}

data_type UacAudioControlHandler::request_get_descriptor(std::uint8_t type, std::uint8_t language_id,
                                                         std::uint16_t descriptor_length, std::uint32_t *p_status) {
    if (type == CS_INTERFACE) return class_desc_;
    return VirtualInterfaceHandler::request_get_descriptor(type, language_id, descriptor_length, p_status);
}

void UacAudioControlHandler::handle_non_standard_request_type_control_urb(
    std::uint32_t seqnum, const UsbEndpoint &ep, std::uint32_t transfer_flags,
    std::uint32_t transfer_buffer_length, const SetupPacket &setup_packet,
    TransferHandle transfer, std::error_code &ec) {
    auto type = static_cast<RequestType>(setup_packet.calc_request_type());
    if (type != RequestType::Class) {
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }

    auto entity = setup_packet.index >> 8;
    auto control_selector = setup_packet.value >> 8;
    auto request = setup_packet.request;
    auto *trx = GenericTransfer::from_handle(transfer.get());

    if (request == static_cast<std::uint8_t>(StandardRequest::GetDescriptor)) {
        auto desc_type = setup_packet.value >> 8;
        if (desc_type == CS_INTERFACE) {
            auto resp = class_desc_;
            auto act_len = std::min(resp.size(), static_cast<std::size_t>(transfer_buffer_length));
            trx->data.assign(resp.begin(), resp.begin() + act_len);
            trx->actual_length = act_len;
            session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_iso(
                seqnum, static_cast<std::uint16_t>(UrbStatusType::StatusOK),
                static_cast<std::uint32_t>(trx->actual_length), std::move(transfer)));
            return;
        }
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }

    // Microphone (Entity ID = 2)
    if (entity == 0x02 && (static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Microphone))) {
        if (control_selector == 0x01) { // Mute
            if (request == GET_CUR) { trx->data = {static_cast<std::uint8_t>(mic_mute_ ? 1 : 0)}; trx->actual_length = 1; }
            else if (request == SET_CUR) { if (!trx->data.empty()) mic_mute_ = (trx->data[0] != 0); trx->actual_length = 0; }
            else if (request == GET_INFO) { trx->data = {0x03}; trx->actual_length = 1; }
            else { session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0)); return; }
        } else if (control_selector == 0x02) { // Volume
            if (request == GET_CUR) { trx->data = {static_cast<std::uint8_t>(mic_volume_ & 0xFF), static_cast<std::uint8_t>((mic_volume_ >> 8) & 0xFF)}; trx->actual_length = 2; }
            else if (request == GET_MIN) { trx->data = {0x00, 0x80}; trx->actual_length = 2; }
            else if (request == GET_MAX) { trx->data = {0x00, 0x00}; trx->actual_length = 2; }
            else if (request == GET_RES) { trx->data = {0x00, 0x01}; trx->actual_length = 2; }
            else if (request == SET_CUR) { if (trx->data.size() >= 2) mic_volume_ = static_cast<std::int16_t>(trx->data[0] | (trx->data[1] << 8)); trx->actual_length = 0; }
            else if (request == GET_INFO) { trx->data = {0x03}; trx->actual_length = 1; }
            else { session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0)); return; }
        } else { session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0)); return; }
    }
    // Speaker (Entity ID = 5)
    else if (entity == 0x05 && (static_cast<std::uint8_t>(role_) & static_cast<std::uint8_t>(UacRole::Speaker))) {
        if (control_selector == 0x01) { // Mute
            if (request == GET_CUR) { trx->data = {static_cast<std::uint8_t>(spk_mute_ ? 1 : 0)}; trx->actual_length = 1; }
            else if (request == SET_CUR) { if (!trx->data.empty()) spk_mute_ = (trx->data[0] != 0); trx->actual_length = 0; }
            else if (request == GET_INFO) { trx->data = {0x03}; trx->actual_length = 1; }
            else { session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0)); return; }
        } else if (control_selector == 0x02) { // Volume
            if (request == GET_CUR) { trx->data = {static_cast<std::uint8_t>(spk_volume_ & 0xFF), static_cast<std::uint8_t>((spk_volume_ >> 8) & 0xFF)}; trx->actual_length = 2; }
            else if (request == GET_MIN) { trx->data = {0x00, 0x80}; trx->actual_length = 2; }
            else if (request == GET_MAX) { trx->data = {0x00, 0x00}; trx->actual_length = 2; }
            else if (request == GET_RES) { trx->data = {0x00, 0x01}; trx->actual_length = 2; }
            else if (request == SET_CUR) { if (trx->data.size() >= 2) spk_volume_ = static_cast<std::int16_t>(trx->data[0] | (trx->data[1] << 8)); trx->actual_length = 0; }
            else if (request == GET_INFO) { trx->data = {0x03}; trx->actual_length = 1; }
            else { session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0)); return; }
        } else { session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0)); return; }
    } else {
        trx->actual_length = 0;
    }

    session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_iso(
        seqnum, static_cast<std::uint16_t>(UrbStatusType::StatusOK),
        static_cast<std::uint32_t>(trx->actual_length), std::move(transfer)));
}

void UacAudioControlHandler::handle_unlink_seqnum(std::uint32_t unlink_seqnum, std::uint32_t cmd_seqnum) {
    std::lock_guard lock(endpoint_requests_mutex_);
    session->submit_ret_unlink(UsbIpResponse::UsbIpRetUnlink::create_ret_unlink_success(cmd_seqnum));
}


// ==================== UacAudioStreamingHandler ====================

UacAudioStreamingHandler::UacAudioStreamingHandler(UsbInterface &handle_interface, StringPool &string_pool, 
                                                   bool is_input, 
                                                   std::shared_ptr<AudioSource> source, 
                                                   std::shared_ptr<AudioSink> sink) :
    VirtualInterfaceHandler(handle_interface, string_pool), 
    is_input_(is_input), source_(std::move(source)), sink_(std::move(sink)) {
    if (source_) {
        sample_rate_ = source_->sample_rate();
    }
    if (sink_) {
        sample_rate_ = sink_->sample_rate();
    }

    throttler_ = std::make_unique<IsoThrottler>(1000);
    stream_start_time_ = std::chrono::steady_clock::now();
    initial_frame_offset_ = 10000;

    runner_thread = std::thread([this]() {
        while (thread_running_) {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_cv.wait(lock, [this]{ return !response_queue.empty() || !thread_running_; });
            if (!thread_running_) break;

            PendingResponse resp = std::move(response_queue.front());
            response_queue.pop();
            lock.unlock();

            auto *trx = GenericTransfer::from_handle(resp.transfer.get());
            auto &data = trx->data;
            auto &iso_descs = trx->iso_descriptors;

            for (int i = 0; i < resp.num_iso_packets; ++i) {
                auto &iso = iso_descs[i];
                if (iso.length == 0) continue;

                if (sink_) {
                    sink_->write_audio(&data[iso.offset], iso.length);
                }
                iso.actual_length = iso.length;
                // total_sent += iso.length;
            }

            auto now = std::chrono::steady_clock::now();
            if (now < resp.send_time) {
                std::this_thread::sleep_until(resp.send_time);
            }

            auto received_size = static_cast<std::uint32_t>(trx->data.size());
            if (std::find(unlinked_seqnums_.begin(), unlinked_seqnums_.end(), resp.seqnum) != unlinked_seqnums_.end()) {
                unlinked_seqnums_.erase(std::remove(unlinked_seqnums_.begin(), unlinked_seqnums_.end(), resp.seqnum), unlinked_seqnums_.end());
                spdlog::info("UAC: Skipping sending audio data for seqnum {} as it has been unlinked.", resp.seqnum);
                // session->submit_ret_submit(
                //     UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(resp.seqnum, 0));
                continue;
            }
            if (!streaming_) continue;
            session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                resp.seqnum, static_cast<std::uint16_t>(UrbStatusType::StatusOK), received_size, 0,
                static_cast<std::uint16_t>(iso_descs.size()), std::move(resp.transfer)));
        }
    });
}

void UacAudioStreamingHandler::on_setup_interface_handlers() {
    build_class_descriptor();
}

void UacAudioStreamingHandler::build_class_descriptor() {
    if (desc_built_) return;
    desc_built_ = true;

    data_type d;
    std::uint8_t terminal_link = is_input_ ? 0x06 : 0x01;

    // 1. AS General Descriptor
    d.insert(d.end(), {
        0x07,           // bLength
        CS_INTERFACE,   // bDescriptorType
        AS_GENERAL,     // bDescriptorSubtype
        terminal_link,  // bTerminalLink
        0x01,           // bDelay
        0x01, 0x00      // wFormatTag
    });

    // 2. Format Type I Descriptor
    std::uint32_t sr = sample_rate_;
    d.insert(d.end(), {
        0x0B,                                           // bLength
        CS_INTERFACE,                                   // bDescriptorType
        AS_FORMAT_TYPE,                                 // bDescriptorSubtype
        0x01,                                           // bFormatType (PCM)
        static_cast<std::uint8_t>(is_input_ ? source_->channels() : sink_->channels()),       // bNrChannels
        0x02,                                           // bSubFrameSize (2 bytes for 16-bit audio)
        0x10,                                           // bBitResolution (16 bits)
        0x01,                                           // bSamFreqType (1 discrete frequency)
        static_cast<std::uint8_t>(sr & 0xFF),          // tSamFreq (low byte)
        static_cast<std::uint8_t>((sr >> 8) & 0xFF),   // tSamFreq (middle byte)
        static_cast<std::uint8_t>((sr >> 16) & 0xFF)   // tSamFreq (high byte)
    });

    class_desc_ = std::move(d);
}

data_type UacAudioStreamingHandler::get_class_specific_descriptor() {
    return class_desc_;
}

data_type UacAudioStreamingHandler::request_get_descriptor(std::uint8_t type, std::uint8_t language_id,
                                                           std::uint16_t descriptor_length, std::uint32_t *p_status) {
    if (type == CS_INTERFACE) return class_desc_;
    if (type == CS_ENDPOINT) return {0x07, CS_ENDPOINT, EP_GENERAL, 0x01, 0x00, 0x00, 0x00};
    return VirtualInterfaceHandler::request_get_descriptor(type, language_id, descriptor_length, p_status);
}

void UacAudioStreamingHandler::handle_non_standard_request_type_control_urb(
    std::uint32_t seqnum, const UsbEndpoint &ep, std::uint32_t transfer_flags,
    std::uint32_t transfer_buffer_length, const SetupPacket &setup_packet,
    TransferHandle transfer, std::error_code &ec) {
    SPDLOG_INFO("UAC Control Request: seqnum={}, request=0x{:02X}, value=0x{:04X}, index=0x{:04X}, length={}",
                 seqnum, setup_packet.request, setup_packet.value, setup_packet.index, setup_packet.length);
    auto type = static_cast<RequestType>(setup_packet.calc_request_type());
    if (type != RequestType::Class) {
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }

    auto request = setup_packet.request;
    auto *trx = GenericTransfer::from_handle(transfer.get());

    if (request == static_cast<std::uint8_t>(StandardRequest::GetDescriptor)) {
        auto desc_type = setup_packet.value >> 8;
        if (desc_type == CS_INTERFACE || desc_type == CS_ENDPOINT) {
            auto resp = request_get_descriptor(desc_type, 0, transfer_buffer_length, nullptr);
            auto act_len = std::min(resp.size(), static_cast<std::size_t>(transfer_buffer_length));
            trx->data.assign(resp.begin(), resp.begin() + act_len);
            trx->actual_length = act_len;
            session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_iso(
                seqnum, static_cast<std::uint16_t>(UrbStatusType::StatusOK),
                static_cast<std::uint32_t>(trx->actual_length), std::move(transfer)));
            return;
        }
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }

    // Sampling Frequency Control
    if (request == GET_CUR || request == GET_MIN || request == GET_MAX || request == GET_RES) {
        trx->data = {
            static_cast<std::uint8_t>(sample_rate_ & 0xFF),
            static_cast<std::uint8_t>((sample_rate_ >> 8) & 0xFF),
            static_cast<std::uint8_t>((sample_rate_ >> 16) & 0xFF)
        };
        trx->actual_length = 3;
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_iso(
            seqnum, static_cast<std::uint16_t>(UrbStatusType::StatusOK),
            static_cast<std::uint32_t>(trx->actual_length), std::move(transfer)));
    } else if (request == SET_CUR) {
        if (trx->data.size() >= 3) {
            sample_rate_ = trx->data[0] | (trx->data[1] << 8) | (trx->data[2] << 16);
        }
        trx->actual_length = 0;
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_data(
            seqnum, static_cast<std::uint16_t>(UrbStatusType::StatusOK), transfer_buffer_length));
    } else if (request == GET_INFO) {
        trx->data = {0x03};
        trx->actual_length = 1;
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_iso(
            seqnum, static_cast<std::uint16_t>(UrbStatusType::StatusOK),
            static_cast<std::uint32_t>(trx->actual_length), std::move(transfer)));
    } else {
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
    }
}

void UacAudioStreamingHandler::handle_isochronous_transfer(
    std::uint32_t seqnum, const UsbEndpoint &ep, std::uint32_t transfer_flags,
    std::uint32_t transfer_buffer_length, TransferHandle transfer, int num_iso_packets,
    std::error_code &ec) {
    
    if (!streaming_) {
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }

    std::uint32_t current_start_frame = 0;
    // if (is_input_) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - stream_start_time_).count();
    current_start_frame = initial_frame_offset_ + static_cast<std::uint32_t>(elapsed_ms);
    // }

    // static int urb_count = 0;
    // if (urb_count++ < 5) {
    //     spdlog::info("ISO URB: total_len={}, num_packets={}", transfer_buffer_length, num_iso_packets);
    //     auto *trx = GenericTransfer::from_handle(transfer.get());
    //     for (int i = 0; i < num_iso_packets; ++i) {
    //         spdlog::info("  Packet {}: offset={}, length={}, actual_length={}", i, trx->iso_descriptors[i].offset, trx->iso_descriptors[i].length, trx->iso_descriptors[i].actual_length);
    //     }
    // }

    auto *trx = GenericTransfer::from_handle(transfer.get());
    auto &data = trx->data;
    auto &iso_descs = trx->iso_descriptors;

    for (auto &iso : iso_descs) {
        iso.status = 0;
    }

    std::uint32_t total_sent = 0;

    if (ep.is_in()) {
        for (int i = 0; i < num_iso_packets; ++i) {
            auto &iso = iso_descs[i];
            if (iso.length == 0) continue;

            std::vector<std::uint8_t> audio_chunk(iso.length, 0);
            if (source_) {
                std::size_t read_bytes = source_->read_audio(audio_chunk.data(), audio_chunk.size());
                if (read_bytes < iso.length) {
                    std::fill(audio_chunk.begin() + read_bytes, audio_chunk.end(), 0);
                }
            }
            std::memcpy(&data[iso.offset], audio_chunk.data(), audio_chunk.size());
            iso.actual_length = audio_chunk.size();
            total_sent += iso.actual_length;
        }
    } else {
        double audio_duration_ms = static_cast<double>(transfer_buffer_length) / sink_->calculate_expected_bytes_per_packet();
        auto urb_interval = std::chrono::microseconds(static_cast<int>(audio_duration_ms * 1000.0));

        auto now2 = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point send_time;
        
        {
            std::lock_guard<std::mutex> lock(time_mutex);
            auto planned = last_planned_time + urb_interval;
            
            if (planned < now2) {
                send_time = now2;
            } else {
                send_time = planned;
            }
            
            last_planned_time = send_time;
        }

        PendingResponse resp;
        resp.seqnum = seqnum;
        resp.actual_length = transfer_buffer_length;
        resp.num_iso_packets = num_iso_packets;
        resp.transfer = std::move(transfer);
        static std::chrono::steady_clock::time_point last_send_time = send_time;
        resp.send_time = send_time;

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            response_queue.push(std::move(resp));
        }
        queue_cv.notify_one();
    }

    // std::this_thread::sleep_for(std::chrono::microseconds(iso_descs.size() * 1000));

    // static std::uint64_t total_bytes_sent = 0;
    // static auto start_time = std::chrono::steady_clock::now();

    // total_bytes_sent += total_sent;

    // auto now2 = std::chrono::steady_clock::now();
    // auto elapsed_sec = std::chrono::duration<double>(now2 - start_time).count();

    // throttler_->wait_for_slot();

    if (ep.is_in()) {
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
            seqnum, static_cast<std::uint16_t>(UrbStatusType::StatusOK), total_sent, current_start_frame,
            static_cast<std::uint16_t>(iso_descs.size()), std::move(transfer)));
    } else {
        // auto received_size = static_cast<std::uint32_t>(trx->data.size());
        // session->submit_ret_submit(
        //         UsbIpResponse::UsbIpRetSubmit::create_ret_submit_ok_without_data(seqnum, received_size));
    }
}

void UacAudioStreamingHandler::request_set_interface(std::uint16_t alternate_setting, std::uint32_t *p_status) {
    if (alternate_setting == 0) {
        streaming_ = false;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            while (!response_queue.empty()) {
                response_queue.pop();
            }
        }
        spdlog::info("UAC: Set interface to alternate setting 0, stopping streaming.");
        *p_status = 0;
    } else if (alternate_setting == 1) {
        streaming_ = true;
        spdlog::info("UAC: Set interface to alternate setting 1, starting streaming.");
        *p_status = 0;
    } else {
        *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
    }
}

std::uint8_t UacAudioStreamingHandler::request_get_interface(std::uint32_t *p_status) {
    return streaming_ ? 1 : 0;
}

void UacAudioStreamingHandler::on_disconnection(std::error_code &ec) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        while (!response_queue.empty()) {
            response_queue.pop();
        }
        // thread_running_ = false;
    }
    queue_cv.notify_all();
    streaming_ = false;
    VirtualInterfaceHandler::on_disconnection(ec);
}

void UacAudioStreamingHandler::handle_unlink_seqnum(std::uint32_t unlink_seqnum,
                                                    std::uint32_t cmd_seqnum) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        spdlog::info("Handling unlink for seqnum: {}, cmd_seqnum: {}", unlink_seqnum, cmd_seqnum);
        spdlog::info("Current response queue size: {}", response_queue.size());
        unlinked_seqnums_.push_back(unlink_seqnum);
    }

    session->submit_ret_unlink(
        UsbIpResponse::UsbIpRetUnlink::create_ret_unlink_success(cmd_seqnum));
}


// ==================== UacDeviceHelper ====================

void UacDeviceHelper::setup(std::shared_ptr<UsbDevice> device, StringPool &string_pool,
                            UacRole role, std::shared_ptr<AudioSource> source, std::shared_ptr<AudioSink> sink, std::uint8_t start_interface_index) {
    
    auto ac = std::make_shared<UacAudioControlHandler>(device->interfaces[0], string_pool, role);
    device->interfaces[start_interface_index++].handler = ac;

    std::uint8_t current_if_index = start_interface_index;

    if ((static_cast<std::uint8_t>(role) & static_cast<std::uint8_t>(UacRole::Microphone))) {
        if (current_if_index >= device->interfaces.size()) {
            throw std::runtime_error("Not enough interfaces allocated for Microphone");
        }
        auto as_mic = std::make_shared<UacAudioStreamingHandler>(
            device->interfaces[current_if_index], string_pool, true, source, nullptr);
        device->interfaces[current_if_index].handler = as_mic;
        as_mic->sync_string_interface_from(*ac);
        current_if_index++;
        ac->set_asm_handler(as_mic.get());
    }

    if ((static_cast<std::uint8_t>(role) & static_cast<std::uint8_t>(UacRole::Speaker))) {
        if (current_if_index >= device->interfaces.size()) {
            throw std::runtime_error("Not enough interfaces allocated for Speaker");
        }
        auto as_spk = std::make_shared<UacAudioStreamingHandler>(
            device->interfaces[current_if_index], string_pool, false, nullptr, sink);
        device->interfaces[current_if_index].handler = as_spk;
        as_spk->sync_string_interface_from(*ac);
        current_if_index++;
        ac->set_ass_handler(as_spk.get());
    }

    // auto dh = device->handler ? std::dynamic_pointer_cast<VirtualDeviceHandler>(device->handler)
    //                           : device->with_handler<SimpleVirtualDeviceHandler>(string_pool);
    // dh->setup_interface_handlers();
}

} // namespace usbipdcpp