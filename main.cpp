#include "DualsenseHandler.h"
#include "UacVirtualInterfaceHandler.h"
#include "spdlog/spdlog.h"
#include "types.h"
#include "usbipdcpp/Server.h"
#include "usbipdcpp/virtual_device/SimpleVirtualDeviceHandler.h"
#include "usbipdcpp/virtual_device/devices/GamepadHandler.h"
// #include <alsa/asoundlib.h>
#include <array>
#include <asio.hpp>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <queue>
#include <spdlog/common.h>
#include <unordered_map>

size_t sample_rate = 48000;
// size_t channels = 4;
size_t bytes_per_sample = 2; // 16-bit
// snd_pcm_t *handle;

using namespace usbipdcpp;

struct LowpassFilter {
  double prev_out = 0.0;
  double alpha = 0.0;

  void init(double cutoff_hz, double sample_rate) {
    double rc = 1.0 / (2.0 * M_PI * cutoff_hz);
    double dt = 1.0 / sample_rate;
    alpha = dt / (rc + dt);
  }

  int16_t process(int16_t sample) {
    double current_in = static_cast<double>(sample);
    prev_out = prev_out + alpha * (current_in - prev_out);

    if (prev_out > 32767.0)
      prev_out = 32767.0;
    if (prev_out < -32768.0)
      prev_out = -32768.0;
    return static_cast<int16_t>(prev_out);
  }
};

struct HighpassFilter {
  double prev_in = 0.0;
  double prev_out = 0.0;
  double alpha = 0.0;

  void init(double cutoff_hz, double sample_rate) {
    double rc = 1.0 / (2.0 * 3.14159265359 * cutoff_hz);
    double dt = 1.0 / sample_rate;
    alpha = rc / (rc + dt);
  }

  int16_t process(int16_t sample) {
    double current_in = static_cast<double>(sample);
    prev_out = alpha * (prev_out + current_in - prev_in);
    prev_in = current_in;

    if (prev_out > 32767.0)
      prev_out = 32767.0;
    if (prev_out < -32768.0)
      prev_out = -32768.0;
    return static_cast<int16_t>(prev_out);
  }
};

const int cBias = 0x84;
const int cClip = 32635;

static const uint8_t MuLawCompressTable[256] = {
    0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7};

static uint8_t LinearToMuLawSample(int16_t sample) {
  int sign = (sample >> 8) & 0x80;
  if (sign != 0)
    sample = (int16_t)-sample;
  if (sample > cClip)
    sample = cClip;

  sample = (int16_t)(sample + cBias);

  int exponent = (int)MuLawCompressTable[(sample >> 7) & 0xFF];
  int mantissa = (sample >> (exponent + 3)) & 0x0F;
  int compressedByte = ~(sign | (exponent << 4) | mantissa);

  return (uint8_t)compressedByte;
}

class MockToneAudioSource : public usbipdcpp::AudioSource {
public:
  MockToneAudioSource(double frequency_hz = 440.0, std::uint32_t sample_rate = 48000)
      : frequency_(frequency_hz), sample_rate_(sample_rate), phase_(0.0) {}

  std::size_t read_audio(std::uint8_t *buffer, std::size_t max_bytes) override {
    std::size_t num_samples = max_bytes / sizeof(std::int16_t);
    auto *samples = reinterpret_cast<std::int16_t *>(buffer);

    double phase_increment = 2.0 * 3.14159265358979323846 * frequency_ / sample_rate_;

    const double amplitude = 16000.0;

    for (std::size_t i = 0; i < num_samples; ++i) {
      double sample_val = std::sin(phase_) * amplitude;
      samples[i] = static_cast<std::int16_t>(sample_val);

      phase_ += phase_increment;
      if (phase_ >= 2.0 * 3.14159265358979323846) {
        phase_ -= 2.0 * 3.14159265358979323846;
      }
    }

    return num_samples * sizeof(std::int16_t);
  }

  std::uint32_t sample_rate() const override { return sample_rate_; }

private:
  double frequency_;
  std::uint32_t sample_rate_;
  double phase_;
};

class MockAudioSink : public usbipdcpp::AudioSink {
public:
  std::queue<std::vector<uint8_t>> queue;
  std::mutex queue_mutex;
  std::vector<uint8_t> temp_accumulator;
  LowpassFilter filter_rl;
  LowpassFilter filter_rr;
  HighpassFilter filter_fr_high;
  const size_t TARGET_CHUNK_BYTES = 31 * 2 * sizeof(uint8_t); // 31 max length * haptic channels * bytes_per_sample

  MockAudioSink() {
    filter_rl.init(250.0, sample_rate());
    filter_rr.init(250.0, sample_rate());
    filter_fr_high.init(2000.0, sample_rate());
  }
  void write_audio(const std::uint8_t *buffer, std::size_t bytes) override {
    uint32_t num_frames = bytes / (channels() * sizeof(int16_t));
    for (uint32_t f = 0; f < num_frames; f++) {
      int idx = f * channels();
      int16_t fr_high = filter_fr_high.process(reinterpret_cast<const int16_t *>(buffer)[idx + 1]);
      int16_t rl_filtered = filter_rl.process(reinterpret_cast<const int16_t *>(buffer)[idx + 2]);
      int16_t rr_filtered = filter_rr.process(reinterpret_cast<const int16_t *>(buffer)[idx + 3]);

      int32_t rl_mixed = static_cast<int32_t>(rl_filtered) + static_cast<int32_t>(fr_high);
      int32_t rr_mixed = static_cast<int32_t>(rr_filtered) + static_cast<int32_t>(fr_high);

      if (rl_mixed > 32767)
        rl_mixed = 32767;
      if (rl_mixed < -32768)
        rl_mixed = -32768;
      if (rr_mixed > 32767)
        rr_mixed = 32767;
      if (rr_mixed < -32768)
        rr_mixed = -32768;

      int16_t rl_new = static_cast<int16_t>(rl_mixed);
      int16_t rr_new = static_cast<int16_t>(rr_mixed);
      if (f % 12 == 0) { // resampling, 48000/4000
        temp_accumulator.push_back(LinearToMuLawSample(rl_new));
        temp_accumulator.push_back(LinearToMuLawSample(rr_new));
        if (temp_accumulator.size() >= TARGET_CHUNK_BYTES) {
          std::lock_guard<std::mutex> lock(queue_mutex);
          queue.push(std::move(temp_accumulator));

          temp_accumulator.clear();
          temp_accumulator.reserve(TARGET_CHUNK_BYTES);
        }
      }
    }
  }
  std::uint8_t channels() const override { return 4; }

  std::uint16_t channels_config() override {
    return USB_AUDIO_CHANNEL_L | USB_AUDIO_CHANNEL_R | USB_AUDIO_CHANNEL_LS | USB_AUDIO_CHANNEL_RS;
  }
};

struct SCInterface {
  uint32_t interface_number;
  uint32_t in_endpoint;
  uint32_t out_endpoint;
};

libusb_context *usb_ctx = nullptr;
class SteamController {
private:
  libusb_device_handle *usb_handle = nullptr;
  std::vector<std::uint8_t> usb_buf;
  TritonMTUNoQuat_t controller_state{};
  TritonBatteryStatus_t battery_state{};
  bool is_puck = false;
  bool claiming_interfaces_ = false;
  bool search_working_interface_ = false;
  bool report_is_ready_ = false;
  bool battery_report_is_ready_ = false;
  bool haptic_skip_ = false;

  std::unordered_map<uint32_t, SCInterface> interfaces;

  uint32_t main_interface = -1;
  uint32_t in_endpoint = -1;
  uint32_t out_endpoint = -1;

  libusb_transfer *read_transfer = nullptr;
  unsigned char read_buffer[64];

public:
  SteamController() {}
  bool claiming_interfaces() const { return claiming_interfaces_; }
  bool search_working_interface() const { return search_working_interface_; }
  bool report_is_ready() const { return report_is_ready_; }
  bool battery_report_is_ready() const { return battery_report_is_ready_; }
  void set_report_ready(bool ready) { report_is_ready_ = ready; }
  TritonMTUNoQuat_t get_controller_state() { return controller_state; }
  TritonBatteryStatus_t get_battery_state() { return battery_state; }
  bool init() {
    int r = libusb_hotplug_register_callback(usb_ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
                                             LIBUSB_HOTPLUG_ENUMERATE, VALVE_USB_VID, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
                                             &SteamController::hotplug_callback_wrapper, this, NULL);
    if (r != LIBUSB_SUCCESS) {
      spdlog::error("Failed to register hotplug callback: {}", r);
      return false;
    }
    return true;
  }

  int handle_hotplug(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event) {
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev, &desc);
    printf("Hotplug event: %s for device %04x:%04x\n", (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) ? "ARRIVED" : "LEFT", desc.idVendor,
           desc.idProduct);
    if (desc.idVendor != VALVE_USB_VID) {
      return 0;
    }
    if (desc.idProduct == PUCK_PID || desc.idProduct == TRITON_PID) {
      if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        spdlog::info("Steam Controller connected: {:04x}:{:04x}", desc.idVendor, desc.idProduct);
        if (!open_handle(desc.idProduct)) {
          spdlog::error("Failed to open Steam Controller");
        }
      } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        clear();
      }
    }
    return 0;
  }

  bool open_handle(uint16_t pid) {
    is_puck = (pid == PUCK_PID);
    usb_handle = libusb_open_device_with_vid_pid(usb_ctx, VALVE_USB_VID, pid);
    if (!usb_handle) {
      spdlog::error("Failed to open Steam Controller connected to USB");
      return false;
    }
    int r = libusb_set_auto_detach_kernel_driver(usb_handle, 1);
    if (r != LIBUSB_SUCCESS) {
      spdlog::error("Failed to set auto detach kernel driver: {}", r);
      return false;
    }
    claiming_interfaces_ = true;
    return true;
  }
  bool claim_interface() {
    libusb_device *dev = libusb_get_device(usb_handle);
    if (!dev) {
      spdlog::error("Failed to get libusb device");
      return false;
    }
    struct libusb_config_descriptor *config;
    int r = libusb_get_active_config_descriptor(dev, &config);
    if (r < 0) {
      spdlog::error("Failed to get active config descriptor");
      return false;
    }
    int num_interfaces = is_puck ? config->bNumInterfaces - 1 : config->bNumInterfaces;
    for (uint32_t i = 0; i < num_interfaces; ++i) {
      const struct libusb_interface &interface = config->interface[i];
      if (interface.num_altsetting != 1) {
        continue;
      }
      const struct libusb_interface_descriptor &altsetting = interface.altsetting[0];
      if (altsetting.bInterfaceClass == LIBUSB_CLASS_HID) {
        in_endpoint = -1;
        out_endpoint = -1;
        for (int k = 0; k < altsetting.bNumEndpoints; ++k) {
          const struct libusb_endpoint_descriptor &endpoint = altsetting.endpoint[k];
          if ((endpoint.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN) {
            in_endpoint = endpoint.bEndpointAddress;
          } else {
            out_endpoint = endpoint.bEndpointAddress;
          }
        }
        int res;
        if ((res = libusb_claim_interface(usb_handle, i)) < 0) {
          spdlog::warn("Failed to claim interface {}, error {}", i, res);
          libusb_free_config_descriptor(config);
          return false;
        }
        interfaces[i] = {i, in_endpoint, out_endpoint};
      }
    }
    claiming_interfaces_ = false;
    search_working_interface_ = true;
    libusb_free_config_descriptor(config);
    return true;
  }
  bool polling_active_interfaces() {
    if (!is_puck) {
      for (const auto &[interface_number, sc_interface] : interfaces) {
        main_interface = interface_number;
        in_endpoint = sc_interface.in_endpoint;
        out_endpoint = sc_interface.out_endpoint;
        search_working_interface_ = false;
        start_read();
        if (!enable_haptics()) {
          spdlog::error("Failed to enable haptics on Steam Controller");
        }
        return true;
      }
    }
    std::vector<uint8_t> setReport(64, 0);
    setReport[0] = 0x02;
    setReport[1] = 0xb4;
    std::vector<uint8_t> getReport(64, 0);
    uint16_t wValue = static_cast<uint16_t>((3u << 8) | 0x02);
    for (const auto &[interface_number, sc_interface] : interfaces) {
      int res =
          libusb_control_transfer(usb_handle,
                                  static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_CLASS) |
                                      static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                                  0x09, // SET_REPORT
                                  wValue, sc_interface.interface_number, setReport.data(), static_cast<uint16_t>(setReport.size()), 1000);
      if (res == static_cast<int>(setReport.size())) {
        res =
            libusb_control_transfer(usb_handle,
                                    static_cast<uint8_t>(LIBUSB_ENDPOINT_IN) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_CLASS) |
                                        static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                                    0x01, // GET_REPORT
                                    wValue, sc_interface.interface_number, getReport.data(), static_cast<uint16_t>(getReport.size()), 1000);
        // TODO: why it's 63 and not 64
        if (res == 63) {
          if (getReport[0] == 0x02 && getReport[1] == 0xb4 && getReport[3] == 0x02) {
            spdlog::info("Interface {} is active and working", interface_number);
            main_interface = interface_number;
            in_endpoint = sc_interface.in_endpoint;
            out_endpoint = sc_interface.out_endpoint;
            search_working_interface_ = false;
            start_read();
            if (!enable_haptics()) {
              spdlog::error("Failed to enable haptics on Steam Controller");
            }
            return true;
          }
        } else {
          spdlog::warn("Interface {} failed GET_REPORT: res={}", interface_number, res);
        }
      } else {
        spdlog::warn("Interface {} failed SET_REPORT: res={}", interface_number, res);
      }
    }
    return false;
  }
  void start_read() {
    if (read_transfer) {
      libusb_free_transfer(read_transfer);
    }

    read_transfer = libusb_alloc_transfer(0);

    libusb_fill_interrupt_transfer(read_transfer, usb_handle, in_endpoint, read_buffer, sizeof(read_buffer),
                                   &SteamController::read_callback_wrapper, this, 5000);

    int r = libusb_submit_transfer(read_transfer);
    if (r < 0) {
      spdlog::error("Failed to submit read transfer: {}", r);
      libusb_free_transfer(read_transfer);
      read_transfer = nullptr;
    }
  }
  void read_callback(libusb_transfer *transfer) {
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
      if (transfer->actual_length > 0) {
        if (transfer->buffer[0] == ID_TRITON_CONTROLLER_STATE) {
          std::memcpy(&controller_state, transfer->buffer + 1, sizeof(controller_state));
          report_is_ready_ = true;
        } else if (transfer->buffer[0] == ID_TRITON_BATTERY_STATUS) {
          std::memcpy(&battery_state, transfer->buffer + 1, sizeof(battery_state));
          battery_report_is_ready_ = true;
        } else if (transfer->buffer[0] == ID_TRITON_HAPTIC_STREAM_STATUS_STATE) {
          if (transfer->buffer[2] & STREAM_STATUS_HAS_ENOUGH_DATA) {
            haptic_skip_ = true;
          }
          if (transfer->buffer[2] & STREAM_STATUS_NEEDS_MORE_DATA) {
            haptic_skip_ = false;
          }
        }
      }
    } else {
      spdlog::error("Read transfer failed with status");
    }

    libusb_submit_transfer(transfer);
  }
  bool enable_haptics() {
    std::vector<uint8_t> haptic_cmd = {0x86, 0x02, 0x02, 0x09};
    int transferred;
    int res = libusb_interrupt_transfer(usb_handle, out_endpoint, haptic_cmd.data(), haptic_cmd.size(), &transferred, 100);
    if (res != 0 || transferred != haptic_cmd.size()) {
      spdlog::error("Failed to enable haptics: {}", res);
      return false;
    }
    return true;
  }
  void send_audio_haptic(const std::vector<uint8_t> &haptic_data) {
    if (haptic_data.size() > 62) {
      spdlog::error("Haptic data too large: {}", haptic_data.size());
      return;
    }
    if (haptic_skip_)
      return;
    MsgHapticPCMStereo msg{};
    msg.report_id = 0x88;
    msg.length = haptic_data.size() / 2;
    for (size_t i = 0; i < msg.length; ++i) {
      msg.left[i] = haptic_data[i * 2];
      msg.right[i] = haptic_data[i * 2 + 1];
    }
    send_data(reinterpret_cast<unsigned char *>(&msg), sizeof(msg));
  }
  void send_rumble(uint8_t left, uint8_t right) {
    MsgHapticRumble msg{};
    msg.report_id = 0x80;
    msg.left.speed = (uint16_t)left * 257;
    msg.right.speed = (uint16_t)right * 257;
    send_data(reinterpret_cast<unsigned char *>(&msg), sizeof(msg));
  }
  void send_data(unsigned char *data, int length) {
    if (!usb_handle)
      return;
    struct libusb_transfer *transfer = libusb_alloc_transfer(0);
    unsigned char *buffer = (unsigned char *)malloc(length);
    memcpy(buffer, data, length);
    libusb_fill_interrupt_transfer(transfer, usb_handle, out_endpoint, buffer, length, &SteamController::write_callback_wrapper, this,
                                   1000);

    int r = libusb_submit_transfer(transfer);
    if (r < 0) {
      spdlog::error("Failed to submit write transfer: {}", r);
      free(buffer);
      libusb_free_transfer(transfer);
    }
  }
  static int LIBUSB_CALL hotplug_callback_wrapper(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event, void *user_data) {
    auto *instance = static_cast<SteamController *>(user_data);
    return instance->handle_hotplug(ctx, dev, event);
  }
  void write_callback(libusb_transfer *transfer) {
    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
      spdlog::error("Write transfer failed with status: {}", static_cast<int>(transfer->status));
    }
    free(transfer->buffer);
    libusb_free_transfer(transfer);
  }
  static void read_callback_wrapper(libusb_transfer *transfer) {
    SteamController *instance = static_cast<SteamController *>(transfer->user_data);
    instance->read_callback(transfer);
  }
  static void write_callback_wrapper(libusb_transfer *transfer) {
    SteamController *instance = static_cast<SteamController *>(transfer->user_data);
    instance->write_callback(transfer);
  }
  void stop() {
    if (read_transfer)
      libusb_cancel_transfer(read_transfer);
  }
  void clear() {
    stop();
    if (usb_handle) {
      for (const auto &[interface_number, sc_interface] : interfaces) {
        libusb_release_interface(usb_handle, interface_number);
      }
      libusb_close(usb_handle);
    }
  }
  ~SteamController() {
    clear();
    if (usb_ctx) {
      libusb_exit(usb_ctx);
    }
  }
};
static Server server;
static SteamController *g_sController = nullptr;
std::atomic<bool> running{true};
std::atomic<bool> started{false};
std::thread worker_thread;
DualsenseHandler *g_dh = nullptr;

void signal_handler(int signal) {
  running = false;
  if (worker_thread.joinable()) {
    worker_thread.join();
  }
  if (g_sController) {
    delete g_sController;
    g_sController = nullptr;
  }
  if (started) {
    server.stop();
  }
  exit(signal);
}

DualsenseHandler::ReportIn report{0x1, {}};
std::atomic<bool> report_is_ready{false};
void translate_to_dualsense_report() {
  auto &state = report.State;
  const auto &triton_report = g_sController->get_controller_state();
  const auto &battery_report = g_sController->get_battery_state();
  if (g_sController->battery_report_is_ready()) {
    state.powerState = TritonToDualsensePowerState(static_cast<EChargeState>(g_sController->get_battery_state().ucChargeState));
    state.PowerPercent = RoundPowerLevel(g_sController->get_battery_state().ucBatteryLevel);
  } else {
    state.powerState = DualsenseHandler::PowerState::Charging;
    state.PowerPercent = 0xa;
  }
  state.SeqNo = triton_report.seq_num;
  if (triton_report.buttons & TRITON_LBUTTON_DPAD_UP) {
    if (triton_report.buttons & TRITON_LBUTTON_DPAD_RIGHT)
      state.DPad = DualsenseHandler::Direction::NorthEast;
    else if (triton_report.buttons & TRITON_LBUTTON_DPAD_LEFT)
      state.DPad = DualsenseHandler::Direction::NorthWest;
    else
      state.DPad = DualsenseHandler::Direction::North;
  } else if (triton_report.buttons & TRITON_LBUTTON_DPAD_DOWN) {
    if (triton_report.buttons & TRITON_LBUTTON_DPAD_RIGHT)
      state.DPad = DualsenseHandler::Direction::SouthEast;
    else if (triton_report.buttons & TRITON_LBUTTON_DPAD_LEFT)
      state.DPad = DualsenseHandler::Direction::SouthWest;
    else
      state.DPad = DualsenseHandler::Direction::South;
  } else if (triton_report.buttons & TRITON_LBUTTON_DPAD_RIGHT) {
    state.DPad = DualsenseHandler::Direction::East;
  } else if (triton_report.buttons & TRITON_LBUTTON_DPAD_LEFT) {
    state.DPad = DualsenseHandler::Direction::West;
  } else {
    state.DPad = DualsenseHandler::Direction::None;
  }

  state.ButtonCross = (triton_report.buttons & TRITON_LBUTTON_A) != 0;
  state.ButtonCircle = (triton_report.buttons & TRITON_LBUTTON_B) != 0;
  state.ButtonSquare = (triton_report.buttons & TRITON_LBUTTON_X) != 0;
  state.ButtonTriangle = (triton_report.buttons & TRITON_LBUTTON_Y) != 0;

  state.ButtonL1 = (triton_report.buttons & TRITON_LBUTTON_L) != 0;
  state.ButtonR1 = (triton_report.buttons & TRITON_LBUTTON_R) != 0;
  state.ButtonL3 = (triton_report.buttons & TRITON_LBUTTON_L3) != 0;
  state.ButtonR3 = (triton_report.buttons & TRITON_LBUTTON_R3) != 0;
  state.ButtonOptions = (triton_report.buttons & TRITON_LBUTTON_VIEW) != 0;
  state.ButtonCreate = (triton_report.buttons & TRITON_LBUTTON_MENU) != 0;
  state.ButtonLeftFunction = (triton_report.buttons & TRITON_HBUTTON_L4) != 0;
  state.ButtonRightFunction = (triton_report.buttons & TRITON_HBUTTON_R4) != 0;
  state.ButtonHome = (triton_report.buttons & TRITON_LBUTTON_STEAM) != 0;
  state.ButtonPad = (triton_report.buttons & TRITON_LEFT_TOUCHPAD_CLICK) != 0 || (triton_report.buttons & TRITON_RIGHT_TOUCHPAD_CLICK) != 0;
  state.LeftStickX = (uint8_t)((triton_report.sLeftStickX + 32768) >> 8);
  state.LeftStickY = (uint8_t)((-(triton_report.sLeftStickY) + 32768) >> 8);
  state.RightStickX = (uint8_t)((triton_report.sRightStickX + 32768) >> 8);
  state.RightStickY = (uint8_t)((-(triton_report.sRightStickY) + 32768) >> 8);
  state.TriggerLeft = (uint8_t)((triton_report.sTriggerLeft * 2) >> 8);
  state.ButtonL2 = state.TriggerLeft >= 255;
  state.TriggerRight = (uint8_t)((triton_report.sTriggerRight * 2) >> 8);
  state.ButtonR2 = state.TriggerRight >= 255;

  state.AccelerometerX = triton_report.imu.sAccelX;
  state.AccelerometerY = -triton_report.imu.sAccelY;
  state.AccelerometerZ = triton_report.imu.sAccelZ;
  state.AngularVelocityX = triton_report.imu.sGyroX;
  state.AngularVelocityY = -triton_report.imu.sGyroY;
  state.AngularVelocityZ = triton_report.imu.sGyroZ;
  state.SensorTimestamp = triton_report.imu.timestamp * 3;
  // TODO: change timestamp only when data has changed
  if (triton_report.buttons & TRITON_LEFT_TOUCHPAD_TOUCH) {
    if (g_dh && state.touchData.Finger[0].NotTouching == 1) {
      g_dh->increase_touch_index();
      state.touchData.Finger[0].Index = g_dh->get_touch_index();
    }
    state.touchData.Finger[0].NotTouching = 0;
    double normX = (triton_report.sLeftPadX + 32768.0) / 65535.0;
    double normY = (triton_report.sLeftPadY + 32768.0) / 65535.0;

    int x = std::clamp(static_cast<int>(std::round(normX * 1920)), 0, 1920);
    int y = std::clamp(static_cast<int>(std::round((1.0 - normY) * 1070)), 0, 1070);
    state.touchData.Finger[0].FingerX = x;
    state.touchData.Finger[0].FingerY = y;
  } else {
    state.touchData.Finger[0].NotTouching = 1;
  }
  if (triton_report.buttons & TRITON_RIGHT_TOUCHPAD_TOUCH) {
    if (g_dh && state.touchData.Finger[1].NotTouching == 1) {
      g_dh->increase_touch_index();
      state.touchData.Finger[1].Index = g_dh->get_touch_index();
    }
    state.touchData.Finger[1].NotTouching = 0;
    double normX = (triton_report.sRightPadX + 32768.0) / 65535.0;
    double normY = (triton_report.sRightPadY + 32768.0) / 65535.0;

    int x = std::clamp(static_cast<int>(std::round(normX * 1920)), 0, 1920);
    int y = std::clamp(static_cast<int>(std::round((1.0 - normY) * 1070)), 0, 1070);
    state.touchData.Finger[1].FingerX = x;
    state.touchData.Finger[1].FingerY = y;
  } else {
    state.touchData.Finger[1].NotTouching = 1;
  }
}

bool is_module_loaded(const std::string &module_name) {
  std::string path = "/sys/module/" + module_name;
  return std::filesystem::exists(path);
}

int main() {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  if (!is_module_loaded("vhci_hcd")) {
    spdlog::warn("vhci_hcd module may not be loaded. Load it with: sudo "
                 "modprobe vhci_hcd");
  }
  if (libusb_init(&usb_ctx) < 0) {
    spdlog::error("libusb_init failed");
    return 1;
  }
  if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
    spdlog::error("This platform does not support libusb hotplug.");
    return 1;
  }
  g_sController = new SteamController();
  if (!g_sController->init()) {
    spdlog::error("Failed to init Steam Controller");
    delete g_sController;
    return 1;
  }
  auto audio_sink = std::make_shared<MockAudioSink>();
  spdlog::set_level(spdlog::level::info);
  uint16_t port = 53240;
  auto busid = "1-1";

  StringPool string_pool;

  std::vector<UsbInterface> interfaces = {
      UsbInterface{
          .interface_class = static_cast<std::uint8_t>(ClassCode::Audio),
          .interface_subclass = static_cast<std::uint8_t>(AudioVideoSubClassCode::AVControl),
          .interface_protocol = 0x00,
          .endpoints = {{}},
      },
      UsbInterface{
          .interface_class = static_cast<std::uint8_t>(ClassCode::Audio),
          .interface_subclass = static_cast<std::uint8_t>(AudioVideoSubClassCode::AudioStreaming),
          .interface_protocol = 0x00,
          .endpoints = {{},
                        {UsbEndpoint{
                            .address = 0x01, // OUT
                            .attributes = static_cast<std::uint8_t>(EndpointAttributes::Isochronous) |
                                          static_cast<std::uint8_t>(IsoSyncType::Adaptive),
                            .max_packet_size = audio_sink->calculate_expected_bytes_per_packet(),
                            .interval = 4,
                        }}},
      },
      UsbInterface{
          .interface_class = static_cast<std::uint8_t>(ClassCode::HID),
          .interface_subclass = 0x00,
          .interface_protocol = 0x00,
          .endpoints = {{
              UsbEndpoint{
                  .address = 0x84,    // IN
                  .attributes = 0x03, // Interrupt
                  .max_packet_size = 64,
                  .interval = 6,
              },
              UsbEndpoint{
                  .address = 0x03,    // OUT
                  .attributes = 0x03, // Interrupt
                  .max_packet_size = 64,
                  .interval = 6,
              },
          }},
      },
  };
  interfaces[2].with_handler<DualsenseHandler>(string_pool);

  auto device = std::make_shared<UsbDevice>(UsbDevice{
      .path = "/usbipdcpp/dualsense",
      .busid = busid,
      .bus_num = 1,
      .dev_num = 1,
      .speed = static_cast<std::uint32_t>(UsbSpeed::High),
      .vendor_id = 0x054c,
      .product_id = 0x0ce6,
      .device_bcd = 0x0100,
      .device_class = 0x00,
      .device_subclass = 0x00,
      .device_protocol = 0x00,
      .configuration_value = 1,
      .num_configurations = 1,
      .interfaces = interfaces,
      .ep0_in = UsbEndpoint::get_ep0_in(UsbSpeed::High),
      .ep0_out = UsbEndpoint::get_ep0_out(UsbSpeed::High),
  });

  auto device_handler = device->with_handler<SimpleVirtualDeviceHandler>(string_pool);
  device_handler->change_string_manufacturer(L"Sony Interactive Entertainment");
  device_handler->change_string_product(L"DualSense Wireless Controller");

  g_dh = &dynamic_cast<DualsenseHandler &>(*device->interfaces[2].handler);
  // auto audio_source = std::make_shared<MockToneAudioSource>(440.0, 48000);
  // usbipdcpp::UacDeviceHelper::setup(
  //     device, string_pool, usbipdcpp::UacRole::Microphone, audio_source,
  //     nullptr, 1
  // );
  usbipdcpp::UacDeviceHelper::setup(device, string_pool, usbipdcpp::UacRole::Speaker, nullptr, audio_sink, 0);

  device_handler->setup_interface_handlers();
  server.add_device(std::move(device));

  asio::ip::tcp::endpoint endpoint{asio::ip::tcp::v4(), port};
  server.start(endpoint);
  started = true;

  SPDLOG_INFO("Gamepad started on port {}, busid {}", port, busid);
  SPDLOG_INFO("Connect with: usbip --tcp-port {} attach -r 127.0.0.1 -b {}", port, busid);
  SPDLOG_INFO("Press Enter to exit...");

  worker_thread = std::thread([&]() {
    int actual_length = 0;
    auto rumble_last_time = std::chrono::steady_clock::now();
    int last_left_rumble = 0;
    int last_right_rumble = 0;
    struct timeval tv = {0, 1000}; // 1ms timeout
    while (running) {
      auto now = std::chrono::steady_clock::now();
      libusb_handle_events_timeout_completed(usb_ctx, &tv, NULL);
      if (g_dh && g_dh->is_client_connected()) {
        if (g_sController->report_is_ready()) {
          translate_to_dualsense_report();
          g_dh->send_input_report(asio::buffer(&report, sizeof(report)));
          g_sController->set_report_ready(false);
        }
        auto left_rumble = g_dh->get_left_rumble();
        auto right_rumble = g_dh->get_right_rumble();
        auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(now - rumble_last_time).count();
        if ((left_rumble > 0 || right_rumble > 0) && time_since_last > 40) {
          g_sController->send_rumble(left_rumble, right_rumble);
          last_left_rumble = left_rumble;
          last_right_rumble = right_rumble;
          rumble_last_time = std::chrono::steady_clock::now();
        } else if (left_rumble == 0 && right_rumble == 0 && (last_left_rumble > 0 || last_right_rumble > 0)) {
          g_sController->send_rumble(0, 0);
          last_left_rumble = 0;
          last_right_rumble = 0;
          rumble_last_time = std::chrono::steady_clock::now();
        }
        std::vector<uint8_t> current_packet;
        {
          std::lock_guard<std::mutex> lock(audio_sink->queue_mutex);
          if (!audio_sink->queue.empty()) {
            current_packet = std::move(audio_sink->queue.front());
            audio_sink->queue.pop();
          }
        }
        if (!current_packet.empty()) {
          g_sController->send_audio_haptic(current_packet);
        }
      }
      if (g_sController->claiming_interfaces()) {
        if (!g_sController->claim_interface()) {
          spdlog::error("Failed to claim Steam Controller interface, close all other "
                        "applications that may be using, try again in 1 second");
          std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
          spdlog::info("Successfully claimed Steam Controller interface");
        }
      } else if (g_sController->search_working_interface()) {
        if (!g_sController->polling_active_interfaces()) {
          spdlog::error("Failed to find active Steam Controller interface");
          std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
          spdlog::info("Successfully found active Steam Controller interface");
        }
      }
    }
  });

  std::cin.get();
  signal_handler(0);
  return 0;
}