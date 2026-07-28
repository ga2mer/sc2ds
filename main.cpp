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

    // Ограничиваем результат диапазоном int16_t и округляем
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
    0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7};

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
  MockToneAudioSource(double frequency_hz = 440.0,
                      std::uint32_t sample_rate = 48000)
      : frequency_(frequency_hz), sample_rate_(sample_rate), phase_(0.0) {}

  std::size_t read_audio(std::uint8_t *buffer, std::size_t max_bytes) override {
    std::size_t num_samples = max_bytes / sizeof(std::int16_t);
    auto *samples = reinterpret_cast<std::int16_t *>(buffer);

    double phase_increment =
        2.0 * 3.14159265358979323846 * frequency_ / sample_rate_;

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
  const size_t TARGET_CHUNK_BYTES =
      31 * 2 *
      sizeof(uint8_t); // 31 max length * haptic channels * bytes_per_sample

  MockAudioSink() {
    filter_rl.init(250.0, sample_rate());
    filter_rr.init(250.0, sample_rate());
    filter_fr_high.init(2000.0, sample_rate());
  }
  void write_audio(const std::uint8_t *buffer, std::size_t bytes) override {
    uint32_t num_frames = bytes / (channels() * sizeof(int16_t));
    for (uint32_t f = 0; f < num_frames; f++) {
      int idx = f * channels();

      int16_t fr_high = filter_fr_high.process(
          reinterpret_cast<const int16_t *>(buffer)[idx + 1]);
      int16_t rl_filtered =
          filter_rl.process(reinterpret_cast<const int16_t *>(buffer)[idx + 2]);
      int16_t rr_filtered =
          filter_rr.process(reinterpret_cast<const int16_t *>(buffer)[idx + 3]);

      int32_t rl_mixed =
          static_cast<int32_t>(rl_filtered) + static_cast<int32_t>(fr_high);
      int32_t rr_mixed =
          static_cast<int32_t>(rr_filtered) + static_cast<int32_t>(fr_high);

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
      if (f % 12 == 0) {
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
    return USB_AUDIO_CHANNEL_L | USB_AUDIO_CHANNEL_R | USB_AUDIO_CHANNEL_LS |
           USB_AUDIO_CHANNEL_RS;
  }
};

class SteamController {
private:
  libusb_context *usb_ctx = nullptr;
  libusb_device_handle *usb_handle = nullptr;
  std::vector<std::uint8_t> usb_buf;
  TritonMTUNoQuat_t last_state;
  bool is_puck = true;
  uint32_t main_interface = -1;
  uint32_t in_endpoint = -1;
  uint32_t out_endpoint = -1;

public:
  SteamController() : usb_buf(64) {}
  bool hijack_controller() {
    if (libusb_init(&usb_ctx) < 0) {
      spdlog::error("libusb_init failed");
      return false;
    }
    usb_handle = libusb_open_device_with_vid_pid(usb_ctx, 0x28de, 0x1304);
    if (!usb_handle) {
      spdlog::warn(
          "Failed to open Steam Controller connected to Puck, trying USB");
      usb_handle = libusb_open_device_with_vid_pid(usb_ctx, 0x28de, 0x1302);
      is_puck = false;
    }
    if (!usb_handle) {
      spdlog::error("Failed to open Steam Controller connected to USB");
      return false;
    }
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
    int num_interfaces =
        is_puck ? config->bNumInterfaces - 1 : config->bNumInterfaces;
    for (int i = 0; i < num_interfaces; ++i) {
      const struct libusb_interface &interface = config->interface[i];
      spdlog::debug("Checking interface {}, num_altsetting: {}", i,
                    interface.num_altsetting);
      for (int j = 0; j < interface.num_altsetting; ++j) {
        const struct libusb_interface_descriptor &altsetting =
            interface.altsetting[j];
        spdlog::debug(
            "  Altsetting {}: bInterfaceClass=0x{:02x}, bNumEndpoints={}", j,
            altsetting.bInterfaceClass, altsetting.bNumEndpoints);
        if (altsetting.bInterfaceClass == LIBUSB_CLASS_HID) {
          spdlog::debug("Found HID interface at index {}, altsetting {}", i, j);
          in_endpoint = -1;
          out_endpoint = -1;
          for (int k = 0; k < altsetting.bNumEndpoints; ++k) {
            const struct libusb_endpoint_descriptor &endpoint =
                altsetting.endpoint[k];
            spdlog::debug("    Endpoint {}: address=0x{:02x}, dir={}", k,
                          endpoint.bEndpointAddress,
                          (endpoint.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK)
                              ? "IN"
                              : "OUT");
            if ((endpoint.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) ==
                LIBUSB_ENDPOINT_IN) {
              in_endpoint = endpoint.bEndpointAddress;
            } else {
              out_endpoint = endpoint.bEndpointAddress;
            }
          }

          int transferred = 0;
          spdlog::debug("Testing HID interface {} with in_endpoint=0x{:02x}", i,
                        in_endpoint);
          if (libusb_kernel_driver_active(usb_handle, i) == 1) {
            if (libusb_detach_kernel_driver(usb_handle, i) < 0) {
              spdlog::warn("Failed to detach kernel driver from interface {}",
                           i);
              continue;
            }
          }
          if (libusb_claim_interface(usb_handle, i) < 0) {
            spdlog::warn("Failed to claim interface {}", i);
            continue;
          }
          int test_res =
              libusb_interrupt_transfer(usb_handle, in_endpoint, usb_buf.data(),
                                        usb_buf.size(), &transferred, 100);
          spdlog::debug("Transfer result: {}, transferred: {} bytes", test_res,
                        transferred);
          if (test_res == 0 && transferred > 0) {
            main_interface = i;
            spdlog::info("Found active HID interface {}: in_endpoint=0x{:02x}, "
                         "out_endpoint=0x{:02x}",
                         i, in_endpoint, out_endpoint);
            break;
          } else {
            spdlog::debug("Interface {} failed test: res={}, transferred={}", i,
                          test_res, transferred);
            libusb_release_interface(usb_handle, i);
            libusb_attach_kernel_driver(usb_handle, i);
          }
        }
      }
      if (main_interface != -1)
        break;
    }
    if (main_interface == -1) {
      spdlog::error("Failed to find active HID interface");
      return false;
    }
    return true;
  }
  TritonMTUNoQuat_t *get_controller_state() {
    int transferred;
    if (!usb_handle) {
      return nullptr;
    }
    int res = libusb_interrupt_transfer(usb_handle, in_endpoint, usb_buf.data(),
                                        usb_buf.size(), &transferred, 1000);
    if (res != 0 || transferred == 0) {
      spdlog::error("Failed to read controller state: {}", res);
      return nullptr;
    }
    if (usb_buf[0] != 0x42) {
      return nullptr;
    }
    std::memcpy(&last_state, usb_buf.data() + 1, sizeof(last_state));
    return &last_state;
  }
  bool enable_haptics() {
    std::vector<uint8_t> haptic_cmd = {0x86, 0x02, 0x02, 0x09};
    int transferred;
    int res =
        libusb_interrupt_transfer(usb_handle, out_endpoint, haptic_cmd.data(),
                                  haptic_cmd.size(), &transferred, 100);
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
    MsgHapticPCMStereo msg{};
    msg.report_id = 0x88;
    msg.length = haptic_data.size() / 2;
    for (size_t i = 0; i < msg.length; ++i) {
      msg.left[i] = haptic_data[i * 2];
      msg.right[i] = haptic_data[i * 2 + 1];
    }
    int transferred;
    if (!usb_handle)
      return;
    int res = libusb_interrupt_transfer(usb_handle, out_endpoint,
                                        reinterpret_cast<uint8_t *>(&msg),
                                        sizeof(msg), &transferred, 1000);
    if (res != 0 || transferred != sizeof(msg)) {
      spdlog::error("Failed to send haptic data: {}", res);
    }
  }
  void send_rumble(uint8_t left, uint8_t right) {
    MsgHapticRumble msg{};
    msg.report_id = 0x80;
    msg.left.speed = (uint16_t)left * 257;
    msg.right.speed = (uint16_t)right * 257;
    int transferred;
    if (!usb_handle)
      return;
    int res = libusb_interrupt_transfer(usb_handle, out_endpoint,
                                        reinterpret_cast<uint8_t *>(&msg),
                                        sizeof(msg), &transferred, 1000);
    if (res != 0 || transferred != sizeof(msg)) {
      spdlog::error("Failed to send haptic data: {}", res);
    }
  }
  ~SteamController() {
    if (usb_handle) {
      if (is_puck) {
        for (int i = 2; i <= 5; ++i) {
          if (libusb_kernel_driver_active(usb_handle, i) == 0) {
            libusb_release_interface(usb_handle, i);
            libusb_attach_kernel_driver(usb_handle, i);
          }
        }
      } else {
        libusb_release_interface(usb_handle, 0);
        libusb_attach_kernel_driver(usb_handle, 0);
      }
      libusb_close(usb_handle);
    }
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
DualsenseHandler::USBGetStateData last_state{};
std::array<std::uint8_t, sizeof(DualsenseHandler::ReportIn)>
translate_to_dualsense_report(const TritonMTUNoQuat_t &triton_report) {
  DualsenseHandler::USBGetStateData state{};
  state.powerState = DualsenseHandler::PowerState::Complete;
  state.PowerPercent = 0x0a; // 100%
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
  state.ButtonPad = (triton_report.buttons & TRITON_LEFT_TOUCHPAD_CLICK) != 0 ||
                    (triton_report.buttons & TRITON_RIGHT_TOUCHPAD_CLICK) != 0;
  state.LeftStickX = (uint8_t)((triton_report.sLeftStickX + 32768) >> 8);
  state.LeftStickY = (uint8_t)((-(triton_report.sLeftStickY) + 32768) >> 8);
  state.RightStickX = (uint8_t)((triton_report.sRightStickX + 32768) >> 8);
  state.RightStickY = (uint8_t)((-(triton_report.sRightStickY) + 32768) >> 8);
  state.TriggerLeft = (uint8_t)((triton_report.sTriggerLeft * 2) >> 8);
  if (state.TriggerLeft >= 255)
    state.ButtonL2 = 1;
  state.TriggerRight = (uint8_t)((triton_report.sTriggerRight * 2) >> 8);
  if (state.TriggerRight >= 255)
    state.ButtonR2 = 1;

  state.AccelerometerX = triton_report.imu.sAccelX;
  state.AccelerometerY = triton_report.imu.sAccelY;
  state.AccelerometerZ = triton_report.imu.sAccelZ;
  state.AngularVelocityX = triton_report.imu.sGyroX;
  state.AngularVelocityY = triton_report.imu.sGyroY;
  state.AngularVelocityZ = triton_report.imu.sGyroZ;
  if (triton_report.buttons & TRITON_LEFT_TOUCHPAD_TOUCH) {
    if (g_dh && last_state.touchData.Finger[0].NotTouching == 1) {
      g_dh->increase_touch_index();
      state.touchData.Finger[0].Index = g_dh->get_touch_index();
    } else {
      state.touchData.Finger[0].Index = last_state.touchData.Finger[0].Index;
    }
    state.touchData.Finger[0].NotTouching = 0;
    double normX = (triton_report.sLeftPadX + 32768.0) / 65535.0;
    double normY = (triton_report.sLeftPadY + 32768.0) / 65535.0;

    int x = std::clamp(static_cast<int>(std::round(normX * 1920)), 0, 1920);
    int y =
        std::clamp(static_cast<int>(std::round((1.0 - normY) * 1070)), 0, 1070);
    state.touchData.Finger[0].FingerX = x;
    state.touchData.Finger[0].FingerY = y;
  } else {
    state.touchData.Finger[0].NotTouching = 1;
  }
  if (triton_report.buttons & TRITON_RIGHT_TOUCHPAD_TOUCH) {
    if (g_dh && last_state.touchData.Finger[1].NotTouching == 1) {
      g_dh->increase_touch_index();
      state.touchData.Finger[1].Index = g_dh->get_touch_index();
    } else {
      state.touchData.Finger[1].Index = last_state.touchData.Finger[1].Index;
    }
    state.touchData.Finger[1].NotTouching = 0;
    double normX = (triton_report.sRightPadX + 32768.0) / 65535.0;
    double normY = (triton_report.sRightPadY + 32768.0) / 65535.0;

    int x = std::clamp(static_cast<int>(std::round(normX * 1920)), 0, 1920);
    int y =
        std::clamp(static_cast<int>(std::round((1.0 - normY) * 1070)), 0, 1070);
    state.touchData.Finger[1].FingerX = x;
    state.touchData.Finger[1].FingerY = y;
  } else {
    state.touchData.Finger[1].NotTouching = 1;
  }
  last_state = state;
  DualsenseHandler::ReportIn report{};
  report.ReportID = 0x01;
  report.State = state;
  std::array<std::uint8_t, sizeof(DualsenseHandler::ReportIn)> out{};
  std::memcpy(out.data(), &report, sizeof(report));
  return out;
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
  g_sController = new SteamController();
  if (!g_sController->hijack_controller()) {
    spdlog::error("Failed to hijack Steam Controller");
    delete g_sController;
    return 1;
  }
  if (!g_sController->enable_haptics()) {
    spdlog::error("Failed to enable haptics on Steam Controller");
    delete g_sController;
    return 1;
  }
  auto audio_sink = std::make_shared<MockAudioSink>();
  // if (snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
  //   std::cerr << "Can't open audio device" << std::endl;
  //   return 1;
  // }

  // snd_pcm_set_params(handle,
  //                    SND_PCM_FORMAT_S16_LE,
  //                    SND_PCM_ACCESS_RW_INTERLEAVED,
  //                    audio_sink->channels(),
  //                    audio_sink->sample_rate(),
  //                    1,
  //                    5 * 1000);
  spdlog::set_level(spdlog::level::info);
  uint16_t port = 53240;
  auto busid = "1-1";

  StringPool string_pool;

  std::vector<UsbInterface> interfaces = {
      UsbInterface{
          .interface_class = static_cast<std::uint8_t>(ClassCode::Audio),
          .interface_subclass =
              static_cast<std::uint8_t>(AudioVideoSubClassCode::AVControl),
          .interface_protocol = 0x00,
          .endpoints = {{}},
      },
      UsbInterface{
          .interface_class = static_cast<std::uint8_t>(ClassCode::Audio),
          .interface_subclass =
              static_cast<std::uint8_t>(AudioVideoSubClassCode::AudioStreaming),
          .interface_protocol = 0x00,
          .endpoints =
              {{},
               {UsbEndpoint{
                   .address = 0x01, // OUT
                   .attributes =
                       static_cast<std::uint8_t>(
                           EndpointAttributes::Isochronous) |
                       static_cast<std::uint8_t>(IsoSyncType::Adaptive),
                   .max_packet_size =
                       audio_sink->calculate_expected_bytes_per_packet(),
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

  auto device_handler =
      device->with_handler<SimpleVirtualDeviceHandler>(string_pool);
  device_handler->change_string_manufacturer(L"Sony Interactive Entertainment");
  device_handler->change_string_product(L"DualSense Wireless Controller");

  g_dh = &dynamic_cast<DualsenseHandler &>(*device->interfaces[2].handler);
  // auto audio_source = std::make_shared<MockToneAudioSource>(440.0, 48000);
  // usbipdcpp::UacDeviceHelper::setup(
  //     device, string_pool, usbipdcpp::UacRole::Microphone, audio_source,
  //     nullptr, 1
  // );
  usbipdcpp::UacDeviceHelper::setup(
      device, string_pool, usbipdcpp::UacRole::Speaker, nullptr, audio_sink, 0);

  device_handler->setup_interface_handlers();
  server.add_device(std::move(device));

  asio::ip::tcp::endpoint endpoint{asio::ip::tcp::v4(), port};
  server.start(endpoint);
  started = true;

  SPDLOG_INFO("Gamepad started on port {}, busid {}", port, busid);
  SPDLOG_INFO("Connect with: usbip --tcp-port {} attach -r 127.0.0.1 -b {}",
              port, busid);
  SPDLOG_INFO("Press Enter to exit...");

  worker_thread = std::thread([&]() {
    int actual_length = 0;
    auto rumble_last_time = std::chrono::steady_clock::now();
    int last_left_rumble = 0;
    int last_right_rumble = 0;
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      auto now = std::chrono::steady_clock::now();
      if (!g_sController) {
        break;
      }
      auto state = g_sController->get_controller_state();
      if (state && g_dh && g_dh->is_client_connected()) {
        auto report_buffer = translate_to_dualsense_report(*state);
        g_dh->send_input_report(asio::buffer(report_buffer));
      }
      if (g_dh && g_dh->is_client_connected()) {
        auto left_rumble = g_dh->get_left_rumble();
        auto right_rumble = g_dh->get_right_rumble();
        auto time_since_last =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - rumble_last_time)
                .count();
        if ((left_rumble > 0 || right_rumble > 0) && time_since_last > 40) {
          g_sController->send_rumble(left_rumble, right_rumble);
          last_left_rumble = left_rumble;
          last_right_rumble = right_rumble;
          rumble_last_time = std::chrono::steady_clock::now();
        } else if (left_rumble == 0 && right_rumble == 0 &&
                   (last_left_rumble > 0 || last_right_rumble > 0)) {
          g_sController->send_rumble(0, 0);
          last_left_rumble = 0;
          last_right_rumble = 0;
          rumble_last_time = std::chrono::steady_clock::now();
        }
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
        // static std::ofstream audio_file("output.pcm",
        //                                 std::ios::binary | std::ios::app);
        // if (audio_file.is_open()) {
        //   audio_file.write(
        //       reinterpret_cast<const char *>(current_packet.data()),
        //       current_packet.size() * sizeof(uint8_t));
        // }
        //   snd_pcm_uframes_t frames =
        //       current_packet.size() / audio_sink->channels();
        //   snd_pcm_sframes_t frames_written =
        //       snd_pcm_writei(handle, current_packet.data(), frames);
        //   if (frames_written < 0) {
        //     frames_written = snd_pcm_recover(handle, frames_written, 0);
        //   }
      }
    }
  });

  std::cin.get();
  signal_handler(0);
  // snd_pcm_drain(handle);
  // snd_pcm_close(handle);
  return 0;
}