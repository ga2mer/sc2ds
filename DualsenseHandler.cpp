#include "DualsenseHandler.h"

#include <chrono>

#include <asio.hpp>
#include <spdlog/spdlog.h>

namespace usbipdcpp {

DualsenseHandler::DualsenseHandler(UsbInterface &handle_interface,
                                   StringPool &string_pool)
    : HidVirtualInterfaceHandler(handle_interface, string_pool) {
  report_descriptor_ = {
      0x05, 0x01,       // Usage Page (Generic Desktop Ctrls)
      0x09, 0x05,       // Usage (Game Pad)
      0xA1, 0x01,       // Collection (Application)
      0x85, 0x01,       //   Report ID (1)
      0x09, 0x30,       //   Usage (X)
      0x09, 0x31,       //   Usage (Y)
      0x09, 0x32,       //   Usage (Z)
      0x09, 0x35,       //   Usage (Rz)
      0x09, 0x33,       //   Usage (Rx)
      0x09, 0x34,       //   Usage (Ry)
      0x15, 0x00,       //   Logical Minimum (0)
      0x26, 0xFF, 0x00, //   Logical Maximum (255)
      0x75, 0x08,       //   Report Size (8)
      0x95, 0x06,       //   Report Count (6)
      0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position)
      0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
      0x09, 0x20,       //   Usage (0x20)
      0x95, 0x01,       //   Report Count (1)
      0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position)
      0x05, 0x01, //   Usage Page (Generic Desktop Ctrls)
      0x09, 0x39, //   Usage (Hat switch)
      0x15, 0x00, //   Logical Minimum (0)
      0x25, 0x07, //   Logical Maximum (7)
      0x35, 0x00, //   Physical Minimum (0)
      0x46, 0x3B, 0x01, //   Physical Maximum (315)
      0x65, 0x14,       //   Unit (System: English Rotation, Length: Centimeter)
      0x75, 0x04,       //   Report Size (4)
      0x95, 0x01,       //   Report Count (1)
      0x81, 0x42, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null
                  //   State)
      0x65, 0x00, //   Unit (None)
      0x05, 0x09, //   Usage Page (Button)
      0x19, 0x01, //   Usage Minimum (0x01)
      0x29, 0x0F, //   Usage Maximum (0x0F)
      0x15, 0x00, //   Logical Minimum (0)
      0x25, 0x01, //   Logical Maximum (1)
      0x75, 0x01, //   Report Size (1)
      0x95, 0x0F, //   Report Count (15)
      0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position)
      0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
      0x09, 0x21,       //   Usage (0x21)
      0x95, 0x0D,       //   Report Count (13)
      0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position)
      0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
      0x09, 0x22,       //   Usage (0x22)
      0x15, 0x00,       //   Logical Minimum (0)
      0x26, 0xFF, 0x00, //   Logical Maximum (255)
      0x75, 0x08,       //   Report Size (8)
      0x95, 0x34,       //   Report Count (52)
      0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position)
      0x85, 0x02, //   Report ID (2)
      0x09, 0x23, //   Usage (0x23)
      0x95, 0x2F, //   Report Count (47)
      0x91, 0x02, //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x05, //   Report ID (5)
      0x09, 0x33, //   Usage (0x33)
      0x95, 0x28, //   Report Count (40)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x08, //   Report ID (8)
      0x09, 0x34, //   Usage (0x34)
      0x95, 0x2F, //   Report Count (47)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x09, //   Report ID (9)
      0x09, 0x24, //   Usage (0x24)
      0x95, 0x13, //   Report Count (19)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x0A, //   Report ID (10)
      0x09, 0x25, //   Usage (0x25)
      0x95, 0x1A, //   Report Count (26)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x20, //   Report ID (32)
      0x09, 0x26, //   Usage (0x26)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x21, //   Report ID (33)
      0x09, 0x27, //   Usage (0x27)
      0x95, 0x04, //   Report Count (4)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x22, //   Report ID (34)
      0x09, 0x40, //   Usage (0x40)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x80, //   Report ID (-128)
      0x09, 0x28, //   Usage (0x28)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x81, //   Report ID (-127)
      0x09, 0x29, //   Usage (0x29)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x82, //   Report ID (-126)
      0x09, 0x2A, //   Usage (0x2A)
      0x95, 0x09, //   Report Count (9)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x83, //   Report ID (-125)
      0x09, 0x2B, //   Usage (0x2B)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x84, //   Report ID (-124)
      0x09, 0x2C, //   Usage (0x2C)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0x85, //   Report ID (-123)
      0x09, 0x2D, //   Usage (0x2D)
      0x95, 0x02, //   Report Count (2)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0xA0, //   Report ID (-96)
      0x09, 0x2E, //   Usage (0x2E)
      0x95, 0x01, //   Report Count (1)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0xE0, //   Report ID (-32)
      0x09, 0x2F, //   Usage (0x2F)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0xF0, //   Report ID (-16)
      0x09, 0x30, //   Usage (0x30)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0xF1, //   Report ID (-15)
      0x09, 0x31, //   Usage (0x31)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0xF2, //   Report ID (-14)
      0x09, 0x32, //   Usage (0x32)
      0x95, 0x0F, //   Report Count (15)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0xF4, //   Report ID (-12)
      0x09, 0x35, //   Usage (0x35)
      0x95, 0x3F, //   Report Count (63)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0x85, 0xF5, //   Report ID (-11)
      0x09, 0x36, //   Usage (0x36)
      0x95, 0x03, //   Report Count (3)
      0xB1, 0x02, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                  //   Null Position,Non-volatile)
      0xC0,       // End Collection
  };
}

void DualsenseHandler::on_new_connection(Session &current_session,
                                         error_code &ec) {
  HidVirtualInterfaceHandler::on_new_connection(current_session, ec);

  client_connected_ = true;
  client_connected_.notify_all();
  client_connect_cv_.notify_all();

  should_stop_ = false;
  {
    std::lock_guard lock(state_mutex_);
    current_state_ = GamepadState{};
    last_state_ = GamepadState{};
  }
}

void DualsenseHandler::on_disconnection(error_code &ec) {
  should_stop_ = true;
  // state_cv_.notify_all();
  // if (send_thread_.joinable())
  //   send_thread_.join();
  client_connected_ = false;
  HidVirtualInterfaceHandler::on_disconnection(ec);
}

std::uint16_t DualsenseHandler::get_report_descriptor_size() {
  return static_cast<std::uint16_t>(report_descriptor_.size());
}

data_type DualsenseHandler::get_report_descriptor() {
  return report_descriptor_;
}

void DualsenseHandler::on_output_report_received(asio::const_buffer data) {
    // TODO: types
    if (data.size() > 0 && static_cast<const uint8_t *>(data.data())[0] == 0x02) {
        if (data.size() < sizeof(ReportOut02)) {
            SPDLOG_ERROR("Received SetState report with invalid length: {}", data.size());
            return;
        }
        const ReportOut02 *report = static_cast<const ReportOut02 *>(data.data());
        left_rumble_ = report->State.RumbleEmulationLeft;
        right_rumble_ = report->State.RumbleEmulationRight;
    }
}

data_type DualsenseHandler::request_get_report(std::uint8_t type,
                                               std::uint8_t report_id,
                                               std::uint16_t length,
                                               std::uint32_t *p_status) {
  printf("DualsenseHandler::request_get_report(type=%u, report_id=%u, length=%u)\n",
         type, report_id, length);
  if (static_cast<HIDReportType>(type) == HIDReportType::Input) {
    std::lock_guard lock(state_mutex_);
    data_type result(REPORT_SIZE, 0);
    result[0] = current_state_.buttons & 0xFF;
    result[1] = (current_state_.buttons >> 8) & 0xFF;
    result[2] = current_state_.hat;
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
      uint16_t val =
          static_cast<uint16_t>(static_cast<int16_t>(current_state_.axes[i]));
      result[3 + i * 2] = val & 0xFF;
      result[4 + i * 2] = (val >> 8) & 0xFF;
    }
    return result;
  }
  if (static_cast<HIDReportType>(type) == HIDReportType::Feature) {
    if (report_id == 0x09) {
      ReportFeatureInMacAll report{};
      report.ReportID = 0x09;
      uint8_t mac_bytes[] = {0x83, 0x47, 0x31, 0x9b, 0xb9, 0x4c};
      memcpy(report.ClientMac, mac_bytes, 6);
      report.Hard08 = 0x08;
      report.Hard25 = 0x25;
      report.Hard00 = 0x00;
      uint8_t host_mac_bytes[] = {0x00, 0x1A, 0x7D, 0xDA, 0x71, 0x13};
      memcpy(report.HostMac, host_mac_bytes, 6);
      return data_type(reinterpret_cast<uint8_t *>(&report),
                       reinterpret_cast<uint8_t *>(&report) + sizeof(report));
    } else if (report_id == 0x20) {
      ReportFeatureInVersion report{};
      report.ReportID = 0x20;
      strncpy(report.BuildDate, "May 18 2022",
              sizeof(report.BuildDate));
      strncpy(report.BuildTime, "09:33:22",
              sizeof(report.BuildTime));
      report.FwType = 0x0003;
      report.SwSeries = 0x0004;
      report.HardwareInfo = 0x00000313;
      report.FirmwareVersion = 0x01040027;
      report.DeviceInfo[0] = 0x41;
      report.DeviceInfo[1] = 0x0a;
      report.FwVersion1 = 0x0001002a;
      report.FwVersion2 = 0x0001000b;
      report.FwVersion3 = 0x00000006;
      return data_type(reinterpret_cast<uint8_t *>(&report),
                       reinterpret_cast<uint8_t *>(&report) + sizeof(report));
    } else if (report_id == 0x05) {
      ReportFeatureInCalibrateBT report{};
      report.ReportID = 0x05;
      // report.GyroPitchBias = 0xffff;
      // report.GyroYawBias = 0xfffe;
      // report.GyroRollBias = 0x0007;
      // report.GyroPitchPlus = 0x22a5;
      // report.GyroPitchMinus = 0xdd59;
      // report.GyroYawPlus = 0x229e;
      // report.GyroYawMinus = 0xdd5e;
      // report.GyroRollPlus = 0x22aa;
      // report.GyroRollMinus = 0xdd64;
      // report.GyroSpeedPlus = 0x021c;
      // report.GyroSpeedMinus = 0x021c;
      // report.AccelXPlus = 0x1fe3;
      // report.AccelXMinus = 0xdfeb;
      // report.AccelYPlus = 0x1fd2;
      // report.AccelYMinus = 0xdfd8;
      // report.AccelZPlus = 0x2002;
      // report.AccelZMinus = 0x2002;
      // report.Unknown = 0x0007;
      return data_type(reinterpret_cast<uint8_t *>(&report),
                       reinterpret_cast<uint8_t *>(&report) + sizeof(report));
    } else {
      SPDLOG_ERROR("Unsupported feature report ID: {}", report_id);
    }
    data_type result(length, 0);
    return result;
  }
  *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
  return {};
}

void DualsenseHandler::request_set_report(std::uint8_t type, std::uint8_t report_id,
                                                               std::uint16_t length, const data_type &data,
                                                               std::uint32_t *p_status) {
    if (type == 0x03 && report_id == 0x08) {
      *p_status = 0;
      return;
    }
    SPDLOG_WARN("unhandled request_set_report (type={}, report_id={}, length={})", type, report_id, length);
    HidVirtualInterfaceHandler::request_set_report(type, report_id, length, data, p_status);
}

// ========== 按钮 API ==========

void DualsenseHandler::set_button(uint8_t index, bool pressed) {
  if (index >= NUM_BUTTONS)
    return;
  std::lock_guard lock(state_mutex_);
  uint16_t old = current_state_.buttons;
  if (pressed)
    current_state_.buttons |= (1u << index);
  else
    current_state_.buttons &= ~(1u << index);
  if (current_state_.buttons != old)
    state_cv_.notify_one();
}

bool DualsenseHandler::get_button(uint8_t index) const {
  if (index >= NUM_BUTTONS)
    return false;
  std::lock_guard lock(state_mutex_);
  return (current_state_.buttons >> index) & 1;
}

void DualsenseHandler::press_buttons(std::initializer_list<uint8_t> indices) {
  std::lock_guard lock(state_mutex_);
  current_state_.buttons = 0;
  for (auto idx : indices) {
    if (idx < NUM_BUTTONS)
      current_state_.buttons |= (1u << idx);
  }
  state_cv_.notify_one();
}

void DualsenseHandler::release_all_buttons() {
  std::lock_guard lock(state_mutex_);
  if (current_state_.buttons != 0) {
    current_state_.buttons = 0;
    state_cv_.notify_one();
  }
}

// ========== D-pad API ==========

void DualsenseHandler::set_hat(HatDirection dir) {
  std::lock_guard lock(state_mutex_);
  uint8_t val = static_cast<uint8_t>(dir);
  if (current_state_.hat != val) {
    current_state_.hat = val;
    state_cv_.notify_one();
  }
}

DualsenseHandler::HatDirection DualsenseHandler::get_hat() const {
  std::lock_guard lock(state_mutex_);
  return static_cast<HatDirection>(current_state_.hat);
}

// ========== 模拟轴 API ==========

void DualsenseHandler::set_axis(uint8_t index, int16_t value) {
  if (index >= NUM_AXES)
    return;
  std::lock_guard lock(state_mutex_);
  if (current_state_.axes[index] != value) {
    current_state_.axes[index] = value;
    state_cv_.notify_one();
  }
}

int16_t DualsenseHandler::get_axis(uint8_t index) const {
  if (index >= NUM_AXES)
    return 0;
  std::lock_guard lock(state_mutex_);
  return current_state_.axes[index];
}

bool DualsenseHandler::wait_for_client(int timeout_ms) {
  if (timeout_ms < 0) {
    client_connected_.wait(false);
    return true;
  }
  std::unique_lock lock(client_connect_mutex_);
  return client_connect_cv_.wait_for(
      lock, std::chrono::milliseconds(timeout_ms),
      [this] { return client_connected_.load(); });
}

} // namespace usbipdcpp
