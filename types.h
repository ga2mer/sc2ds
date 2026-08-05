#pragma once
#include "DualsenseHandler.h"
#include <cstdint>

#define VALVE_USB_VID 0x28DE

enum ValveControllerPID {
  TRITON_PID = 0x1302,
  PUCK_PID = 0x1304,
};

enum ETritonReportIDTypes {
  ID_TRITON_CONTROLLER_STATE = 0x42,
  ID_TRITON_BATTERY_STATUS = 0x43,
  ID_TRITON_CONTROLLER_STATE_BLE = 0x45,
  ID_TRITON_WIRELESS_STATUS_X = 0x46,
  ID_TRITON_CONTROLLER_STATE_TIMESTAMP = 0x47,

  ID_TRITON_WIRELESS_STATUS = 0x79,
};

struct MsgHapticRumble {
  uint8_t report_id;
  uint8_t type;
  uint16_t intensity;

  struct __attribute__((packed)) {
    uint16_t speed;
    int8_t gain;
  } left, right;
} __attribute__((packed));

struct MsgHapticPCMStereo {
  uint8_t report_id;
  // <=31
  uint8_t length;

  // signed 8 bit pcm le
  char left[31];
  char right[31];

} __attribute__((packed));

struct TritonMTUIMUNoQuat_t {
  uint32_t timestamp;
  short sAccelX;
  short sAccelY;
  short sAccelZ;

  short sGyroX;
  short sGyroY;
  short sGyroZ;
} __attribute__((packed));

struct TritonMTUNoQuat_t {
  uint8_t seq_num;
  uint32_t buttons;
  short sTriggerLeft;
  short sTriggerRight;

  short sLeftStickX;
  short sLeftStickY;
  short sRightStickX;
  short sRightStickY;

  short sLeftPadX;
  short sLeftPadY;
  unsigned short unPressureLeft;

  short sRightPadX;
  short sRightPadY;
  unsigned short unPressureRight;
  TritonMTUIMUNoQuat_t imu;
} __attribute__((packed));

enum EChargeState {
  k_EChargeStateReset,
  k_EChargeStateDischarging,
  k_EChargeStateCharging,
  k_EChargeStateSrcValidate,
  k_EChargeStateChargingDone,
};

constexpr usbipdcpp::DualsenseHandler::PowerState TritonToDualsensePowerState(EChargeState state) {
  switch (state) {
  case k_EChargeStateDischarging:
    return usbipdcpp::DualsenseHandler::PowerState::Discharging;

  case k_EChargeStateCharging:
    return usbipdcpp::DualsenseHandler::PowerState::Charging;

  case k_EChargeStateChargingDone:
    return usbipdcpp::DualsenseHandler::PowerState::Complete;

  case k_EChargeStateReset:
  case k_EChargeStateSrcValidate:
  default:
    return usbipdcpp::DualsenseHandler::PowerState::ChargingError;
  }
}

inline uint8_t RoundPowerLevel(uint8_t powerLevel) {
  if (!powerLevel)
    return 100;
  if (powerLevel > 0 && powerLevel <= 10)
    return 1;
  return (powerLevel + 5) / 10;
}

struct TritonBatteryStatus_t {
  unsigned char ucChargeState; // EChargeState
  unsigned char ucBatteryLevel;
  unsigned short sBatteryVoltage;
  unsigned short sSystemVoltage;
  unsigned short sInputVoltage;
  unsigned short sCurrent;
  unsigned short sInputCurrent;
  unsigned short sTemperature;
} __attribute__((packed));

typedef enum {
  TRITON_LBUTTON_A = 0x00000001,
  TRITON_LBUTTON_B = 0x00000002,
  TRITON_LBUTTON_X = 0x00000004,
  TRITON_LBUTTON_Y = 0x00000008,

  TRITON_HBUTTON_QAM = 0x00000010,
  TRITON_LBUTTON_R3 = 0x00000020,
  TRITON_LBUTTON_VIEW = 0x00000040,
  TRITON_HBUTTON_R4 = 0x00000080,

  TRITON_LBUTTON_R5 = 0x00000100,
  TRITON_LBUTTON_R = 0x00000200,
  TRITON_LBUTTON_DPAD_DOWN = 0x00000400,
  TRITON_LBUTTON_DPAD_RIGHT = 0x00000800,

  TRITON_LBUTTON_DPAD_LEFT = 0x00001000,
  TRITON_LBUTTON_DPAD_UP = 0x00002000,
  TRITON_LBUTTON_MENU = 0x00004000,
  TRITON_LBUTTON_L3 = 0x00008000,

  TRITON_LBUTTON_STEAM = 0x00010000,
  TRITON_HBUTTON_L4 = 0x00020000,
  TRITON_LBUTTON_L5 = 0x00040000,
  TRITON_LBUTTON_L = 0x00080000,

  TRITON_RIGHT_JOYSTICK_TOUCH = 0x00100000,
  TRITON_RIGHT_TOUCHPAD_TOUCH = 0x00200000,
  TRITON_RIGHT_TOUCHPAD_CLICK = 0x00400000,
  TRITON_RIGHT_TRIGGER_CLICK = 0x00800000,

  TRITON_LEFT_JOYSTICK_TOUCH = 0x01000000,
  TRITON_LEFT_TOUCHPAD_TOUCH = 0x02000000,
  TRITON_LEFT_TOUCHPAD_CLICK = 0x04000000,
  TRITON_LEFT_TRIGGER_CLICK = 0x08000000,

  TRITON_RIGHT_GRIP_TOUCH = 0x10000000,
  TRITON_LEFT_GRIP_TOUCH = 0x20000000,
} TritonButtons;