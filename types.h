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