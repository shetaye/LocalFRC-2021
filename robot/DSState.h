#ifndef _DS_STATE_H_
#define _DS_STATE_H_

#include <stdint.h>

enum GamepadButton {
    A               = 0x0001,
    B               = 0x0002,
    X               = 0x0004,
    Y               = 0x0008,
    Start           = 0x0010,
    Back            = 0x0020,
    LeftTrigger     = 0x0040,
    RightTrigger    = 0x0080,
    LeftButton      = 0x0100,
    RightButton     = 0x0200,
    DPadUp          = 0x0400,
    DPadDown        = 0x0800,
    DPadRight       = 0x1000,
    DPadLeft        = 0x2000,
    Reserved1       = 0x4000,
    Resverved2      = 0x8000,
};

enum GamepadAxis {
    LeftX = 0,
    LeftY = 1,
    RightX = 2,
    RightY = 3,
    TriggerX = 4,
    TriggerY = 5
};

class Gamepad {
public:
    int8_t axis[6];
    uint16_t buttonState;

    bool getButton(GamepadButton button) {
      return buttonState & button;
    }

    bool getAxis(GamepadAxis axisID) {
      return axis[axisID];
    }
};

//enum DSMode {
//    Auto    = 1,
//    TeleOp  = 2
//};

class DriverStation {
public:
    bool estopped;
    bool enabled;
    uint8_t mode;

    uint8_t switchState;

    Gamepad gamepad1;
    Gamepad gamepad2;
};

#endif // _DS_STATE_H_
