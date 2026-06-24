#pragma once
#include <Dynamixel2Arduino.h>
#include "State.h"
#include "Gait.h"
#include "Gyro.h"

#define HEADER1 0xAA
#define HEADER2 0xBB
#define TYPE_CONTROL 0xCC
#define TYPE_EVENT 0xCD

#define UNPRESSED 0x0
#define PRESSED 0x1

#define INTERVAL_MS_SIGNAL_LOST 1000
#define DEBUG_UART_RX 0   // 0->OFF, 1->ON

enum EventCode {
    EVENT_CONNECTED    = 1,
    EVENT_DISCONNECTED = 2
};

enum PackageType {
    RC_CONTROL_DATA = 251,
    RC_SETTINGS_DATA = 252,
    HEXAPOD_SETTINGS_DATA = 253,
    HEXAPOD_SENSOR_DATA = 254
};

enum ButtonEvent {
    BUTTON_NONE,
    BUTTON_X,
    BUTTON_SQUARE,
    BUTTON_TRIANGLE,
    BUTTON_CIRCLE,
    BUTTON_DPAD_UP,
    BUTTON_DPAD_DOWN,
    BUTTON_DPAD_LEFT,
    BUTTON_DPAD_RIGHT
};

struct RC_Control_Data_State {
    uint8_t type;
    int8_t axisX;
    int8_t axisY;
    int8_t axisRX;
    int8_t axisRY;

    int8_t gyro_X;
    int8_t gyro_Y;
    int8_t gyro_Z;

    uint8_t buttons;
    uint8_t misc;
    uint8_t dpad;
};

struct RC_Settings_Data_Package {
    uint8_t type;
    uint8_t calibrating;
    uint8_t increaseValue;
    uint8_t decreaseValue;
    uint8_t reserved;
    int8_t calibrationIndex;
};

struct Hexapod_Settings_Data_Package {
    uint8_t type;
    int8_t calibrationOffsets[18];
};

struct Hexapod_Sensor_Data_Package {
    uint8_t type;
    int8_t xPositions[6];
    int8_t yPositions[6];
    int8_t zPositions[6];
};

struct RobotStatusPacket {
    uint8_t header;
    uint8_t state;
    uint8_t gait;
};

// Declare the data package variables
extern RC_Control_Data_State rc_control_data;
extern RC_Settings_Data_Package rc_settings_data;
extern Hexapod_Settings_Data_Package hex_settings_data;
extern Hexapod_Sensor_Data_Package hex_sensor_data;

extern byte receiveType;
extern bool controllerConnected;
extern ButtonEvent readButtonEvent();

void setupCom();
bool receiveComData();
void initializeControllerPayload();
void initializeHexPayload();
void sendRobotData();

