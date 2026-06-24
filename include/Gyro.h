#pragma once

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Adafruit_I2CDevice.h>
#include "Com.h"

#define SDA_PIN 11
#define SCL_PIN 12
#define BNO_ADDR  0x4B
#define DEBUG_GYRO 0  // 0->OFF, 1->ON

extern BNO080 myIMU;
extern float roll, pitch, yaw;
extern float pitchFiltered, rollFiltered;
extern uint32_t lastGyroTime;

struct BalanceOutput {
    float xShift;
    float yShift;
    float pitchOffset;
    float rollOffset;
    float zmpX;
    float zmpY;
};

enum BalanceMode {
    STANDING_MODE,
    WALKING_MODE
};

extern BalanceOutput balanceOutput;
extern BalanceMode balanceMode;

void setupGyro();
void gyroUpdate();
void stabilization(BalanceMode mode);
