#pragma once

#include "Display.h"
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// ========== Pins ==========
#define SDA_PIN 11
#define SCL_PIN 12

// ========== I2C addresses ==========
#define BNO_ADDR  0x4B

extern BNO080 myIMU;
extern float roll, pitch, yaw;

void setupGyro();
void gyroUpdate();
