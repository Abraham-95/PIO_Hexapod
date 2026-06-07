#pragma once
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ========== Pins ==========
#define SDA_PIN 11
#define SCL_PIN 12

// ========== I2C addresses ==========
#define BNO_ADDR  0x4B
#define OLED_ADDR 0x3C

// ========== OLED setup ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

extern Adafruit_SSD1306 display;
extern BNO080 myIMU;
extern float roll, pitch, yaw;

void setupGyro();
void gyroUpdate();
