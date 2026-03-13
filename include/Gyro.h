#pragma once
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <MahonyAHRS.h>
#include <math.h>

// #define DEBUG_GYRO   // debug gate

extern MPU9250_asukiaaa mySensor;
extern Mahony filter;

extern float aX, aY, aZ;
extern float gX, gY, gZ;
extern float mX, mY, mZ;
extern float pitch, roll, yaw;
extern unsigned long prevTime;

void setupGyro();
void gyroStart();
