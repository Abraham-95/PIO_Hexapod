#include "Gyro.h"

MPU9250_asukiaaa mySensor;
Mahony filter;

float aX, aY, aZ;
float gX, gY, gZ;
float mX, mY, mZ;
float pitch, roll, yaw;
unsigned long prevTime = 0;

void setupGyro() {
  // I2c Initialization
  Wire.begin();
  mySensor.setWire(&Wire);

  // Inisialisasi IMU
  mySensor.beginAccel(); mySensor.beginGyro(); mySensor.beginMag();

  filter.begin(100);
  Serial.println("Mahony Filter initialized!");
}

void gyroStart() {
  mySensor.accelUpdate(); mySensor.gyroUpdate(); //mySensor.magUpdate();
  aX = mySensor.accelX(); aY = mySensor.accelY(); aZ = mySensor.accelZ();
  gX = mySensor.gyroX(); gY = mySensor.gyroY(); gZ = mySensor.gyroZ();
  mX = mySensor.magX(); mY = mySensor.magY(); mZ = mySensor.magZ();

  // Jalankan filter Mahony
  filter.update(gX, gY, gZ, aX, aY, aZ, mX, mY, mZ);

  // Ambil hasil Euler (derajat)
  roll = filter.getRoll(); pitch = filter.getPitch(); yaw = filter.getYaw();

  // Cetak setiap 100ms (non-blocking)
  static unsigned long printTimer = 0;
  if (millis() - printTimer >= 100) {
    printTimer = millis();
    Serial.print("Pitch: "); Serial.print(pitch, 2);
    Serial.print("\tRoll: "); Serial.print(roll, 2);
    Serial.print("\tYaw: "); Serial.println(yaw, 2);
  }
}

