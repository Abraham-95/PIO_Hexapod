#include "Gyro.h"

BNO080 myIMU;
float roll = 0; float pitch = 0; float yaw = 0;

void setupGyro() {
  delay(100);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Start BNO080 at 0x4B
  if (!myIMU.begin(BNO_ADDR, Wire)) {
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0, 0); display.println("BNO080 not found");
    display.display();
    while (1) delay(10);
  }
  // Enable Euler angles (via Rotation Vector)
  myIMU.enableRotationVector(50); // 50ms (20Hz)
}

void gyroUpdate() {
  static uint32_t lastGyroUpdate = 0;
  if(millis() - lastGyroUpdate >= 20){
      gyroUpdate();lastGyroUpdate = millis();
  }

  if (myIMU.dataAvailable()) {
    // Convert radians to degrees
    roll  = myIMU.getRoll()  * 180.0f / PI;
    pitch = myIMU.getPitch() * 180.0f / PI;
    yaw   = myIMU.getYaw()   * 180.0f / PI;
  }
}

