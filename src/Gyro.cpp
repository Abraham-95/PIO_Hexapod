#include "Gyro.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BNO080 myIMU;

float roll = 0; float pitch = 0; float yaw = 0;

void setupGyro() {
  delay(200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Start OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    //Serial.println("OLED not found at 0x3C");
    while (1) delay(10);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Start BNO080 at 0x4B
  if (!myIMU.begin(BNO_ADDR, Wire)) {
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0, 0); display.println("BNO080 not found");
    display.display();
    while (1) delay(10);
  }
  // Enable Euler angles (via Rotation Vector)
  myIMU.enableRotationVector(50); // 50ms (20Hz)

  // Header
  display.setTextSize(1); display.setCursor(0, 0);
  display.println("BNO080 Euler (deg)"); display.display();
}

void gyroUpdate() {
  if (myIMU.dataAvailable()) {
    // Convert radians to degrees
    roll  = myIMU.getRoll()  * 180.0f / PI;
    pitch = myIMU.getPitch() * 180.0f / PI;
    yaw   = myIMU.getYaw()   * 180.0f / PI;

    // OLED update
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Roll / Pitch / Yaw");

    // Use larger text for readability
    display.setTextSize(1);

    display.setCursor(0, 12); display.print("R:"); display.print(roll, 2);
    display.setCursor(0, 30); display.print("P:"); display.print(pitch, 2);
    display.setCursor(0, 48); display.print("Y:"); display.print(yaw, 2);

    display.display();
  }
}

