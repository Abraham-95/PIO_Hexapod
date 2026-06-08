#include "Display.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setupDisplay() {
  // Start OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    //Serial.println("OLED not found at 0x3C");
    while (1) delay(10);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setTextSize(1); display.setCursor(0, 0);
  display.println("Display Booting Up"); display.display();
}

void drawDisplay() {
  static uint32_t lastDisplayUpdate = 0;
  if(millis() - lastDisplayUpdate < 100) return;
  lastDisplayUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);

  display.drawRect(0,0,128,64,SSD1306_WHITE);
  display.drawLine(0,10,127,10,SSD1306_WHITE);
  display.drawLine(0,30,127,30,SSD1306_WHITE);

  display.setCursor(44,2); display.print("HEXAPOD");

  display.setCursor(3, 13); display.print("STATE");
  display.setCursor(41, 13); display.print(stateToString(currentState));

  display.setCursor(3, 21); display.print("GAIT");
  display.setCursor(41, 21); display.print(gaitToString(currentGait));

  display.setCursor(3, 33); display.print("ROLL");
  display.setCursor(41, 33); display.print(pitch, 1);

  display.setCursor(3, 43); display.print("PITCH");
  display.setCursor(41, 43); display.print(roll, 1);

  display.setCursor(3, 53); display.print("YAW");
  display.setCursor(41, 53); display.print(yaw, 1);

  display.display();
}

