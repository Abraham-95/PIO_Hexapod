#include "Com.h"

// Declare the data package variables
RC_Control_Data_State rc_control_data;
RC_Settings_Data_Package rc_settings_data;
Hexapod_Settings_Data_Package hex_settings_data;
Hexapod_Sensor_Data_Package hex_sensor_data;

unsigned long rc_last_received_time = 0;
unsigned long lastSignalMillis = 0;

byte receiveType = RC_CONTROL_DATA;
bool controllerConnected = false;

int16_t gyroRoll  = 0;
int16_t gyroPitch = 0;
int16_t gyroYaw   = 0;

#if DEBUG_UART_RX
static uint32_t packetCount = 0;
static uint32_t checksumErrorCount = 0;
static uint32_t lastDebugPrint = 0;
#endif

void setupCom() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  Serial3.begin(115200);
  Serial3.setTimeout(50);

  initializeHexPayload();
  initializeControllerPayload();
}

void blinkLED() {
  digitalWrite(LED_BUILTIN, HIGH); delay(50);
  digitalWrite(LED_BUILTIN, LOW); delay(50);
}

void initializeHexPayload() {
  hex_sensor_data.type = HEXAPOD_SENSOR_DATA;
  hex_settings_data.type = HEXAPOD_SETTINGS_DATA;
  Serial.println("Filling hex_settings_data.calibrationOffsets with 0's.");
  for (int i = 0; i < 18; i++) {
    hex_settings_data.calibrationOffsets[i] = 0;
  }
}

void initializeControllerPayload() {
    // control package
    rc_control_data.type = RC_CONTROL_DATA;

    rc_control_data.axisX = 0;
    rc_control_data.axisY = 0;
    rc_control_data.axisRX = 0;
    rc_control_data.axisRY = 0;
    rc_control_data.gyro_X = 0;
    rc_control_data.gyro_Y = 0;
    rc_control_data.gyro_Z = 0;
    rc_control_data.buttons = UNPRESSED;
    rc_control_data.misc = UNPRESSED;
    rc_control_data.dpad = UNPRESSED;

    // settings package
    rc_settings_data.type = RC_SETTINGS_DATA;
    rc_settings_data.calibrating = 0;
    rc_settings_data.increaseValue = UNPRESSED;
    rc_settings_data.decreaseValue = UNPRESSED;
    rc_settings_data.calibrationIndex = -1; // -1 means no calibration index is set
}

bool receiveComData() {
  static enum {
    WAIT_HEADER1, WAIT_HEADER2, WAIT_TYPE,
    WAIT_LENGTH, WAIT_DATA, WAIT_CHECKSUM
  } state = WAIT_HEADER1;

  static uint8_t type;
  static uint8_t length;
  static uint8_t data[16];
  static uint8_t index;
  static uint8_t checksum;

  while (Serial3.available()) {
    uint8_t byteIn = Serial3.read();
    switch(state) {
      case WAIT_HEADER1: {
        checksum = 0;
        if (byteIn == HEADER1) {state = WAIT_HEADER2;} break;}
      case WAIT_HEADER2: {
        if (byteIn == HEADER2) {state = WAIT_TYPE;} else {state = WAIT_HEADER1;} break;}
      case WAIT_TYPE: {
        type = byteIn; checksum = byteIn; state = WAIT_LENGTH; break;}
      case WAIT_LENGTH: {
        length = byteIn; checksum ^= byteIn;
        if (length > sizeof(data)) {state = WAIT_HEADER1; break;}
        index = 0; state = WAIT_DATA;  break;}
      case WAIT_DATA: {
        data[index++] = byteIn; checksum ^= byteIn;
        if (index >= length) {state = WAIT_CHECKSUM;} break;}
      case WAIT_CHECKSUM: {
        if (checksum == byteIn) {
      #if DEBUG_UART_RX
        packetCount++;
      #endif
          if (type == TYPE_CONTROL && length == 7) {
            rc_control_data.buttons   = data[0];
            rc_control_data.misc      = data[1];
            rc_control_data.dpad      = data[2];
            rc_control_data.axisX     = (int8_t)data[3];
            rc_control_data.axisY     = (int8_t)data[4];
            rc_control_data.axisRX    = (int8_t)data[5];
            rc_control_data.axisRY    = (int8_t)data[6];
            rc_last_received_time = millis();
          }
          else if (type == TYPE_EVENT && length == 1) {
          switch(data[0]) {
            case EVENT_CONNECTED: // Controller Connected
              controllerConnected = true;
              sendRobotData();
              break;
            case EVENT_DISCONNECTED: // Controller Disconnected
                controllerConnected = false; break;
            }
          } else {
      #if DEBUG_UART_RX
        checksumErrorCount++;
      #endif
          }
        }
        state = WAIT_HEADER1; break;
      }
    }
  }
  #if DEBUG_UART_RX
  if (millis() - lastDebugPrint > 100) {  // 5x per detik max
    lastDebugPrint = millis();
    Serial.println("======== UART RX DEBUG ========");
    Serial.print("Packets OK : "); Serial.println(packetCount);
    Serial.print("Checksum Err : "); Serial.println(checksumErrorCount);
    Serial.print("Buttons: 0x"); Serial.println(rc_control_data.buttons, HEX);
    Serial.print("Misc   : 0x"); Serial.println(rc_control_data.misc, HEX);
    Serial.print("DPad   : 0x"); Serial.println(rc_control_data.dpad, HEX);
    Serial.print("LX: "); Serial.print(rc_control_data.axisX);
    Serial.print("  LY: "); Serial.print(rc_control_data.axisY);
    Serial.print("  RX: "); Serial.print(rc_control_data.axisRX);
    Serial.print("  RY: "); Serial.println(rc_control_data.axisRY);
    Serial.println("===============================");
  }
  #endif

  if (millis() - rc_last_received_time > INTERVAL_MS_SIGNAL_LOST) {
    rc_control_data.axisX = 0; rc_control_data.axisY = 0;
    rc_control_data.axisRX = 0; rc_control_data.axisRY = 0;
    rc_control_data.buttons = 0;
    rc_control_data.dpad = 0;
    rc_control_data.misc = 0;
    return false;
  }
  return true;
}

ButtonEvent readButtonEvent() {
  static uint8_t prevButtons = 0;
  static uint8_t prevDpad    = 0;

  ButtonEvent event = BUTTON_NONE;

  uint8_t b = rc_control_data.buttons;
  uint8_t d = rc_control_data.dpad;

  if ((b & 0x01) && !(prevButtons & 0x01)) event = BUTTON_X;
  else if ((b & 0x04) && !(prevButtons & 0x04)) event = BUTTON_SQUARE;
  else if ((b & 0x08) && !(prevButtons & 0x08)) event = BUTTON_TRIANGLE;
  else if ((b & 0x02) && !(prevButtons & 0x02)) event = BUTTON_CIRCLE;
  else if ((d & 0x01) && !(prevDpad & 0x01)) event = BUTTON_DPAD_UP;
  else if ((d & 0x02) && !(prevDpad & 0x02)) event = BUTTON_DPAD_DOWN;
  else if ((d & 0x08) && !(prevDpad & 0x08)) event = BUTTON_DPAD_LEFT;
  else if ((d & 0x04) && !(prevDpad & 0x04)) event = BUTTON_DPAD_RIGHT;

  prevButtons = b;
  prevDpad = d;

  //if(event != BUTTON_NONE) blinkLED();

  return event;
}

void sendRobotData() {
    static uint8_t lastState = 255;
    static uint8_t lastGait  = 255;

    uint8_t state = getStateCode();
    uint8_t gait  = getGaitCode();

    if(state == lastState && gait == lastGait) return;

    lastState = state;
    lastGait  = gait;

    RobotStatusPacket packet;
    packet.header = 0xAA;
    packet.state = getStateCode();
    packet.gait  = getGaitCode();

    Serial3.write((uint8_t*)&packet,sizeof(packet));
}

