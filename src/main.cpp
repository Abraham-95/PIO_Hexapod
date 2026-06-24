#include <Dynamixel2Arduino.h>
#include "Utils.h"
#include "State.h"
#include "Com.h"
#include "LegManager.h"
#include "Gait.h"
#include "Gyro.h"

InitializationState *initializationState = new InitializationState();
SleepState *sleepState = new SleepState();
StandingState *standingState = new StandingState();
WalkingState *walkingState = new WalkingState();
JumpingState *jumpingState = new JumpingState();
GyroState *gyroState = new GyroState();

State *currentState = nullptr;
State *previousState = nullptr;

enum ControlMode {MANUAL, AUTO};
ControlMode controlMode = MANUAL;

void setup() {
  //DEBUG_SERIAL.begin(115200);
  //while (!DEBUG_SERIAL);
  unsigned long startTime = millis();
  while (!DEBUG_SERIAL && millis() - startTime < 1000) {}

  attachServos();
  setupCom();
  setupGyro();
  setGait(currentGait);
  loadCalibrationOffsets();

  currentState = initializationState;
  previousState = nullptr;
  currentState->init();
  changeState(standingState);
}

void loop() {
  static unsigned long previousLoopTime = 0;
  unsigned long currentLoopTime = micros();
  if (currentLoopTime - previousLoopTime < loopPeriod) return;
  previousLoopTime = currentLoopTime;

  bool connected = receiveComData();
  updateRuntimeVariables();

  //================ State Manager ================
  //controller is not connected
  if (!connected && controlMode == AUTO) {changeState(sleepState);}

  //=================== Walking ===================
  if (receiveType == RC_CONTROL_DATA) {
    ButtonEvent event = readButtonEvent();
    switch (event) {
      case BUTTON_X: changeState(standingState); break;
      case BUTTON_SQUARE: changeState(sleepState); break;
      case BUTTON_TRIANGLE: changeState(gyroState); break;
      case BUTTON_CIRCLE: changeState(jumpingState); break;
      case BUTTON_DPAD_UP: break;
      case BUTTON_DPAD_DOWN: break;
      case BUTTON_DPAD_RIGHT:
        if (currentState == walkingState  || currentState == standingState) {
          setGait(nextGait(currentGait));
        }
        break;
      case BUTTON_DPAD_LEFT:
        if (currentState == walkingState || currentState == standingState) {
          setGait(prevGait(currentGait));
        }
        break;
      default: break;
    }
    static unsigned long lastMovementTime = 0;

    double joyLx = rc_control_data.axisX;
    double joyLy = rc_control_data.axisY;
    double joyRx = rc_control_data.axisRX;
    double joyRy = rc_control_data.axisRY;

    bool movementDetected = joyLx > 30 || joyLx < -30 || joyLy > 30 || joyLy < -30 ||
                            joyRx > 30 || joyRx < -30 || joyRy > 30 || joyRy < -30;

    if (movementDetected) {
      lastMovementTime = millis();
      if (currentState == sleepState) changeState(standingState);
      else if (currentState == standingState) changeState(walkingState);}
    else {
      unsigned long idleTime = millis() - lastMovementTime;
      if (currentState == walkingState && idleTime >= TIME_TO_STAND) {
        changeState(standingState);
      }
      if (currentState == standingState && idleTime >= TIME_TO_SLEEP) {
        changeState(sleepState);
      }
    }
  }
  if (currentState) {currentState->loop();}
  //sendRobotData();
  gyroUpdate();
  updateServos();

  //Freq debug
  //static uint32_t t0 = micros(); uint32_t now = micros();
  //Serial.println(now - t0); t0 = now;
}



