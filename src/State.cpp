#include "State.h"

void changeState(State* next) {
  if (next == nullptr || next == currentState) return;
  if (currentState) currentState->exit();
  previousState = currentState;
  currentState = next;
  currentState->init();
}

const char* stateToString(State* state) {
  if(state == initializationState) return "INITIALIZE";
  if(state == sleepState)          return "SLEEP STATE";
  if(state == standingState)       return "STANDING STATE";
  if(state == walkingState)        return "WALKING STATE";
  if(state == gyroState)           return "GYRO STATE";
  if(state == jumpingState)        return "JUMPING STATE";

  return "UNKNOWN";
}

uint8_t getStateCode() {
  if(currentState == standingState) return 0;
  if(currentState == walkingState)  return 1;
  if(currentState == gyroState)     return 2;
  if(currentState == jumpingState)  return 3;
  if(currentState == sleepState)    return 4;
  return 255;
}
