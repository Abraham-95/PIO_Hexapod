#include <Dynamixel2Arduino.h>
#include "State.h"
#include "LegManager.h"
#include "Com.h"

Vector3 jumpBasePoints[6];

enum JumpPhase{
  JUMP_IDLE,
  JUMP_PRELOAD,
  JUMP_PUSH,
  JUMP_AIR,
  JUMP_LAND
};

JumpPhase jumpPhase;
unsigned long jumpTimer;

const float preloadDepth = -40;
const float pushDepth    = -160;
const float airLift      = -50;
const int preloadTime = 200;
const int pushTime    = 70;
const int airTime     = 150;
const int landTime    = 180;


void JumpingState::init() {
  Serial.println("Jumping State.");
  for(int i=0;i<6;i++){jumpBasePoints[i] = currentLegPositions[i];}
  jumpPhase = JUMP_IDLE;
}

void JumpingState::exit() {
}

void JumpingState::loop() {
  int joyRy = rc_control_data.axisRY;
  bool jumpTrigger = joyRy < -30;
  unsigned long now = millis();

  switch(jumpPhase){
    case JUMP_IDLE:
      if(jumpTrigger){jumpPhase = JUMP_PRELOAD; jumpTimer = now;} break;
    case JUMP_PRELOAD:
      for(int i=0;i<6;i++){
        Vector3 p = jumpBasePoints[i];
        p.z += preloadDepth; moveToPos(i,p);}
      if(now - jumpTimer > preloadTime){
        jumpPhase = JUMP_PUSH; jumpTimer = now;
      } break;
    case JUMP_PUSH:
      for(int i=0;i<6;i++){
        Vector3 p = jumpBasePoints[i];
        p.z -= pushDepth; moveToPos(i,p);
      }
      if(now - jumpTimer > pushTime){
        jumpPhase = JUMP_AIR; jumpTimer = now;
      } break;
    case JUMP_AIR:
      for(int i=0;i<6;i++){
        Vector3 p = jumpBasePoints[i];
        p.z -= airLift; moveToPos(i,p);
      }
      if(now - jumpTimer > airTime){
        jumpPhase = JUMP_LAND; jumpTimer = now;
      } break;
    case JUMP_LAND:
      for(int i=0;i<6;i++){moveToPos(i,jumpBasePoints[i]);}
      if(now - jumpTimer > landTime){jumpPhase = JUMP_IDLE;} break;
  }
}


