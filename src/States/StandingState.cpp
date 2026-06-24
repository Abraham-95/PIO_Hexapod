#include <Dynamixel2Arduino.h>
#include <vector>
#include <algorithm>
#include "State.h"
#include "LegManager.h"

// Standing Control Points Array - Changed to a vector of vectors
Vector3 SCPA[6][3];

Vector3 standingStartPoints[6];      //the points the legs are at in the beginning of the standing state
Vector3 standingInBetweenPoints[6];  //the middle points of the bezier curves that the legs will follow to smoothly transition to the end points
Vector3 standingEndPoint;

enum StandPhase {STAND_ADJUSTING, STAND_HOLD};
StandPhase StandingPhase;
unsigned long startTime;

int legAdjustOrder[6] = {1, 2, 3, 4, 5, 6}; // the order in which the legs will be adjusted to stand
int standProgress[6] = {0, 0, 0, 0, 0, 0};
unsigned int legAdjustTimeOffset = 100;

bool moveAllAtOnce = false;

void StandingState::init() {
  Serial.println("Standing State.");
  cGait = getGait(currentGait);
  moveAllAtOnce = (previousState == sleepState);
  calculateLegAdjustOrder();
  memcpy(standingStartPoints, currentLegPositions, sizeof(currentLegPositions));
  standingEndPoint = Vector3(0, distanceFromCenter, distanceFromGround);

  // Calculate the inbetween and ending points
  for (int i = 0; i < 6; i++) {
    Vector3 mid = standingStartPoints[i];
    mid.x = (mid.x + standingEndPoint.x) * 0.5f;
    mid.y = (mid.y + standingEndPoint.y) * 0.5f;
    mid.z = max(mid.z, standingEndPoint.z) + 60.0f;

    standingInBetweenPoints[i] = mid;
    SCPA[i][0] = standingStartPoints[i];
    SCPA[i][1] = standingInBetweenPoints[i];
    SCPA[i][2] = standingEndPoint;
    standProgress[i] = 0;
  }
  startTime = millis();
  StandingPhase = STAND_ADJUSTING;
}

void StandingState::exit() {
}

void StandingState::loop() {
  if (StandingPhase == STAND_ADJUSTING) {bool allDone = true;
    for (int i = 0; i < 6; i++) {int leg = legAdjustOrder[i];
      if (standProgress[leg] < points) {allDone = false;
        if (moveAllAtOnce || millis() - startTime >= i * legAdjustTimeOffset) {
          standProgress[leg] = min(standProgress[leg] + 40, (int)points);
        }
        float t = standProgress[leg] / points;
        moveToPos(leg, GetPointOnBezierCurve(SCPA[leg], 3, t));
      }
    }
    if (allDone) {StandingPhase = STAND_HOLD;}
  }
  else if (StandingPhase == STAND_HOLD) {
    standingEndPoint = Vector3(0, distanceFromCenter, distanceFromGround);

    #if DEBUG_GYRO
    static uint32_t lastDebug = 0;
    bool printDebug = false;

    if (millis() - lastDebug >= 100) { // 10 Hz
      lastDebug = millis();
      printDebug = true;

      Serial.println("\n========= STANDING DEBUG =========");
      Serial.print("Pitch Raw : "); Serial.print(pitch, 1);
      Serial.print("\tPitch Filt : "); Serial.println(pitchFiltered, 1);

      Serial.print("Roll Raw  : "); Serial.print(roll, 1);
      Serial.print("\tRoll Filt : "); Serial.println(rollFiltered, 1);
      Serial.println("----------------------------------");
    }
    #endif

    stabilization(STANDING_MODE);

    for (int i = 0; i < 6; i++) {
      Vector3 bodyPoint = convertLocalLegPointToGlobal(standingEndPoint, i);

      bodyPoint.x -= balanceOutput.xShift;
      bodyPoint.y -= balanceOutput.yShift;

      bodyPoint.z += bodyPoint.x * sin(balanceOutput.rollOffset)
                   - bodyPoint.y * sin(balanceOutput.pitchOffset);

      Vector3 localPoint = convertGlobalLegPointToLocal(bodyPoint, i);
      moveToPos(i, localPoint);

      //SCPA[i][2] = standingEndPoint;
      //moveToPos(i, standingEndPoint);

      #if DEBUG_GYRO
      if (printDebug) {
        Serial.print("Leg "); Serial.print(i);
        Serial.print(" X ="); Serial.print(localPoint.x);
        Serial.print(" Y ="); Serial.print(localPoint.y);
        Serial.print(" Z ="); Serial.println(localPoint.z);
      }
      #endif
    }
  }
}

void StandingState::calculateLegAdjustOrder(){
  // Fill with indices 0..5
  for (int i = 0; i < 6; i++) {legAdjustOrder[i] = i;}

  // Sort indices by z value, highest first
  std::sort(legAdjustOrder, legAdjustOrder + 6, [](int a, int b) {
    return currentLegPositions[a].z > currentLegPositions[b].z; });
}

