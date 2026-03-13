#include <Dynamixel2Arduino.h>
#include "State.h"
#include "Com.h"
#include "LegManager.h"

extern Dynamixel2Arduino dxl;
enum SleepPhase {SLEEP_MOVING, SLEEP_HOLD};
SleepPhase sleepPhase;

void SleepState::init() {
    Serial.println("Sleep State.");
    target = Vector3(0, 130, 0);
    sleepPhase = SLEEP_MOVING;
}

void SleepState::exit() {
}

void SleepState::loop() {
    /*int potValue = analogRead(POTENTIO_PIN);
    int mappedPotValue = map(potValue, 0, 1023, -200, 200);
    Serial.println("Pot Value: " + String(potValue));
    Serial.println("Target: " + standingEndPoint.toString());*/

    if (sleepPhase == SLEEP_MOVING) {
        bool allReached = true;
        for (int i = 0; i < 6; i++) {
            Vector3 next = lerp(currentLegPositions[i], target, 0.02);
            if (currentLegPositions[i].distanceTo(target) < 1.0f) {next = target;}
            else {allReached = false;}
            moveToPos(i, next);
        }
        if (allReached) {sleepPhase = SLEEP_HOLD;}
    }
    else if (sleepPhase == SLEEP_HOLD) {
        for (int i = 0; i < 6; i++) {moveToPos(i, target);}
    }
}
