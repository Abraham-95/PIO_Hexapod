#pragma once

#include <Dynamixel2Arduino.h>
#include "Utils.h"
#include "Gait.h"
#include <vector>

#define NUM_LEGS 6
#define GOAL_POSITION_ADDR 30
#define GOAL_POSITION_LEN  2
#define TOTAL_SERVOS (NUM_LEGS * 3)

extern DYNAMIXEL::InfoSyncWriteInst_t syncWriteInfo;
extern DYNAMIXEL::XELInfoSyncWrite_t xels[TOTAL_SERVOS];

extern uint8_t goalPositionData[TOTAL_SERVOS][2];
extern uint8_t syncWritePacketBuf[256];
extern int16_t goalRaw[TOTAL_SERVOS];

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;

extern Dynamixel2Arduino dxl;

extern Vector3 currentLegPositions[NUM_LEGS];
extern int8_t calibrationOffsets[18];
extern bool gyroTilt;
//extern Vector3 cycleStartPositions[NUM_LEGS];

extern Vector2 joyLTargetVector;
extern float joyLTargetMagnitude;

extern Vector2 joyLCurrentVector;
extern float joyLCurrentMagnitude;

extern Vector2 joyRTargetVector;
extern float joyRTargetMagnitude;

extern Vector2 joyRCurrentVector;
extern float joyRCurrentMagnitude;

extern float points;

extern Vector3 saved_offsets[NUM_LEGS];

const uint8_t DXL_ID_COXA[NUM_LEGS] = {1, 4, 7, 10, 13, 16};
const uint8_t DXL_ID_FEMUR[NUM_LEGS] = {2, 5, 8, 11, 14, 17};
const uint8_t DXL_ID_TIBIA[NUM_LEGS] = {3, 6, 9, 12, 15, 18};

const float coxa = 52;   // coxa length
const float femur = 83;  // femur length
const float tibia = 148; // tibia length
const float totalLegLength = coxa + femur + tibia;

const float COXA_BASE_OFFSET = 0;
const float FEMUR_BASE_OFFSET = 0;
const float TIBIA_BASE_OFFSET = 0;

const float strideMultiplier[6] = {-1, -1, -1, 1, 1, 1};
const float rotationMultiplier[6] = {1, 0, -1, 1, 0, -1};
const float globalLegPlacementRadians[6] = {
    135.0 * DEG_TO_RAD, // Leg 0: Front Left
    180.0 * DEG_TO_RAD, // Leg 1: Middle Left
    215.0 * DEG_TO_RAD, // Leg 2: Back Left
    325.0 * DEG_TO_RAD, // Leg 3: Back Right
    0.0   * DEG_TO_RAD, // Leg 4: Middle Right
    35.0  * DEG_TO_RAD  // Leg 5: Front Right
};

const unsigned long loopFrequency = 120; // Hz (adjust as needed)
const unsigned long loopPeriod = 1000000 / loopFrequency; // microseconds

const float distanceFromCenter = 65;
const float distanceFromGround = -90;
const float distanceFromHexapodBottomToFemurJoint = 40;
const float maxDistanceFromGround = 200;
const float legLandHeight = 45;
const float legPlacementAngle = 55;

const float globalSpeedMult = 0.6;
const float globalRotationStrideLengthMult = 0.3;
const float globalStrafeStrideLengthMult = 0.3;
const float globalLiftHeightMult = 0.5;

const Vector3 baseLegCalibrationPosition = Vector3(0, coxa, femur+tibia);

const int TIME_TO_STAND = 1000;
const int TIME_TO_SLEEP = 30000;

const int POTENTIO_PIN = A0;

enum LegState {
    Propelling,
    Lifting,
    Standing,
    Reset
};

Vector3 posToAngle(Vector3 pos);
void setLegAngles(int leg, Vector3 angles);
void moveToPos(int leg, Vector3 pos);
void attachServos();
void saveCalibrationOffsets();
void loadCalibrationOffsets();
void printCalibrationOffsets();
void updateRuntimeVariables();
void updateServos();
Vector3 convertLocalLegPointToGlobal(Vector3 localLegPoint, int legIndex);
Vector3 convertGlobalLegPointToLocal(Vector3 globalLegPoint, int legIndex);
