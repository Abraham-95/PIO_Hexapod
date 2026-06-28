#include <Dynamixel2Arduino.h>
#include "FlashAsEEPROM.h"
#include "LegManager.h"

DYNAMIXEL::InfoSyncWriteInst_t syncWriteInfo;
DYNAMIXEL::XELInfoSyncWrite_t xels[TOTAL_SERVOS];

uint8_t goalPositionData[TOTAL_SERVOS][2];
uint8_t syncWritePacketBuf[256];
int16_t goalRaw[TOTAL_SERVOS];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

Vector3 currentLegPositions[NUM_LEGS];
//Vector3 cycleStartPositon[6];

int8_t calibrationOffsets[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

Vector3 saved_offsets[NUM_LEGS] = {
    Vector3(0, 0, 0), // leg 1
    Vector3(0, 0, 0), // leg 2
    Vector3(0, 0, 0), // leg 3
    Vector3(0, 0, 0), // leg 4
    Vector3(0, 0, 0), // leg 5
    Vector3(0, 0, 0)  // leg 6
};

Vector2 joyLTargetVector; float joyLTargetMagnitude;
Vector2 joyLCurrentVector; float joyLCurrentMagnitude;
Vector2 joyRTargetVector; float joyRTargetMagnitude;
Vector2 joyRCurrentVector; float joyRCurrentMagnitude;

float points = 1000;

bool gyroTilt = false;
const float smoothingFactor = 0.1; // Adjust this value (0.0 to 1.0)

Vector3 posToAngle(Vector3 pos) {
    float dis = Vector3(0, 0, 0).distanceTo(pos);
    if (dis > totalLegLength) {
        //Serial.println("Position out of reach");
        return Vector3(0, 0, 0);
    }
    float x = pos.x;
    float y = pos.y;
    float z = pos.z;

    float theta1 = atan2(y, x) * (180 / PI);
    float l = sqrt(x * x + y * y);
    float l1 = l - coxa;
    float h = sqrt(l1 * l1 + z * z);

    float phi1 = acos(constrain((pow(h, 2) + pow(femur, 2) - pow(tibia, 2)) / (2 * h * femur), -1, 1));
    float phi2 = atan2(z, l1);
    float theta2 = (phi1 + phi2) * 180 / PI;
    float phi3 = acos(constrain((pow(femur, 2) + pow(tibia, 2) - pow(h, 2)) / (2 * femur * tibia), -1, 1));
    float theta3 = (phi3 * 180 / PI);

    float mappedTheta1 = constrain(mapFloat(theta1, 0, 180, 60, 240), 120, 180);

    float mappedTheta2;
    if (z >= -tibia) {mappedTheta2 = map(theta2, 0, 90, 150, 60);}
    else {mappedTheta2 = map(theta2, 0, 90, 150, 240);}
    mappedTheta2 = constrain(mappedTheta2, 60, 240);

    float mappedTheta3 = constrain(map(theta3, 0, 180, 0, 150), 0, 150);

    return Vector3(
        300 - mappedTheta1 + COXA_BASE_OFFSET,
        mappedTheta2 + FEMUR_BASE_OFFSET,
        mappedTheta3 + TIBIA_BASE_OFFSET
    );
}

void setLegAngles(int leg, Vector3 angles) {
    float coxaAngle  = constrain(angles.x + saved_offsets[leg].x, 120, 180);
    float femurAngle = constrain(angles.y + saved_offsets[leg].y, 60, 240);
    float tibiaAngle = constrain(angles.z + saved_offsets[leg].z, 0, 150);

    int base = leg * 3;

    goalRaw[base+0] = map(coxaAngle,  0, 300, 0, 1023);
    goalRaw[base+1] = map(femurAngle, 0, 300, 0, 1023);
    goalRaw[base+2] = map(tibiaAngle, 0, 300, 0, 1023);

    for(int i=0;i<3;i++){
        goalPositionData[base+i][0] = goalRaw[base+i] & 0xFF;
        goalPositionData[base+i][1] = (goalRaw[base+i] >> 8) & 0xFF;
    }
}

void moveToPos(int leg, Vector3 pos) {
    currentLegPositions[leg] = pos;
    Vector3 angles = posToAngle(pos);
    setLegAngles(leg, angles);
}

void updateServos() {
    static uint32_t lastTime = 0;
    if (millis() - lastTime < 10) return;   // 100Hz servo update
    lastTime = millis();

    syncWriteInfo.is_info_changed = true;
    dxl.syncWrite(&syncWriteInfo);
}

void attachServos() {
    dxl.begin(1000000);
    dxl.setPortProtocolVersion(1.0);

    syncWriteInfo.addr = GOAL_POSITION_ADDR;
    syncWriteInfo.addr_length = GOAL_POSITION_LEN;
    syncWriteInfo.p_xels = xels;
    syncWriteInfo.xel_count = TOTAL_SERVOS;
    syncWriteInfo.is_info_changed = true;

    syncWriteInfo.packet.p_buf = syncWritePacketBuf;
    syncWriteInfo.packet.buf_capacity = sizeof(syncWritePacketBuf);
    syncWriteInfo.packet.is_completed = false;

    for(int leg=0; leg<NUM_LEGS; leg++){
        int base = leg * 3;

        xels[base+0].id = DXL_ID_COXA[leg];
        xels[base+1].id = DXL_ID_FEMUR[leg];
        xels[base+2].id = DXL_ID_TIBIA[leg];

        xels[base+0].p_data = goalPositionData[base+0];
        xels[base+1].p_data = goalPositionData[base+1];
        xels[base+2].p_data = goalPositionData[base+2];
    }
    for (int i = 0; i < NUM_LEGS; i++) {
        dxl.torqueOff(DXL_ID_COXA[i]);
        dxl.torqueOff(DXL_ID_FEMUR[i]);
        dxl.torqueOff(DXL_ID_TIBIA[i]);

        dxl.setOperatingMode(DXL_ID_COXA[i], OP_POSITION);
        dxl.setOperatingMode(DXL_ID_FEMUR[i], OP_POSITION);
        dxl.setOperatingMode(DXL_ID_TIBIA[i], OP_POSITION);

        dxl.torqueOn(DXL_ID_COXA[i]);
        dxl.torqueOn(DXL_ID_FEMUR[i]);
        dxl.torqueOn(DXL_ID_TIBIA[i]);
    }
}

void saveCalibrationOffsets() {
    for (int i = 0; i < 18; i++) {
        EEPROM.write(i, calibrationOffsets[i]); // Write each offset to EEPROM
    }
    EEPROM.commit(); // Save changes to flash memory
    Serial.println("Calibration Offsets saved to EEPROM.");
}

void loadCalibrationOffsets() {
    Serial.println("Calibration Offsets loaded from EEPROM.");
    for (int i = 0; i < 18; i++) {
        calibrationOffsets[i] = EEPROM.read(i); // Read each offset from EEPROM
    }
}

Vector3 convertLocalLegPointToGlobal(Vector3 localLegPoint, int legIndex){
    // Assumes globalLegPlacementRadians are angles CCW from Global +X (Right)
    // Assumes Local Frame: +x Left (relative to leg), +y Outward (along leg)
    // Assumes Global Frame: +X Right, +Y Forward

    float distanceFromCenterToLegBase = 110;
    if (legIndex == 1 || legIndex == 4) {
        distanceFromCenterToLegBase = 95;
    }
    // Angle relative to Global +X (Right)
    float theta = globalLegPlacementRadians[legIndex];
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);

    // 1. Calculate base position using standard polar to Cartesian
    float baseX = distanceFromCenterToLegBase * cosTheta;
    float baseY = distanceFromCenterToLegBase * sinTheta;

    // 2. Transform local point to global frame offset
    // Global offset = localX * (Unit vector for local +x) + localY * (Unit vector for local +y)
    // Unit vector for local +y (Outward) is (cosTheta, sinTheta)
    // Unit vector for local +x (Left) is (-sinTheta, cosTheta)
    float rotatedRelX = localLegPoint.x * (-sinTheta) + localLegPoint.y * cosTheta;
    float rotatedRelY = localLegPoint.x * cosTheta + localLegPoint.y * sinTheta;

    // 3. Add base position to the rotated local offset
    float globalX = baseX + rotatedRelX;
    float globalY = baseY + rotatedRelY;
    float globalZ = localLegPoint.z; // Z remains the same

    return Vector3(globalX, globalY, globalZ);
}

Vector3 convertGlobalLegPointToLocal(Vector3 globalPoint, int legIndex){
    // Assumes globalLegPlacementRadians are angles CCW from Global +X (Right)
    // Assumes Local Frame: +x Left (relative to leg), +y Outward (along leg)
    // Assumes Global Frame: +X Right, +Y Forward

    float distanceFromCenterToLegBase = 110;
    if (legIndex == 1 || legIndex == 4) {
        distanceFromCenterToLegBase = 95;
    }
    // Angle relative to Global +X (Right)
    float theta = globalLegPlacementRadians[legIndex];
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);

    // 1. Calculate base position
    float baseX = distanceFromCenterToLegBase * cosTheta;
    float baseY = distanceFromCenterToLegBase * sinTheta;

    // 2. Calculate vector from base to global point in global frame
    float relX_global = globalPoint.x - baseX;
    float relY_global = globalPoint.y - baseY;

    // 3. Project the relative global vector onto the local axes
    // localX = Dot product of rel_global with local +x unit vector (-sin, cos)
    // localY = Dot product of rel_global with local +y unit vector (cos, sin)
    float localX = relX_global * (-sinTheta) + relY_global * cosTheta;
    float localY = relX_global * cosTheta + relY_global * sinTheta;
    float localZ = globalPoint.z; // Z remains the same

    return Vector3(localX, localY, localZ);
}

void updateRuntimeVariables() {
    /*int leftPot = -160;
    int targetDistanceFromGround = leftPot - distanceFromHexapodBottomToFemurJoint;
    float distanceFromGround = distanceFromGround + smoothingFactor * (targetDistanceFromGround - distanceFromGround);*/

    //cGait = getGait((GaitType)TRI);

    //gyroTilt = tilt;
}

// Inverse Body Kinematics
Vector3 bodyIK(Vector3 p, float roll, float pitch) {
    Vector3 out = p;
    // Roll
    out.z += p.y * sin(roll);
    // Pitch
    out.z -= p.x * sin(pitch);

    return out;
}

