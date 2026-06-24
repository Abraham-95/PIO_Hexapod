#include "Gyro.h"

BNO080 myIMU;
float roll = 0; float pitch = 0; float yaw = 0;
float pitchFiltered = 0; float rollFiltered = 0;

float gyroDt = 0;
uint32_t lastGyroTime = 0;

float comPosX = 0; float comVelX = 0;
float comPosY = 0; float comVelY = 0;
float zmpX = 0; float zmpY = 0;

float prevComPosX = 0; float prevComPosY = 0;
float prevComVelX = 0; float prevComVelY = 0;

BalanceOutput balanceOutput;

void setupGyro() {
  delay(100);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Start BNO080 at 0x4B
  if (!myIMU.begin(BNO_ADDR, Wire)) {
    while (1) delay(10);
  }
  // Enable Euler angles (via Rotation Vector)
  myIMU.enableRotationVector(50); // 50ms (20Hz)
}

void gyroUpdate() {
  if (myIMU.dataAvailable()) {
    // Convert radians to degrees & switch Roll & Pitch to match body axes
    pitch = -myIMU.getRoll()  * 180.0f / PI;
    roll  = myIMU.getPitch() * 180.0f / PI;
    yaw   = myIMU.getYaw()   * 180.0f / PI;
  }
  pitchFiltered += 0.15f * (pitch - pitchFiltered);
  rollFiltered += 0.15f * (roll - rollFiltered);

  rc_control_data.gyro_X = (int16_t)rollFiltered;
  rc_control_data.gyro_Y = (int16_t)pitchFiltered;
  rc_control_data.gyro_Z = (int16_t)yaw;

  uint32_t now = millis();
  gyroDt = (now - lastGyroTime) * 0.001f;
  lastGyroTime = now;

  if(gyroDt <= 0 || gyroDt > 0.05f) gyroDt = 0.01f;
}

void stabilization(BalanceMode mode) {
  // Stabilization Tuning Parameters
  float deadband, Kp, Kd, pitchGain, rollGain, shiftGain;

  switch(mode) {
    case STANDING_MODE:
      deadband  = 1.5f;
      Kp = 0.15f; Kd = 0.05f;
      pitchGain = 0.8f; rollGain  = 0.3f; shiftGain = 1.0f;
      break;
    case WALKING_MODE:
      deadband  = 3.0f;
      Kp = 0.08f; Kd = 0.03f;
      pitchGain = 0.4f; rollGain  = 0.15f; shiftGain = 0.30f;
      break;
    default:
      deadband  = 1.5f;
      Kp = 0.15f; Kd = 0.05f;
      pitchGain = 0.8f; rollGain  = 0.3f; shiftGain = 1.0f;
      break;
  }
  // IMU Compensation
  float pitchComp = constrain(pitchFiltered, -45.0f, 45.0f);
  float rollComp  = constrain(rollFiltered , -45.0f, 45.0f);

  if(abs(pitchComp) <= deadband) pitchComp = 0;
  if(abs(rollComp)  <= deadband) rollComp = 0;

  // Center of Mass (COM) Estimation
  float bodyHeight = abs(distanceFromGround);
  comPosX = constrain(bodyHeight * radians(pitchComp), -30, 30);
  comPosY = constrain(bodyHeight * radians(rollComp), -30, 30);

  comVelX = (comPosX - prevComPosX) / gyroDt;
  comVelY = (comPosY - prevComPosY) / gyroDt;
  prevComPosX = comPosX; prevComPosY = comPosY;

  static float velFilteredX = 0; static float velFilteredY = 0;
  velFilteredX = 0.8f * velFilteredX + 0.2f * comVelX;
  velFilteredY = 0.8f * velFilteredY + 0.2f * comVelY;
  comVelX = velFilteredX; comVelY = velFilteredY;

  // Support Polygon
  Vector2 supportCenter;
  supportCenter.x=0; supportCenter.y=0;

  for(int i=0;i<6;i++) {
    supportCenter.x += currentLegPositions[i].x;
    supportCenter.y += currentLegPositions[i].y;
  }
  const float nominalCenterY = 65.0f;
  supportCenter.x = supportCenter.x / 6.0f;
  supportCenter.y = (supportCenter.y / 6.0f) - nominalCenterY;

  // ZMP: Zero Moment Point
  zmpX = comPosX;
  zmpY = comPosY;

  // PD Controller
  float errorX = zmpX - supportCenter.x;
  float errorY = zmpY - supportCenter.y;

  if(abs(errorX) < 1.0f) errorX = 0;
  if(abs(errorY) < 1.0f) errorY = 0;

  balanceOutput.xShift = constrain(Kp * errorX + Kd * comVelX * shiftGain, -20, 20);
  balanceOutput.yShift = constrain(Kp * errorY + Kd * comVelY * shiftGain, -20, 20);

  // Body Leveling
  balanceOutput.pitchOffset = -radians(pitchComp * pitchGain);
  balanceOutput.rollOffset  = -radians(rollComp * rollGain);

  #if DEBUG_GYRO
  static uint32_t lastPrint = 0;
  if(millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.println("\n========= BALANCE DEBUG =========");
    Serial.print("Pitch : "); Serial.print(pitchFiltered, 2);
    Serial.print("\tRoll : "); Serial.println(rollFiltered, 2);
    Serial.print("COM X : "); Serial.print(comPosX, 2);
    Serial.print("\tCOM Y : "); Serial.println(comPosY, 2);
    Serial.print("COM Vel X : "); Serial.print(comVelX, 2);
    Serial.print("\tCOM Vel Y : "); Serial.println(comVelY, 2);
    Serial.print("ZMP X : "); Serial.print(zmpX, 2);
    Serial.print("\tZMP Y : "); Serial.println(zmpY, 2);
    Serial.print("Support Center X : "); Serial.print(supportCenter.x, 2);
    Serial.print("\tSupport Center Y : "); Serial.println(supportCenter.y, 2);
    Serial.print("Body Shift X : "); Serial.print(balanceOutput.xShift, 2);
    Serial.print("\tBody Shift Y : "); Serial.println(balanceOutput.yShift, 2);
    Serial.print("Pitch Offset : "); Serial.print(degrees(balanceOutput.pitchOffset), 2);
    Serial.print("\tRoll Offset : "); Serial.println(degrees(balanceOutput.rollOffset), 2);
    Serial.println("================================");
 }
#endif
}




