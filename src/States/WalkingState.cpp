#include <Dynamixel2Arduino.h>
#include "State.h"
#include "Com.h"
#include "LegManager.h"
#include "Gait.h"

void WalkingState::init() {
  Serial.println("Walking State.");

  cGait = getGait(currentGait);
  forwardAmount = 0; turnAmount = 0;
  joyLCurrentVector = Vector2(0,0); joyRCurrentVector = Vector2(0,0);
  joyLCurrentMagnitude = 0; joyRCurrentMagnitude = 0;

  // Initialize Leg States
  for (int i = 0; i < 6; i++) {
    legStates[i] = Reset;
    cycleProgress[i] = cGait.cycleOffsetPercentages[i] * points;
    tArray[i] = (float)cycleProgress[i] / points;
    cycleStartPoints[i] = currentLegPositions[i];
  }
}

void WalkingState::exit() {
  for (int i = 0; i < 6; i++) {
    legStates[i] = Standing; cycleProgress[i] = 0;
  }
  forwardAmount = 0; turnAmount = 0;
  joyLCurrentVector = Vector2(0,0); joyRCurrentVector = Vector2(0,0);
  joyLCurrentMagnitude = 0; joyRCurrentMagnitude = 0;
}

void WalkingState::loop() {
  double joyLx = rc_control_data.axisX;
  double joyLy = rc_control_data.axisY;
  double joyRx = rc_control_data.axisRX;
  double joyRy = rc_control_data.axisRY;

  /*Serial.print("LX: "); Serial.print(joyLx);
  Serial.print(" LY: "); Serial.print(joyLy);
  Serial.print(" RX: "); Serial.print(joyRx);
  Serial.print(" RY: "); Serial.println(joyRy);*/

  joyLTargetVector = Vector2(joyLx, joyLy);
  joyLTargetMagnitude = constrain(calculateHypotenuse(abs(joyLx), abs(joyLy)), 0, 100);

  joyRTargetVector = Vector2(joyRx, joyRy);
  joyRTargetMagnitude = constrain(calculateHypotenuse(abs(joyRx), abs(joyRy)), 0, 100);

  joyLCurrentVector = lerp(joyLCurrentVector, joyLTargetVector, 0.04);
  joyLCurrentMagnitude = lerp(joyLCurrentMagnitude, joyLTargetMagnitude, 0.04);

  joyRCurrentVector = lerp(joyRCurrentVector, joyRTargetVector, 0.06);
  joyRCurrentMagnitude = lerp(joyRCurrentMagnitude, joyRTargetMagnitude, 0.06);

  bool movingStraight = abs(joyLy) > 10 && abs(joyLx) < 10 &&
                        abs(joyRx) < 10;

  if (movingStraight) {
    if (!headingLock) {lockedYaw = yaw; headingLock = true;}
  } else {headingLock = false;}

  float yawCorrection = 0;
  if (headingLock) {
    float yawError = lockedYaw - yaw;
    while (yawError > 180) yawError -= 360;
    while (yawError < -180) yawError += 360;

    if (abs(yawError) < 1.0f) yawError = 0;
    yawCorrection = constrain(yawError * 0.8f, -15.0f, 15.0f);
  }
  forwardAmount = joyLCurrentVector.y;

  if (headingLock) turnAmount = yawCorrection;
  else turnAmount = joyRCurrentVector.x;

  float potRightPercentage = 0.5; // Adjust this value as needed

  for (int i = 0; i < 6; i++) {
    tArray[i] = (float)cycleProgress[i] / points;
  }

  forwardAmount = joyLCurrentMagnitude;
  turnAmount = joyRCurrentVector.x;
  cGait = getGait(currentGait);

  stabilization(WALKING_MODE);

  for(int i = 0; i < 6; i++) {
    Vector3 gaitPoint = getGaitPoint(i, cGait.pushFraction);
    Vector3 bodyPoint = convertLocalLegPointToGlobal(gaitPoint, i);

    bodyPoint.x -= balanceOutput.xShift;
    bodyPoint.y -= balanceOutput.yShift;
    bodyPoint.z += bodyPoint.x * sin(balanceOutput.rollOffset)
                 - bodyPoint.y * sin(balanceOutput.pitchOffset);

    Vector3 localPoint = convertGlobalLegPointToLocal(bodyPoint, i);

    moveToPos(i, localPoint);
    //moveToPos(i, getGaitPoint(i, cGait.pushFraction));
  }

  if (!isIdle()) {
    float progressChangeAmount = max(abs(forwardAmount), abs(turnAmount))
      * cGait.gaitSpeedMult * globalSpeedMult * potRightPercentage;
    progressChangeAmount = constrain(progressChangeAmount, 0, 80);

    for (int i = 0; i < 6; i++) {cycleProgress[i] += progressChangeAmount;
      if (cycleProgress[i] >= points) cycleProgress[i] -= points;
    }
  }
}

bool WalkingState::isIdle() const {
  constexpr float WALK_IDLE_THRESHOLD = 10.0f;
  return abs(forwardAmount) < WALK_IDLE_THRESHOLD &&
         abs(turnAmount)    < WALK_IDLE_THRESHOLD;
}

Vector3 WalkingState::getGaitPoint(int leg, float pushFraction) {
  float rotateStrideLength = turnAmount * globalRotationStrideLengthMult;

  Vector2 strafeStrideLength = joyLCurrentVector * globalStrafeStrideLengthMult * cGait.strideLengthMult;
  strafeStrideLength.y = constrain(strafeStrideLength.y,-cGait.maxStrideLength/2, cGait.maxStrideLength/2);
  strafeStrideLength.x = constrain(strafeStrideLength.x,-cGait.maxStrideLength, cGait.maxStrideLength);

  float t = tArray[leg];

  //Propelling
  if(t < pushFraction) {
    if(legStates[leg] != Propelling) {
      cycleStartPoints[leg] = currentLegPositions[leg];
    }
    legStates[leg] = Propelling;

    //-----This cycle is a straight line that will cause the hexapod to strafe-----//

    //Starting point of the line
    Vector3 strafeControlPoints[2];
    strafeControlPoints[0] = cycleStartPoints[leg];

    //Ending point of the line
    strafeControlPoints[1] = Vector3(
        strafeStrideLength.y * strideMultiplier[leg],
        -strafeStrideLength.x * strideMultiplier[leg] + distanceFromCenter,
        distanceFromGround
    ).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(0, distanceFromCenter));

    Vector3 strafePoint = GetPointOnBezierCurve(strafeControlPoints, 2, mapFloat(t, 0, pushFraction, 0, 1));
    //-------------------------------------------------------------------------------//

    //Starting point of the curve
    Vector3 rotateControlPoints[3];
    rotateControlPoints[0] = cycleStartPoints[leg];

    //Middle point of the curve
    rotateControlPoints[1] = Vector3(
        0,                  //X
        distanceFromCenter, //Y
        distanceFromGround  //Z
    );

    //Ending point of the curve
    rotateControlPoints[2] = Vector3(
      rotateStrideLength,   //X
      distanceFromCenter,   //Y
      distanceFromGround    //Z
    );

    Vector3 rotatePoint = GetPointOnBezierCurve(rotateControlPoints, 3, mapFloat(t, 0, pushFraction, 0, 1));
    //-------------------------------------------------------------------------------//

    float denom = abs(forwardAmount) + abs(turnAmount);
    if (denom < 1e-3f) {return currentLegPositions[leg];}

    //Return the weighted average of the two points
    return (strafePoint * abs(forwardAmount) + rotatePoint * abs(turnAmount)) / denom;
  }

  //Lifting
  else {
    if(legStates[leg] != Lifting) {cycleStartPoints[leg] = currentLegPositions[leg];}
    legStates[leg] = Lifting;

    //------This cycle will cause the hexapod leg to lift up and return to the beginning of the walk cycle in a straight line-----//

    //Starting point of the curve
    Vector3 strafeControlPoints[4];
    strafeControlPoints[0] = cycleStartPoints[leg];

    //Control point directly above the starting point causing the leg to lift up quickly
    strafeControlPoints[1] = cycleStartPoints[leg] + Vector3(0, 0, cGait.liftHeight * globalLiftHeightMult);

    //Control point directly above the ending point preventing the leg from running into the ground
    strafeControlPoints[2] = Vector3(
      -strafeStrideLength.y * strideMultiplier[leg],                      //X
      strafeStrideLength.x * strideMultiplier[leg] + distanceFromCenter,  //Y
      distanceFromGround + legLandHeight                                  //Z
    ).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(0,distanceFromCenter));

    //Ending point of the curve
    strafeControlPoints[3] = Vector3(
      -strafeStrideLength.y * strideMultiplier[leg],
      strafeStrideLength.x * strideMultiplier[leg] + distanceFromCenter,
      distanceFromGround
    ).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(0,distanceFromCenter));

    Vector3 straightPoint = GetPointOnBezierCurve(strafeControlPoints, 4, mapFloat(t, pushFraction, 1, 0, 1));
    //-------------------------------------------------------------------------------//


    //------This cycle will cause the hexapod leg to lift up and return to the beginning of the walk cycle in a curved line-----//

    //Starting point of the curve
    Vector3 rotateControlPoints[5];
    rotateControlPoints[0] = cycleStartPoints[leg];

    //Control point directly above the starting point causing the leg to lift up quickly
    rotateControlPoints[1] = cycleStartPoints[leg] + Vector3(0, 0, cGait.liftHeight * globalLiftHeightMult);

    //Control point at the apex of the curve and offset away from the hexapods body, cause the leg to lift up and away.
    rotateControlPoints[2] = Vector3(
      0,                                                            //X
      distanceFromCenter,                                           //Y
      distanceFromGround + cGait.liftHeight * globalLiftHeightMult  //Z
    );

    //Control point directly above the ending point preventing the leg from running into the ground
    rotateControlPoints[3] = Vector3(
      -rotateStrideLength,                //X
      distanceFromCenter,                 //Y
      distanceFromGround + legLandHeight  //Z
    );

    //Ending point of the curve
    rotateControlPoints[4] = Vector3(
      -rotateStrideLength,            //X
      distanceFromCenter,             //Y
      distanceFromGround              //Z
    );

    Vector3 rotatePoint = GetPointOnBezierCurve(rotateControlPoints, 5, mapFloat(t, pushFraction, 1, 0, 1));
    //-------------------------------------------------------------------------------//

    //Return the weighted average of the two points
    float denom = abs(forwardAmount) + abs(turnAmount);
    if (denom < 1e-3f) {return currentLegPositions[leg];}
    return (straightPoint * abs(forwardAmount) + rotatePoint * abs(turnAmount)) / denom;
  }
}
