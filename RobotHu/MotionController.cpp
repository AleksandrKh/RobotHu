//
//  MotionController.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 26/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "MotionController.hpp"
#include <cmath>
#include "Utils.hpp"
//#include "wiringPi.h"

#define xSideFactor 1 // 1 - Left side of the camera under zero, right is over zero, -1 - vice versa
#define ySideFactor 1 // 1 - Upper side of the camera under zero, lower is over zero, -1 - vice versa

#define kMotorStepsPerRevolution 200
#define kWheelsRadiusInMeters 0.05
#define kDistanceBetweenWheelsInMeters 0.15

#define kMotorStepInMetersCalibFactor 1

MotionController::MotionController() {

    motorStepInMeters = 2 * M_PI * kWheelsRadiusInMeters / kMotorStepsPerRevolution;
    machineTurningCircleLength = M_PI * kDistanceBetweenWheelsInMeters;
    
    calibrate();
    
    //wiringPiSetup ();
}

void MotionController::calibrate() {
    
    motorStepInMeters *= kMotorStepInMetersCalibFactor;
}

void MotionController::move(MotionVector motionVector) {
    
    // TODO think if need rot or not
    // TODO check if previous moving in process need singleton
    
    if (motionVector.angleInDegrees >= 1)
        rotate(motionVector.angleInDegrees);
    
    go(motionVector.distanceInMeters);
}

void MotionController::rotate(double angleInDegrees) {
    
    double turningSegmentOfCicleLength = machineTurningCircleLength * angleInDegrees / 360.0f;
    int stepsNum = round(turningSegmentOfCicleLength / motorStepInMeters);

    int sideFactor = xSideFactor * angleInDegrees / fabs(angleInDegrees);
    int leftMotorStepsNum = stepsNum * sideFactor;
    int rightMotorStepsNum = stepsNum * (-sideFactor);
    
    //digitalWrite(1, 1);
    
    // SIGN MOTORS
}

void MotionController::go(double distanceInMeters) {
    
    int stepsNum = round(distanceInMeters / motorStepInMeters);
    
    // SIGN MOTORS
}

MotionVector MotionController::convertCoordinateToMotionVector(std::vector<double> coordinate) {
    
    double x = coordinate[0];
    double z = coordinate[2];
    
    double distance = sqrt(pow(x, 2) + pow(z, 2));
    
    distance *= z / fabs(z); // consider direction sign
    
    double angle = RADIANS_TO_DEGREES(acos(fabs(z / distance)));
    
    angle *= xSideFactor * x / fabs(x); // consider angle sign
    
    return {angle, distance};
}
