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
#include <thread>
#include <iostream>
#include "Motor.hpp"

#include <unistd.h>

using namespace std;

#define xSideFactor 1 // 1 - Left side of the camera under zero, right is over zero, -1 - vice versa
#define ySideFactor 1 // 1 - Upper side of the camera under zero, lower is over zero, -1 - vice versa

#define kMotorStepsPerRevolution k17HS4401MotorStepsPerRevolution
#define kWheelsRadiusInMeters 0.05
#define kDistanceBetweenWheelsInMeters 0.15
#define kMinDistanceForRotationInMeters 2.0
#define kMotorStepInMetersCalibFactor 1

#define kLeftMotorPin1 0
#define kRightMotorPin1 1

#define kGoSpeedInMeterPerSec 0.05
#define kRotSpeedInMeterPerSec 0.01

void MotionController::setup() {

    sharedNewMotion = false;
    sharedMotionInProcess = false;
    
    motorSetup();
}

void MotionController::motorSetup() {
    
    motorStepInMeters = (2 * M_PI * kWheelsRadiusInMeters / kMotorStepsPerRevolution) * kMotorStepInMetersCalibFactor;
    
    machineTurningCircleLength = M_PI * kDistanceBetweenWheelsInMeters;
    
    goDelay = motorStepInMeters / kGoSpeedInMeterPerSec;
    rotDelay = motorStepInMeters / kRotSpeedInMeterPerSec;
}

void MotionController::shouldMove(MotionVector motionVector) {
    
    sharedNewMotion = true;
    
    while (true) {
        
        if (sharedNewMotion && !sharedMotionInProcess) {
            
            sharedNewMotion = false;
            move(motionVector);
            break;
        }
    }
}

void MotionController::move(MotionVector motionVector) {
    
    m.lock();
    
    sharedMotionInProcess = true;
    
    Utils::printMessage("Moving started");
    
    // TODO need trajectory approximator?
    if (motionVector.angleInDegrees >= 1)
        rotate(motionVector.angleInDegrees);

    if (!sharedNewMotion) // if no new motion while rotation being processed
        go(motionVector.distanceInMeters);
    
    Utils::printMessage("Moving finished");

    sharedMotionInProcess = false;
    
    m.unlock();
}

void MotionController::rotate(double angleInDegrees) {
    
    double turningSegmentOfCicleLength = machineTurningCircleLength * angleInDegrees / 360.0f;
    int stepsNum = round(turningSegmentOfCicleLength / motorStepInMeters);

    int directionFactor = xSideFactor * angleInDegrees / fabs(angleInDegrees);
    int leftMotorDirectionFactor = directionFactor;
    int rightMotorDirectionFactor = -directionFactor;
    
    for (int i = 0; i < stepsNum; i++) {
        
        if (sharedNewMotion)
            break;
        
        stepLeftMotor(leftMotorDirectionFactor);
        stepRightMotor(rightMotorDirectionFactor);
        usleep(30000);
        //usleep(rotDelay);
    }
}

void MotionController::go(double distanceInMeters) {
    
    int stepsNum = round(distanceInMeters / motorStepInMeters);
    
    int directionFactor = distanceInMeters / fabs(distanceInMeters);
    
    for (int i = 0; i < abs(stepsNum); i++) {
        
        if (sharedNewMotion)
            break;
        
        stepLeftMotor(directionFactor);
        stepRightMotor(directionFactor);
        usleep(30000);
        //usleep(goDelay);
    }
}

#pragma mark - Motors

void MotionController::stepLeftMotor(int direction) {
    
    stepMotor(kLeftMotorPin1, direction);
}

void MotionController::stepRightMotor(int direction) {
    
    stepMotor(kRightMotorPin1, direction);
}

void MotionController::stepMotor(int motorPin, int direction) {
    
    //direction > 0 ? digitalWrite(motorPin, HIGH) : digitalWrite(motorPin, LOW); // TODO
}

#pragma mark - Helpers

MotionVector MotionController::convertCoordinateToMotionVector(std::vector<double> coordinate) {
    
    double x = coordinate[0];
    double z = coordinate[2];
    
    double distance = sqrt(pow(x, 2) + pow(z, 2));
    
    distance *= z / fabs(z); // consider direction sign
    
    double angle = RADIANS_TO_DEGREES(acos(fabs(z / distance)));
    
    angle *= xSideFactor * x / fabs(x); // consider angle sign
    
    return {angle, distance};
}

bool MotionController::compareMotionVectors(MotionVector mv1, MotionVector mv2) {
    
    return mv1.angleInDegrees == mv2.angleInDegrees && mv1.distanceInMeters == mv2.distanceInMeters;
}
