//
//  MotionController.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 26/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "MotionController.hpp"
#include "../Utils/Utils.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <unistd.h>

using namespace std;

#define xSideFactor 1 // 1 - Left side of the camera under zero, right is over zero, -1 - vice versa
#define ySideFactor 1 // 1 - Upper side of the camera under zero, lower is over zero, -1 - vice versa

#define kMotorStepsPerRevolution k17HS4401MotorStepsPerRevolution
#define kWheelsRadiusInMeters 0.05
#define kDistanceBetweenWheelsInMeters 0.15
#define kMinDistanceForRotationInMeters 2.0
#define kMotorStepInMetersCalibFactor 1

#define kGoSpeedInMeterPerSec 0.05
#define kRotSpeedInMeterPerSec 0.01

#define kLeftMotorEnablePin 16
#define kLeftMotorStepPin 20
#define kLeftMotorDirPin 21
#define kRightMotorEnablePin 13
#define kRightMotorStepPin 19
#define kRightMotorDirPin 26

void MotionController::setup() {

    sharedNewMotion = false;
    sharedMotionInProcess = false;
    
    motorsSetup();
}

void MotionController::motorsSetup() {
    
    motorStepInMeters = (2 * M_PI * kWheelsRadiusInMeters / kMotorStepsPerRevolution) * kMotorStepInMetersCalibFactor;
    
    machineTurningCircleLength = M_PI * kDistanceBetweenWheelsInMeters;
    
    goDelayInMicroSec = motorStepInMeters / kGoSpeedInMeterPerSec * 1000000;
    rotDelayInMicroSec = motorStepInMeters / kRotSpeedInMeterPerSec * 1000000;
    
    leftMotor = Motor(kLeftMotorEnablePin, kLeftMotorStepPin, kLeftMotorDirPin);
    rightMotor = Motor(kRightMotorEnablePin, kRightMotorStepPin, kRightMotorDirPin);
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
    
    Utils::printMessage("Motion started");

    leftMotor.enable();
    rightMotor.enable();
    
    if (fabs(motionVector.angleInDegrees) > 1) {
        Utils::printMessage("Rotation at the start at " + to_string(motionVector.angleInDegrees) + " degrees");
        rotate(motionVector.angleInDegrees);
    }
    
    if (!sharedNewMotion) { // if no new motion while rotation being processed
        
        Utils::printMessage("Movement of " + to_string(motionVector.distanceInMeters) + " meters");
        go(motionVector.distanceInMeters);
    }
    
    if (!sharedNewMotion) { // if no new motion while moving being processed
        
        if (fabs(motionVector.angleInDegrees) > 1) {
            Utils::printMessage("Rotation at the end at -" + to_string(motionVector.angleInDegrees) + " degrees");
            rotate(-motionVector.angleInDegrees);
        }
    }
    
    leftMotor.disable();
    rightMotor.disable();
    
    Utils::printMessage("Motion finished");
    
    sharedMotionInProcess = false;
    
    m.unlock();
}

void MotionController::rotate(double angleInDegrees) {
    
    double turningSegmentOfCicleLength = machineTurningCircleLength * angleInDegrees / 360.0f;
    int stepsNum = round(turningSegmentOfCicleLength / motorStepInMeters);
    
    int directionFactor = xSideFactor * angleInDegrees / fabs(angleInDegrees);
    
    Utils::printMessage("Rotate " + to_string(stepsNum) + " steps in " + (directionFactor > 0 ? "right" : "left") + " direction");

    leftMotor.setDirection(directionFactor);
    rightMotor.setDirection(-directionFactor);
    
    for (int i = 0; i < stepsNum; i++) {
        
        if (sharedNewMotion) {
            Utils::printMessage("Break rotation due to new motion");
            return;
        }
        
        leftMotor.step();
        rightMotor.step();
        usleep(rotDelayInMicroSec);
    }
    
    Utils::printMessage("Rotation completed");
}

void MotionController::go(double distanceInMeters) {
    
    int stepsNum = round(distanceInMeters / motorStepInMeters);
    
    int directionFactor = distanceInMeters / fabs(distanceInMeters);
    
    Utils::printMessage("Move " + to_string(stepsNum) + " steps in " + (directionFactor > 0 ? "forward" : "backward") + " direction");
    
    leftMotor.setDirection(directionFactor);
    rightMotor.setDirection(-directionFactor);
    
    for (int i = 0; i < abs(stepsNum); i++) {
        
        if (sharedNewMotion) {
            Utils::printMessage("Break moving due to new motion");
            return;
        }
        
        leftMotor.step();
        rightMotor.step();
        usleep(goDelayInMicroSec);
    }
    
    Utils::printMessage("Moving completed");
}

#pragma mark - Helpers

MotionVector MotionController::convertCoordinateToMotionVector(std::vector<double> coordinate) {
    
    double x = coordinate[0];
    double z = coordinate[2];
    
    double distance = 0.0;
    distance = sqrt(pow(x, 2) + pow(z, 2));
    distance *= z / fabs(z); // consider direction sign
    
    double angle = 0.0;
    angle = RADIANS_TO_DEGREES(acos(fabs(z / distance)));
    
    if (distance < 0)
        angle = 180 - angle;
    
    angle *= xSideFactor * x / fabs(x); // consider angle sign
    
    return {fabs(angle) > __DBL_EPSILON__ ? angle : 0.0, fabs(distance) > __DBL_EPSILON__ ? distance : 0.0};
}

bool MotionController::compareMotionVectors(MotionVector mv1, MotionVector mv2) {
    
    return mv1.angleInDegrees == mv2.angleInDegrees && mv1.distanceInMeters == mv2.distanceInMeters;
}
