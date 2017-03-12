//
//  MotionController.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 26/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "MotionController.hpp"
#include "config.h"
#include "../Utils/Utils.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <unistd.h>

using namespace std;

void MotionController::setup() {
    
    newMotionShared = false;
    motionInProcessShared = false;
    
    goSpeedInMeterPerSec = kDefaultGoSpeedInMeterPerSec;
    
    motorsSetup();
}

void MotionController::motorsSetup() {
    
    rotSpeedInMeterPerSec = goSpeedInMeterPerSec / 2.0;

    motorStepInMeters = (2 * M_PI * kWheelsRadiusInMeters / kMotorStepsPerRevolution) * kMotorStepInMetersCalibFactor;
    
    machineTurningCircleLength = M_PI * kDistanceBetweenWheelsInMeters;
    
    goDelayInMicroSec = motorStepInMeters / goSpeedInMeterPerSec * 1000000;
    rotDelayInMicroSec = motorStepInMeters / rotSpeedInMeterPerSec * 1000000;
    
    leftMotor = Motor(kLeftMotorEnablePin, kLeftMotorStepPin, kLeftMotorDirPin);
    rightMotor = Motor(kRightMotorEnablePin, kRightMotorStepPin, kRightMotorDirPin);
}

void MotionController::setSpeed(double speedInMeterPerSec) {
    
    goSpeedInMeterPerSec = speedInMeterPerSec;
    
    motorsSetup();
}

void MotionController::shouldMove(MotionVector motionVector) {
    
    Utils::printMotionVector(motionVector);
    newMotionShared = true;
    
    while (true) {
        
        if (newMotionShared && !motionInProcessShared) {
            
            newMotionShared = false;
            move(motionVector);
            break;
        }
    }
}

void MotionController::move(MotionVector motionVector) {
    
    m.lock();
    
    motionInProcessShared = true;
    
    Utils::printMessage("Motion started");

    leftMotor.enable();
    rightMotor.enable();
    
    if (fabs(motionVector.xzCorrectionAngleInDeg) > 1) {
        Utils::printMessage("XZ correction rotation at " + to_string(motionVector.xzCorrectionAngleInDeg) + " degrees");
        rotate(motionVector.xzCorrectionAngleInDeg);
    }
    
    if (fabs(motionVector.angleInDeg) > 1) {
        Utils::printMessage("Initial rotation at " + to_string(motionVector.angleInDeg) + " degrees");
        rotate(motionVector.angleInDeg);
    }
    
    if (!newMotionShared) { // if no new motion while rotation being processed
        
        Utils::printMessage("Movement of " + to_string(motionVector.distanceInMeters) + " meters");
        go(motionVector.distanceInMeters);
    }
    
    if (!newMotionShared) { // if no new motion while moving being processed
        
        if (fabs(motionVector.angleInDeg) > 1) {
            Utils::printMessage("Reversed rotation at " + to_string(motionVector.angleInDeg) + " degrees");
            rotate(-motionVector.angleInDeg);
        }
    }
    
    leftMotor.disable();
    rightMotor.disable();
    
    Utils::printMessage("Motion finished");
    
    motionInProcessShared = false;
    
    m.unlock();
}

void MotionController::rotate(double angleInDegrees) {
    
    double turningSegmentOfCicleLength = machineTurningCircleLength * angleInDegrees / 360.0f;
    int stepsNum = round(turningSegmentOfCicleLength / motorStepInMeters);
    
    int directionFactor = angleInDegrees / fabs(angleInDegrees);
    
    Utils::printMessage("Rotate " + to_string(stepsNum) + " steps in " + (directionFactor > 0 ? "right" : "left") + " direction");

    leftMotor.setDirection(directionFactor);
    rightMotor.setDirection(directionFactor);
    
    for (int i = 0; i < stepsNum; i++) {
        
        if (newMotionShared) {
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
        
        if (newMotionShared) {
            Utils::printMessage("Break moving due to new motion");
            return;
        }
        
        leftMotor.step();
        rightMotor.step();
        usleep(goDelayInMicroSec);
    }
    
    Utils::printMessage("Moving completed");
}
