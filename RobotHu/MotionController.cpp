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
#include "wiringPi.h"


#include <unistd.h>

using namespace std;

#define xSideFactor 1 // 1 - Left side of the camera under zero, right is over zero, -1 - vice versa
#define ySideFactor 1 // 1 - Upper side of the camera under zero, lower is over zero, -1 - vice versa

#define kMovingMonitorFreqInmSec 100
#define kMotorStepsPerRevolution 200
#define kWheelsRadiusInMeters 0.05
#define kDistanceBetweenWheelsInMeters 0.15
#define kMinDistanceForRotationInMeters 2.0
#define kMotorStepInMetersCalibFactor 1

#define kLeftMotorPin1 0
#define kRightMotorPin1 1

#define kGoSpeedInMeterPerSec 0.05
#define kRotSpeedInMeterPerSec 0.01

void MotionController::setup() {

    newMotion = false;
    motionInProcess = false;
    
    motorSetup();
    
    thread t1(&MotionController::startMovingMonitor, this);
    t1.detach();
}

void MotionController::startMovingMonitor() {
    
    chrono::milliseconds interval(kMovingMonitorFreqInmSec);
    
    while (true) {
        
        if (newMotion && !motionInProcess) {
            
            newMotion = false;
            move();
        }
        
        this_thread::sleep_for(interval);
    }
}

void MotionController::setMotionVector(MotionVector _motionVector) {
    
    motionVector = _motionVector;
    newMotion = true;
}

void MotionController::move() {
    
    // TODO Need lock mutex here
    
    motionInProcess = true;
    
    Utils::printMessage("Moving started");
    
    // TODO need trajectory approximator
    if (motionVector.angleInDegrees >= 1) {

        rotate(motionVector.angleInDegrees);
    }

    go(motionVector.distanceInMeters);
    
    motionInProcess = false;
    
    Utils::printMessage("Moving finished");
    
    // TODO Need unlock mutex here
}

void MotionController::rotate(double angleInDegrees) {
    
    double turningSegmentOfCicleLength = machineTurningCircleLength * angleInDegrees / 360.0f;
    int stepsNum = round(turningSegmentOfCicleLength / motorStepInMeters);

    int directionFactor = xSideFactor * angleInDegrees / fabs(angleInDegrees);
    int leftMotorDirectionFactor = directionFactor;
    int rightMotorDirectionFactor = -directionFactor;
    
    for (int i = 0; i < stepsNum; i++) {
        
//        if (newMotion)
//            break; // TODO
        
        stepLeftMotor(leftMotorDirectionFactor);
        stepRightMotor(rightMotorDirectionFactor);
        //usleep(300000);
        //delay(rotDelay);
    }
}

void MotionController::go(double distanceInMeters) {
    
    int stepsNum = round(distanceInMeters / motorStepInMeters);
    
    int directionFactor = distanceInMeters / fabs(distanceInMeters);
    
    for (int i = 0; i < stepsNum; i++) {
        
//        if (newMotion)
//            break; // TODO
        
        stepLeftMotor(directionFactor);
        stepRightMotor(directionFactor);
        //usleep(300000);
        //delay(goDelay);
    }
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

#pragma mark - Motors

void MotionController::testMotion() {
    
    for (int i = 0; i < 100; i++) {
        
        stepLeftMotor(1);
        stepRightMotor(1);
        //delay(goDelay);
    }
}

void MotionController::motorSetup() {
    
    motorStepInMeters = 2 * M_PI * kWheelsRadiusInMeters / kMotorStepsPerRevolution;
    motorStepInMeters *= kMotorStepInMetersCalibFactor;
    
    machineTurningCircleLength = M_PI * kDistanceBetweenWheelsInMeters;
    
    //wiringPiSetup ();
    //pinMode(kLeftMotorPin1, OUTPUT);
    //pinMode(kRightMotorPin1, OUTPUT);
    
    goDelay = motorStepInMeters / kGoSpeedInMeterPerSec;
    rotDelay = motorStepInMeters / kRotSpeedInMeterPerSec;
}

void MotionController::stepLeftMotor(int direction) {
    
    stepMotor(kLeftMotorPin1, direction);
}

void MotionController::stepRightMotor(int direction) {
    
    stepMotor(kRightMotorPin1, direction);
}

void MotionController::stepMotor(int motorPin, int direction) {
    
    //direction > 0 ? digitalWrite(motorPin, HIGH) : digitalWrite(motorPin, LOW);
}
