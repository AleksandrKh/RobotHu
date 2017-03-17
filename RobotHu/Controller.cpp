//
//  Controller.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "Controller.hpp"
#include "config.h"
#include "../Utils/Utils.hpp"
#include <iostream>
#include <thread>
#include <cmath>

using namespace std;

#define kPoseAnalyzerFreqInMilliSec 1000

Controller::Controller() {
    
    lastPose = {0, 0, 0};
    isPoseUpdated = false;
}

void Controller::start(double holdingPoseDistanceInMeters, double speedInMeterPerSec) {
    
    this->holdingPoseDistanceInMeters = holdingPoseDistanceInMeters;
    MotionController::Instance().setSpeed(speedInMeterPerSec);

    // Delegates
    function<void(PoseVector)> didObtainPoseDelegate = [=](PoseVector pose) {
        this->didObtainPoseDelegate(pose);
    };
    PoseEstimator::Instance().didObtainPoseDelegate = &didObtainPoseDelegate;
    
    function<void(string)> didReceiveErrorMessage = [=](string errorMessage) {
        this->didReceiveErrorMessage(errorMessage);
    };
    PoseEstimator::Instance().didReceiveErrorMessage = &didReceiveErrorMessage;
    
    function<void()> didLostPoseDelegate = [=]() {
        this->didLostPoseDelegate();
    };
    PoseEstimator::Instance().didLostPoseDelegate = &didLostPoseDelegate;
    
    thread t1(&Controller::startPoseEstimator, this);
    thread t2(&Controller::startPoseAnalyzer, this);
    
    t1.join();
    t2.join();
}

#pragma mark - PoseEstimator

// Analyzes pose and returns delegate

void Controller::startPoseEstimator() {
    
    Utils::printMessage("Start pose estimator");

    PoseEstimator::Instance().start();
}

#pragma mark - PoseEstimator delegates

void Controller::didObtainPoseDelegate(PoseVector pose) {
    
    Utils::printMessage("New pose obtained from estimator");
    Utils::printPoseVector(pose);
    
    lastPose = pose;
    m.lock();
    isPoseUpdated = true;
    m.unlock();
}

void Controller::didLostPoseDelegate() {
    
}

void Controller::didReceiveErrorMessage(string errorMessage) {
    
    Utils::printError(errorMessage);
}

#pragma mark - Pose analyzer

// Analyzing and filtration "dirty" pose data from OpenCV

void Controller::startPoseAnalyzer() {

    Utils::printMessage("Start pose analyzer");

    chrono::milliseconds interval(kPoseAnalyzerFreqInMilliSec);
    
    while (true) {
        
        if (!isPoseUpdated)
            continue;
        
        m.lock();
        isPoseUpdated = false;
        m.unlock();
        
        if (filterPose(lastPose)) {
            
            MotionVector motion = convertPoseToMotion(lastPose);
            
            Utils::printMessage("New motion");
            Utils::printMotionVector(motion);
            
            function<void()> move = [=]() {
                MotionController::Instance().shouldMove(motion);
            };
            
            thread t(move);
            t.detach();
        }
        
        this_thread::sleep_for(interval);
    }
}

bool Controller::filterPose(PoseVector pose) {
    
    // Check min deviation from last pose
    if (fabs(pose.xDistanceInMeters - lastPose.xDistanceInMeters) > kMinXDistanceDeviationInMeters ||
        fabs(pose.zDistanceInMeters - lastPose.zDistanceInMeters) > kMinZDistanceDeviationInMeters) {
         
        return true;
    }
    
    return false;
}

MotionVector Controller::convertPoseToMotion(PoseVector pose) {
    
    MotionVector motion;
    
    motion.xzAngleInDeg = pose.xzAngleInDeg;

    double holdingDistance = pose.zDistanceInMeters - holdingPoseDistanceInMeters;
    motion.distanceInMeters = sqrt(pow(pose.xDistanceInMeters, 2) + pow(holdingDistance, 2));
    motion.distanceInMeters *= holdingDistance / fabs(holdingDistance);
    
    motion.angleInDeg = -RADIANS_TO_DEGREES(atan(fabs(pose.xDistanceInMeters / holdingDistance)));
    
    if (pose.xDistanceInMeters)
        motion.angleInDeg *= pose.xDistanceInMeters / fabs(pose.xDistanceInMeters);
    if (holdingDistance)
        motion.angleInDeg *= holdingDistance / fabs(holdingDistance);
    
    motion.angleInDeg *= xSideFactor;

    return motion;
}

#pragma mark - Test

// For testing motion behaviour without camera data

void Controller::startTest(vector<PoseVector> poses, double holdingPoseDistance, double speedInMeterPerSec) {
    
    this->holdingPoseDistanceInMeters = holdingPoseDistance;
    MotionController::Instance().setSpeed(speedInMeterPerSec);
    
    thread t1(&Controller::startPoseAnalyzer, this);

    function<void()> startTestPoseGenerator = [=]() {
        
        Utils::printMessage("Start test pose generator");
        
        chrono::seconds interval(5);
        
        //    poses = {
        //        // redefine
        //    };
        
        for (int i = 0; i < poses.size(); i++) {
            
            Utils::printMessage("Emit a new test pose");
            Utils::printPoseVector(poses[i]);
            
            didObtainPoseDelegate(poses[i]);
            
            this_thread::sleep_for(interval);
        }
    };
    
    thread t2(startTestPoseGenerator);

    t1.join();
    t2.join();
}
