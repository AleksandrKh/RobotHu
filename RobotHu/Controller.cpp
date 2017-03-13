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

#define kPosesListSize 3
#define kPoseListUpdateDelayInSec 0.5
#define kPoseAnalyzerFreqInMilliSec 200
#define kMotionMonitorFreqInSec 3

Controller::Controller() {
    
    analyzedMotionShared = {0, 0, 0};
    lastPose = {0, 0, 0};
    isMotionUpdatedShared = false;
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
    
    thread t1(&Controller::startPoseEstimator, this);
    thread t2(&Controller::startPoseAnalyzer, this);
    thread t3(&Controller::startMotionMonitor, this);
    
    t1.join();
    t2.join();
    t3.join();
}

#pragma mark - PoseEstimator

// Analyzes pose and returns delegate

void Controller::startPoseEstimator() {
    
    Utils::printMessage("Start pose estimator");

    PoseEstimator::Instance().start();
}

#pragma mark - PoseEstimator delegates

void Controller::didObtainPoseDelegate(PoseVector pose) {
    
//     Utils::printMessage("New pose obtained from estimator");
//     Utils::printPoseVector(pose);

    time_t now = time(0);
    
    if (lastPosesListShared.empty()) {
        
        lastPosesListShared.push_back(pose);
        lastAcceptedPoseTime = now;
        return;
    }
    else if (difftime(now, lastAcceptedPoseTime) > kPoseListUpdateDelayInSec) {
    
        if (lastPosesListShared.size() == kPosesListSize)
            lastPosesListShared.pop_front();
        
        lastPosesListShared.push_back(pose);
    }
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
        
        // Get average pose between last poses
        PoseVector avgPose = {0, 0, 0};
        
        for (auto const& i : lastPosesListShared) {
            
            avgPose.xzAngleInDeg += i.xzAngleInDeg;
            avgPose.xDistanceInMeters += i.xDistanceInMeters;
            avgPose.zDistanceInMeters += i.zDistanceInMeters;
        }
        
        double posesListSize = (double)lastPosesListShared.size();
        
        avgPose.xzAngleInDeg /= posesListSize;
        avgPose.xDistanceInMeters /= posesListSize;
        avgPose.zDistanceInMeters /= posesListSize;
        
        if (filterPose(avgPose)) {
            
            analyzedMotionShared = convertPoseToMotion(avgPose);
            
            Utils::printMessage("New target coordinate recalculated");
            Utils::printMotionVector(analyzedMotionShared);
            
            m.lock();
            isMotionUpdatedShared = true;
            m.unlock();
        }

        this_thread::sleep_for(interval);
    }
}

bool Controller::filterPose(PoseVector pose) {
    
    // Check min deviation from last pose
    if (fabs(pose.xzAngleInDeg - lastPose.xzAngleInDeg) > kMinXZAnlgeDeviationInDeg ||
        fabs(pose.xDistanceInMeters - lastPose.xDistanceInMeters) > kMinDistanceDeviationInMeters ||
        fabs(pose.zDistanceInMeters - lastPose.zDistanceInMeters) > kMinDistanceDeviationInMeters) {
        
        lastPose = pose;
 
        return true;
    }
    
    return false;
}

MotionVector Controller::convertPoseToMotion(PoseVector pose) {
    
    MotionVector motion;
    
    motion.xzCorrectionAngleInDeg = pose.xzAngleInDeg;

    double holdingDistance = pose.zDistanceInMeters - holdingPoseDistanceInMeters;
    motion.distanceInMeters = sqrt(pow(pose.xDistanceInMeters, 2) + pow(holdingDistance, 2));
    
    motion.angleInDeg = RADIANS_TO_DEGREES(atan(fabs(holdingDistance / pose.xDistanceInMeters)));
    
    if (holdingDistance < 0) {
        motion.angleInDeg += 90;
    }
    else {
        motion.angleInDeg = 90 - motion.angleInDeg;
    }
    
    motion.angleInDeg *= xSideFactor * pose.xDistanceInMeters / fabs(pose.xDistanceInMeters); // consider angle sign
    
    return motion;
}

#pragma mark - Motion monitor

void Controller::startMotionMonitor() {
    
    Utils::printMessage("Start motion monitor");
    
    chrono::seconds interval(kMotionMonitorFreqInSec);

    while (true) {
        
        if (isMotionUpdatedShared) {
            
            m.lock();
            isMotionUpdatedShared = false;
            m.unlock();
            
            Utils::printMessage("New pose is sending in motion");
            Utils::printMotionVector(analyzedMotionShared);
            
            function<void()> move = [=]() {
                MotionController::Instance().shouldMove(analyzedMotionShared);
            };
            
            thread t(move);
            t.detach();
        }
        
        this_thread::sleep_for(interval);
    }
}

#pragma mark - Test

// For testing motion behaviour without camera data

void Controller::startTest(vector<PoseVector> poses, double holdingPoseDistance, double speedInMeterPerSec) {
    
    this->holdingPoseDistanceInMeters = holdingPoseDistance;
    MotionController::Instance().setSpeed(speedInMeterPerSec);
    
    thread t1(&Controller::startPoseAnalyzer, this);
    thread t2(&Controller::startMotionMonitor, this);

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
    thread t3(startTestPoseGenerator);

    t1.join();
    t2.join();
    t3.join();
}
