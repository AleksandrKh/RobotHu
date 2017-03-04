//
//  Controller.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "Controller.hpp"
#include "PoseEstimator.hpp"
#include "../Utils/Utils.hpp"
#include <iostream>
#include <thread>
#include <cmath>

using namespace std;

#define kPosesListSize 3
#define kPoseListUpdateDelayInSec 0.5

#define kPoseAnalyzerFreqInMilliSec 100

#define kMinDeviationInMeters 0.05
#define kHoldingPoseDistanceInMeters 1.0
#define kCalmnessAreaInPercent 10.0

#define kMotionMonitorFreqInSec 3

void Controller::start() {

    // Delegates
    function<void(vector<double>)> didObtainPoseDelegate = [=](vector<double> pose) {
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

void Controller::didObtainPoseDelegate(vector<double> pose) {
    
    //Utils::printCoordinate(pose);
    
    time_t now = time(0);
    
    if (sharedLastPosesList.empty()) {
        
        sharedLastPosesList.push_back(pose);
        lastAcceptedPoseTime = now;
        return;
    }
    else if (difftime(now, lastAcceptedPoseTime) > kPoseListUpdateDelayInSec) {
    
        if (sharedLastPosesList.size() == kPosesListSize)
            sharedLastPosesList.pop_front();
        
        sharedLastPosesList.push_back(pose);
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
        
        // Get average coordinate between last poses
        double avgX = 0, avgY = 0, avgZ = 0;
        
        for (auto const& i : sharedLastPosesList) {
            
            avgX += i[0];
            avgY += i[1];
            avgZ += i[2];
        }
        
        double posesListSize = (double)sharedLastPosesList.size();
        
        avgX /= posesListSize;
        avgY /= posesListSize;
        avgZ /= posesListSize;
        
        vector<double> targetCoordinate = {avgX, avgY, avgZ};
        
        // Filter coordinate
        if (filterCoordinate(targetCoordinate)) {
            
            bool shouldUpdate = false;
            
            // Check if new coordinate is outside of "calmness area" - area around holding pose where we are keeping stillness
    
            double calmnessAreaInMeters = kHoldingPoseDistanceInMeters / kCalmnessAreaInPercent;

            if (targetCoordinate[2] > kHoldingPoseDistanceInMeters + calmnessAreaInMeters ||
                targetCoordinate[2] < kHoldingPoseDistanceInMeters - calmnessAreaInMeters) {
                
                targetCoordinate[2] -= kHoldingPoseDistanceInMeters;
                shouldUpdate = true;
            }
            
            if (shouldUpdate) {
                
                sharedAnalyzedMotionVector = MotionController::convertCoordinateToMotionVector(targetCoordinate);
        
                Utils::printMotionVector(sharedAnalyzedMotionVector);
            }
        }
        
        this_thread::sleep_for(interval);
    }
}

bool Controller::filterCoordinate(vector<double> coordinate) {
    
    if (lastFilteredCoordinate.empty()) {
        lastFilteredCoordinate = {0, 0, 0};
    }
    
    // Check min deviation from last coordinate
    if (fabs(lastFilteredCoordinate[0] - coordinate[0]) > kMinDeviationInMeters ||
        fabs(lastFilteredCoordinate[1] - coordinate[1]) > kMinDeviationInMeters ||
        fabs(lastFilteredCoordinate[2] - coordinate[2]) > kMinDeviationInMeters) {
        
        lastFilteredCoordinate = coordinate;
        
        return true;
    }
    
    return false;
}

#pragma mark - Motion monitor

void Controller::startMotionMonitor() {
    
    Utils::printMessage("Start motion monitor");
    
    chrono::seconds interval(kMotionMonitorFreqInSec);

    while (true) {
        
        if (fabs(sharedAnalyzedMotionVector.angleInDegrees) > __DBL_EPSILON__ || fabs(sharedAnalyzedMotionVector.distanceInMeters) > __DBL_EPSILON__) {
            
            if (!MotionController::compareMotionVectors(lastMotionVector, sharedAnalyzedMotionVector)) {
                
                lastMotionVector = sharedAnalyzedMotionVector;
                
                thread t(&Controller::move, this);
                t.detach();
            }
        }
        
        this_thread::sleep_for(interval);
    }
}

void Controller::move() {
    
    MotionController::Instance().shouldMove(lastMotionVector);
}

#pragma mark - Test

// For testing motion behaviour without camera data

void Controller::startTest() {
    
    thread t1(&Controller::startPoseAnalyzer, this);
    thread t2(&Controller::startMotionMonitor, this);
    thread t3(&Controller::startTestPoseGenerator, this);

    t1.join();
    t2.join();
    t3.join();
}

void Controller::startTestPoseGenerator() {
    
    Utils::printMessage("Start test pose generator");
    
    chrono::seconds interval(5);
    
    vector<vector<double> >testPoses = {
        
        // Just examples
        
        {-0.3, 0, kHoldingPoseDistanceInMeters - 0.9},
        {-0.31, 0, kHoldingPoseDistanceInMeters - 0.89},
        {-0.29, 0, kHoldingPoseDistanceInMeters - 0.89},
        {-0.29, 0, kHoldingPoseDistanceInMeters - 0.69},

        // add test pose
    };
    
    for (int i = 0; i < testPoses.size(); i++) {
        
        Utils::printMessage("Test pose n: " + to_string(i));
        
        didObtainPoseDelegate(testPoses[i]);
        
        this_thread::sleep_for(interval);
    }
}
