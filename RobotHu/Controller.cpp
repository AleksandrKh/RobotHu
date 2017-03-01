//
//  Controller.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "Controller.hpp"
#include "PoseEstimator.hpp"
#include "Utils.hpp"
#include <iostream>
#include <thread>
#include <cmath>

using namespace std;

#define kPosesListSize 3
#define kPoseListUpdateDelayInSec 0.5

#define kPoseAnalyzerFreqInMilliSec 100

#define kMinDeviationInMeters 0.05
#define kHoldingPoseDistanceInMeters 1.0
#define kHoldingPoseIgnoringAreaPercent 10.0

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

        //list<vector<double> > lastPosesList(sharedLastPosesList);
        
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
        
        //        function<double(double)> roundCoordinate = [](double coordinate) {
        //            return round(coordinate * 100.0) / 100.0;
        //        };
        
        vector<double> targetCoordinate = {avgX, avgY, avgZ};
        
        // Filter coordinate
        if (filterCoordinate(targetCoordinate)) {
            
            bool shouldUpdate = false;
            
            // Check if new coordinate is outside of "calmness area" - area around holding pose where we are keeping stillness
    
            double calmnessAreaInMeters = kHoldingPoseDistanceInMeters / kHoldingPoseIgnoringAreaPercent;

            if (targetCoordinate[2] > kHoldingPoseDistanceInMeters + calmnessAreaInMeters ||
                targetCoordinate[2] < kHoldingPoseDistanceInMeters - calmnessAreaInMeters) {
                
                targetCoordinate[2] -= kHoldingPoseDistanceInMeters;
                shouldUpdate = true;
            }
            
            if (shouldUpdate) {
                
                sharedMotionVector = MotionController::convertCoordinateToMotionVector(targetCoordinate);
        
                Utils::printMotionVector(sharedMotionVector);
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
    if (fabs(fabs(lastFilteredCoordinate[0]) - fabs(coordinate[0])) > kMinDeviationInMeters ||
        fabs(fabs(lastFilteredCoordinate[1]) - fabs(coordinate[1])) > kMinDeviationInMeters ||
        fabs(fabs(lastFilteredCoordinate[2]) - fabs(coordinate[2])) > kMinDeviationInMeters) {
        
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
        
        if (fabs(sharedMotionVector.angleInDegrees) > __DBL_EPSILON__ && fabs(sharedMotionVector.distanceInMeters) > __DBL_EPSILON__) {
            
            if (!MotionController::compareMotionVectors(lastMotionVector, sharedMotionVector)) {
                
                lastMotionVector = sharedMotionVector;
                
                thread t(&Controller::move, this);
                t.detach();
            }
        }
        
        this_thread::sleep_for(interval);
    }
}

void Controller::move() {
    
    MotionController::Instance().shouldMove(sharedMotionVector);
}
