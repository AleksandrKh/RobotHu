//
//  Controller.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "Controller.hpp"
#include "PoseEstimator.hpp"
#include "MotionController.hpp"
#include "Utils.hpp"
#include <iostream>
#include <thread>
#include <cmath>

using namespace std;

#define kPoseMonitorFreqInmSec 500
#define kPosesListSize 3
#define kMotionHandlerDelayInSec 3
#define kMotionHandlerFreqInmSec 2000
#define kMinDeviationInMeters 0.05

#define kHoldingPoseDistanceInMeters 1.0
#define kHoldingPoseIgnoringAreaPercent 10.0

Controller::Controller() {
    
    currentPose = {0, 0, 0};
    lastFilteredCoordinate = {0, 0, 0};
    
    double calmnessAreaInMeters = kHoldingPoseDistanceInMeters / kHoldingPoseIgnoringAreaPercent;
    upperBorderOfCalmnessArea = kHoldingPoseDistanceInMeters + calmnessAreaInMeters;
    lowerBorderOfCalmnessArea = kHoldingPoseDistanceInMeters - calmnessAreaInMeters;
}

void Controller::start() {

    // Delegates
    function<void(vector<double>)> didObtainPoseDelegate = [this](vector<double> pose) {
        this->didObtainPoseDelegate(pose);
    };
    PoseEstimator::Instance().didObtainPoseDelegate = &didObtainPoseDelegate;

    function<void(string)> didReceiveErrorMessage = [this](string errorMessage) {
        this->didReceiveErrorMessage(errorMessage);
    };
    PoseEstimator::Instance().didReceiveErrorMessage = &didReceiveErrorMessage;
    
    thread t1(&Controller::startPoseEstimator, this);
    //t1.join();
    
    thread t2(&Controller::startPoseMonitor, this);
    //t2.join();
    
    thread t3(&Controller::startMotionHandler, this);
    t3.join();
}

void Controller::startPoseEstimator() {
    
    Utils::printMessage("Start pose estimator");

    PoseEstimator::Instance().start();
}

void Controller::startPoseMonitor() {
    
    Utils::printMessage("Start pose monitor");
    
    chrono::milliseconds interval(kPoseMonitorFreqInmSec);
    
    while (true) {
        
        if (lastPosesList.size() == kPosesListSize)
            lastPosesList.pop_front();
        
        lastPosesList.push_back(currentPose);
        
        //Utils::printCoordinate(currentPose);
        
        this_thread::sleep_for(interval);
    }
}

void Controller::startMotionHandler() {
    
    chrono::seconds delayInterval(kMotionHandlerDelayInSec);
    this_thread::sleep_for(delayInterval);

    Utils::printMessage("Start motion handler");

    chrono::milliseconds interval(kMotionHandlerFreqInmSec);
    
    while (true) {

        list<vector<double> > copyLastPosesList(lastPosesList);
        
        // Get average coordinate between last poses
        double avgX, avgY, avgZ = 0;
        
        for (list<vector<double> >::iterator it = copyLastPosesList.begin(); it != copyLastPosesList.end(); ++it) {
            
            avgX += (*it)[0];
            avgY += (*it)[1];
            avgZ += (*it)[2];
        }
        
        double posesListSize = (double)copyLastPosesList.size();
        
        avgX /= posesListSize;
        avgY /= posesListSize;
        avgZ /= posesListSize;
        
        //        function<double(double)> roundCoordinate = [](double coordinate) {
        //            return round(coordinate * 100.0) / 100.0;
        //        };
        
        vector<double> targetCoordinate = {avgX, avgY, avgZ};
        
        // Filter coordinate
        if (filterCoordinate(targetCoordinate)) {
            
            //Utils::printCoordinate(targetCoordinate);
            
            bool shouldMove = false;
            
            // Check if new coordinate is outside of "calmness area" - area around holding pose where we are keeping stillness
            
            if (targetCoordinate[2] > upperBorderOfCalmnessArea ||
                targetCoordinate[2] < lowerBorderOfCalmnessArea) {
                
                targetCoordinate[2] -= kHoldingPoseDistanceInMeters;
                shouldMove = true;
            }
            
            if (shouldMove) {
                
                MotionVector motionVector = MotionController::convertCoordinateToMotionVector(targetCoordinate);
        
                Utils::printMotionVector(motionVector);
                
                MotionController motionController;
                
                motionController.move(motionVector);
            }
        }
        
        this_thread::sleep_for(interval);
    }
}

#pragma mark - PoseEstimator delegates

void Controller::didObtainPoseDelegate(vector<double> pose) {
        
    currentPose = pose;
}

void Controller::didReceiveErrorMessage(string errorMessage) {
    
    Utils::printError(errorMessage);
}

#pragma mark -

bool Controller::filterCoordinate(vector<double> coordinate) {
    
    // Check min deviation from last coordinate
    if (fabs(fabs(lastFilteredCoordinate[0]) - fabs(coordinate[0])) > kMinDeviationInMeters ||
        fabs(fabs(lastFilteredCoordinate[1]) - fabs(coordinate[1])) > kMinDeviationInMeters ||
        fabs(fabs(lastFilteredCoordinate[2]) - fabs(coordinate[2])) > kMinDeviationInMeters) {
        
        lastFilteredCoordinate = coordinate;
        
        return true;
    }
    
    return false;
}
