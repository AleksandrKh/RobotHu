//
//  Controller.hpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#ifndef Controller_hpp
#define Controller_hpp

#include <vector>
#include <functional>
#include <list>
#include "MotionController.hpp"
#include "PoseEstimator.hpp"
#include <mutex>

class Controller {

public:
    
    Controller();
    
    void start(double holdingPoseDistanceInMeters, double speedInMeterPerSec);
    
    void startTest(std::vector<PoseVector> poses, double holdingPoseDistance, double speedInMeterPerSec);
    
private:
    
    double holdingPoseDistanceInMeters;
    
    std::list<PoseVector > lastPosesListShared;
    PoseVector lastPose;
    MotionVector analyzedMotionShared;
    bool isMotionUpdatedShared;

    time_t lastAcceptedPoseTime;

    void startPoseEstimator();
    void startPoseAnalyzer();
    void startMotionMonitor();
    
    // Incoming delegates
    void didObtainPoseDelegate(PoseVector pose);
    void didReceiveErrorMessage(std::string errorMessage);
    
    // Filters
    bool filterPose(PoseVector pose);
    MotionVector convertPoseToMotion(PoseVector pose);
    
    std::mutex m;
};

#endif /* Controller_hpp */
