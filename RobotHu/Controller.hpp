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
#include <mutex>
#include "MotionController.hpp"

class Controller {

public:
    
    void start();
    void start(double holdingPoseDistance);
    
    void startTest(std::vector<std::vector<double> > poses, double holdingPoseDistance);
    
private:
    
    double holdingPoseDistance;
    
    MotionVector sharedAnalyzedMotionVector, lastMotionVector;
    std::list<std::vector<double> > sharedLastPosesList;

    time_t lastAcceptedPoseTime;
    std::vector<double> lastFilteredCoordinate;

    void startPoseEstimator();
    void startPoseAnalyzer();
    void startMotionMonitor();
    
    // Incoming delegates
    void didObtainPoseDelegate(std::vector<double> pose);
    void didReceiveErrorMessage(std::string errorMessage);
    
    // Filters
    bool filterCoordinate(std::vector<double> coordinate);
        
    std::mutex m;
};

#endif /* Controller_hpp */
