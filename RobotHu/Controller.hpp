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
    
    Controller() {};
    
    void start();
    
    void startTest();
    
private:
    
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
    
    void move();
    
    void startTestPoseGenerator();
    
    std::mutex m;
};

#endif /* Controller_hpp */
