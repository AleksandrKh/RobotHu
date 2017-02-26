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

class Controller {

public:
    
    Controller();
    
    void start();
    
private:
    
    std::vector<double> currentPose;
    std::list<std::vector<double> > lastPosesList;
    std::vector<double> lastFilteredCoordinate;
    double upperBorderOfCalmnessArea, lowerBorderOfCalmnessArea;

    void startPoseEstimator();
    void startPoseMonitor();
    void startMotionHandler();
    
    // Incoming delegates
    void didObtainPoseDelegate(std::vector<double> pose);
    void didReceiveErrorMessage(std::string errorMessage);
    
    // Filters
    bool filterCoordinate(std::vector<double> coordinate);
};

#endif /* Controller_hpp */
