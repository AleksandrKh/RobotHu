//
//  main.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

// Compilation from the route of main file:
// g++ -std=c++14 -o ../Builds/robotHu main.cpp Controller.cpp PoseEstimator.cpp MotionController.cpp Utils/Motor.cpp ../Utils/Utils.cpp ../Utils/InputParser.hpp -lbcm2835 `pkg-config --cflags --libs opencv` -lpthread

#include <iostream>
#include "Controller.hpp"
#include "../Utils/InputParser.hpp"
#include "../Utils/Utils.hpp"

#define kDefaultHoldingPoseDistanceInMeters 1.0
#define kDefaultSpeedInMeterPerSec 0.1

using namespace std;

int main(int argc, const char * argv[]) {
    
    string info = "Example command: ./robotHu -hold 0.5 -speed 0.1\n"
    "-hold - distance in meters that the robot is trying to hold between camera and marker, default is 1\n"
    "-speed - speed in meter per sec\n"
    "To test without camera data execute command with poses vectors: <x> <y> <z> <x1> <y1> <z1> .. <xn> <yn> <zn>\n"
    "Example command: ./robotHu -test 1 -hold 0.5 -poses 0 0 0.5 0.1 0 0 -0.2 0 -0.3\n"
    "-hold - required in test mode";
    
    cout << info << endl;
    
    InputParser input(argc, argv);
    const std::string &testString = input.getCmdOption("-test");
    
    Controller controller;
    
    bool testMode = testString.length() ? true : false;
    
    if (!testMode) {
        
        const std::string &holdString = input.getCmdOption("-hold");
        double hold = holdString.length() ? atof(holdString.c_str()) : kDefaultHoldingPoseDistanceInMeters;
        
        const std::string &speedString = input.getCmdOption("-speed");
        double speed = speedString.length() ? atof(speedString.c_str()) : kDefaultSpeedInMeterPerSec;

        controller.start(hold, speed);
    }
    else {
    
        const std::string &holdString = input.getCmdOption("-hold");
        
        if (!holdString.length()) {
            
            Utils::printError("-hold param is not exist");
            return 1;
        }
        
        double holdDistance = atof(holdString.c_str());
        
        const std::string &posesString = input.getCmdOption("-poses");
        if (!posesString.length()) {
            
            Utils::printError("At least 1 pose should be defined in test mode");
            return 1;
        }
        
        if (argc < 9) {
            
            Utils::printError("Something is not defined");
            return 1;
        }
        
        // Parse poses from the command line
        vector<vector<double> > poses;
        int poseFirstArgPosition = 6;
        for (int i = poseFirstArgPosition; i < argc; i += 3) {
            vector<double> pose;
            for (int j = 0; j < 3; j++) {
                string value = argv[i+j];
                pose.push_back(atof(value.c_str()));
            }
            poses.push_back(pose);
        }

        if (poses.size()) {
            
            controller.startTest(poses, holdDistance);
        }
    }

    return 0;
}
