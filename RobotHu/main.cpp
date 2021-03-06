//
//  main.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

// Compilation from the root of main file:
// g++ -std=c++14 -o ../Builds/robotHu main.cpp Controller.cpp PoseEstimator.cpp MotionController.cpp Utils/Motor.cpp ../Utils/Utils.cpp ../Utils/InputParser.hpp -lbcm2835 `pkg-config --cflags --libs opencv` -lpthread

#include <iostream>
#include "config.h"
#include "Controller.hpp"
#include "../Utils/InputParser.hpp"
#include "../Utils/Utils.hpp"
#include "PoseEstimator.hpp"

using namespace std;

int main(int argc, const char * argv[]) {
    
    string info = "Example command: ./robotHu -hold 0.5 -speed 0.1\n"
    "-hold: distance in meters that the robot is trying to hold between camera and marker, default is 1\n"
    "-speed: speed in meter per sec\n"
    "To test without camera data execute command with poses vectors: <x> <z> <x1> <z1> .. <xn> <zn>\n"
    "Example command: ./robotHu -test 1 -hold 0.5 -speed 0.1 -poses 0.1 0.5 0.3 0 0 -0.3\n"
    "-hold: required in test mode\n"
    "-speed: required in test mode\n"
    "-x: x offset\n"
    "-z: z offset\n";
    
    cout << info << endl;
    
    InputParser input(argc, argv);
    const std::string testString = input.getCmdOption("-test");
    
    bool testMode = testString.length() ? true : false;
    
    const std::string &holdString = input.getCmdOption("-hold");
    double hold = holdString.length() ? atof(holdString.c_str()) : kDefaultHoldingPoseDistanceInMeters;
    
    const std::string &speedString = input.getCmdOption("-speed");
    double speed = speedString.length() ? atof(speedString.c_str()) : kDefaultSpeedInMeterPerSec;
    
    Controller controller;

    if (!testMode) {
        
        controller.start(hold, speed);
    }
    else {
        
        const std::string &posesString = input.getCmdOption("-poses");
        if (!posesString.length()) {
            
            Utils::printError("At least 1 pose should be defined in test mode, see example");
            return 1;
        }
        
        if (argc < 11) {
            
            Utils::printError("Something is not defined, see example");
            return 1;
        }
        
        // Parse poses from the command line
        vector<PoseVector> poses;
        
        int poseFirstArgPosition = 8;
        
        for (int i = poseFirstArgPosition; i < argc; i += 3) {
            
            PoseVector pose;
            string xDistanceInMetersString = argv[i+1];
            pose.xDistanceInMeters = atof(xDistanceInMetersString.c_str());
            string zDistanceInMetersString = argv[i+2];
            pose.zDistanceInMeters = atof(zDistanceInMetersString.c_str());

            poses.push_back(pose);
        }

        if (poses.size()) {
            
            controller.startTest(poses, hold, speed);
        }
    }

    return 0;
}
