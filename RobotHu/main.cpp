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

using namespace std;

int main(int argc, const char * argv[]) {
    
    string info = "Camera should be connected.\n"
    "To test without camera data execute command:\n"
    "./robotHu -test 1 -poses <x> <y> <z> <x1> <y1> <z1> .. <xn> <yn> <zn>\n"
    "Example command: ./robotHu -test 1 -poses 0 0 0.5 0.1 0 0 -0.2 0 -0.3\n";
    cout << info << endl;
    
    InputParser input(argc, argv);
    const std::string &testString = input.getCmdOption("-test");
    
    Controller controller;
    
    bool testMode = testString.length() ? true : false;
    
    if (!testMode) {
        
        controller.start();
    }
    else {
    
        const std::string &posesString = input.getCmdOption("-poses");
        
        if (!posesString.length()) {
            cout << "Error: at least 1 pose should be defined in test mode" << endl;
            return 1;
        }
        
        // Parse poses from the command line
        vector<vector<double> > poses;
        int poseFirstArgPosition = 4;
        for (int i = poseFirstArgPosition; i < argc; i += 3) {
            vector<double> pose;
            for (int j = 0; j < 3; j++) {
                string value = argv[i+j];
                pose.push_back(atof(value.c_str()));
            }
            poses.push_back(pose);
        }

        if (poses.size()) {
            
            controller.startTest(poses);
        }
    }

    return 0;
}
