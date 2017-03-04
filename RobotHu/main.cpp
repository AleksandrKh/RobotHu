//
//  main.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include <iostream>
#include "Controller.hpp"
#include "../Utils/InputParser.hpp"

using namespace std;

int main(int argc, const char * argv[]) {
    
    string exampleCommand = "Just run to start if the camera is connected it\n."
    "If you want to test without camera data then add -test 1. See what happens in the Controller::test()";
    
    InputParser input(argc, argv);
    const std::string &testString = input.getCmdOption("-test");
    
    bool testMode = testString.length() ? true : false;
    
    Controller controller;
    
    testMode ? controller.startTest() : controller.start();
        
    return 0;
}
