//
//  main.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include <iostream>
#include "Controller.hpp"

using namespace std;

int main(int argc, const char * argv[]) {
    
    // TODO make test mode in controller to start with test angle and distance
    
    Controller controller;
    
    controller.start();
        
    return 0;
}


