//
//  Utils.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 26/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "Utils.hpp"
#include <iostream>
#include <cmath>

using namespace std;

#pragma mark - Log
    
void Utils::printMessage(string message) {
        
        cout << message + "\n" << endl;
    }

void Utils::printError(string errorMessage) {

    cout << "ERROR: " << errorMessage + "\n" << endl;
}

void Utils::printCoordinate(vector<double> coordinate) {
    
    cout << "X: " + to_string(coordinate[0]) + " Y: " + to_string(coordinate[1]) + " Z: " + to_string(coordinate[2]) + "\n" << endl;
}

void Utils::printMotionVector(MotionVector motionVector) {
    
    cout << "Angle: " + to_string(motionVector.angleInDegrees) + " Distance: " + to_string(motionVector.distanceInMeters) + "\n" << endl;
}
