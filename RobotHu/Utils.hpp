//
//  Utils.hpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 26/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#ifndef Utils_hpp
#define Utils_hpp

#include <string>
#include <vector>
#include "MotionController.hpp"

#define DEGREES_TO_RADIANS(degrees) ((M_PI * degrees)/180.0f)
#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0f / M_PI))

class Utils {
    
public:
    
    // Strings
    static void printMessage(std::string message);
    static void printError(std::string errorMessage);
    static void printCoordinate(std::vector<double> coordinate);
    static void printMotionVector(MotionVector motionVector);
};

#endif /* Utils_hpp */
