//
//  MotionController.hpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 26/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#ifndef MotionController_hpp
#define MotionController_hpp

#include <vector>

struct MotionVector {
    
    double angleInDegrees;
    double distanceInMeters;
};

class MotionController {
    
public:
    
    MotionController();
    
    double motorStepInMeters;
    double machineTurningCircleLength;
    
    void move(MotionVector motionVector);
    
    static MotionVector convertCoordinateToMotionVector(std::vector<double> coordinate);
    
private:
    
    void calibrate();
    void rotate(double angleInDegrees);
    void go(double distance);
    int convertRotationAngleToSteps(double angleInDegrees);
    
};

#endif /* MotionController_hpp */
