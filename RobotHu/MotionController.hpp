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
#include <mutex>
#include "Utils/Motor.hpp"
#include <functional>

struct MotionVector {
    
    double angleInDeg;
    double distanceInMeters;
};

class MotionController {
        
public:
    
    static MotionController& Instance() {
        
        static MotionController s;
        return s;
    }
    
    void setSpeed(double speedInMeterPerSec);
    void shouldMove(MotionVector motionVector);
    
private:
    
    MotionController() {
        setup();
    }
    ~MotionController() {}
    
    MotionController(MotionController const&) = delete;
    MotionController& operator = (MotionController const&) = delete;
    
    bool stopMotionShared;
    bool motionInProcessShared;
    
    void setup();
    void motorsSetup();
    
    double goSpeedInMeterPerSec, rotSpeedInMeterPerSec;
    double motorStepLengthInMeters;
    double rotationCircleLength;
    double goDelayInMicroSec, rotDelayInMicroSec;
    
    void move(MotionVector motionVector);
    void rotate(double angleInDeg);
    void go(double distance);
    
    Motor leftMotor, rightMotor;
    
    std::mutex m;
};

#endif /* MotionController_hpp */
