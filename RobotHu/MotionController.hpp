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

struct MotionVector {
    
    double angleInDegrees;
    double distanceInMeters;
};

class MotionController {
    
public:
    
    static MotionController& Instance() {
        
        static MotionController s;
        return s;
    }
    
    void setMotionVector(MotionVector _motionVector);
    
    static MotionVector convertCoordinateToMotionVector(std::vector<double> coordinate);
    
private:
    
    MotionController() {
        setup();
    }
    ~MotionController() {}
    
    MotionController(MotionController const&) = delete;
    MotionController& operator = (MotionController const&) = delete;
    
    bool newMotion;
    bool motionInProcess;
    MotionVector motionVector;
    
    double motorStepInMeters;
    double machineTurningCircleLength;
    
    void setup();
    void motorSetup();
    void startMovingMonitor();
    void move();
    void rotate(double angleInDegrees);
    void go(double distance);
    int convertRotationAngleToSteps(double angleInDegrees);
    
    void testMotion();
    double goDelay, rotDelay;
    void stepLeftMotor(int direction);
    void stepRightMotor(int direction);
    void stepMotor(int motorPin, int direction);
};

#endif /* MotionController_hpp */
