//
//  Motor.hpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 04/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#ifndef Motor_hpp
#define Motor_hpp

#include <stdio.h>

// 17HS4401 stepper (RED 2B GREEB 2A) (YELLOW 1A BLUE 1B) in parentheses may be reversed
#define k17HS4401MotorStepsPerRevolution 200

class Motor {
    
public:
    
    Motor() {};
    Motor(int enablePin, int stepPin, int dirPin);
    void enable();
    void disable();
    void setDirection(int direction);
    void step();
    
private:
    int enablePin;
    int stepPin;
    int dirPin;
};

#endif /* Motor_hpp */
