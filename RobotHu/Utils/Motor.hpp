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
