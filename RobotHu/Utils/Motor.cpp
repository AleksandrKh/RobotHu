//
//  Motor.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 04/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "Motor.hpp"
//#include "bcm2835.h"
#include <stdexcept>
#include <unistd.h>
#include "../../Utils/Utils.hpp"

using namespace std;

Motor::Motor(int enablePin, int stepPin, int dirPin) {
    
//    if (!bcm2835_init()) {
//        Utils::printError("bcm2835 can't be init");
//        throw exception();
//    }
//    
//    this->enablePin = enablePin;
//    this->stepPin = stepPin;
//    this->dirPin = dirPin;
//    
//    bcm2835_gpio_fsel(enablePin, BCM2835_GPIO_FSEL_OUTP);
//    bcm2835_gpio_fsel(stepPin, BCM2835_GPIO_FSEL_OUTP);
//    bcm2835_gpio_fsel(dirPin, BCM2835_GPIO_FSEL_OUTP);
//    
//    // Disable
//    bcm2835_gpio_write(enablePin, LOW);
//    bcm2835_gpio_write(enablePin, HIGH);
//    
//    // "Reset" step pin
//    bcm2835_gpio_write(stepPin, LOW);
}

void Motor::enable() {
    
//    bcm2835_gpio_write(enablePin, LOW);
//    usleep(500);
}

void Motor::disable() {
    
//    bcm2835_gpio_write(enablePin, HIGH);
//    usleep(500);
}

void Motor::setDirection(int direction) {
    
//    bcm2835_gpio_write(dirPin, direction > 0 ? LOW : HIGH);
//    usleep(500);
}

void Motor::step() {
    
//    bcm2835_gpio_write(stepPin, HIGH);
//    bcm2835_gpio_write(stepPin, LOW);
//    usleep(500);
}
