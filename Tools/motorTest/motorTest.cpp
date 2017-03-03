//
//  motorTest.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 01/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

// Compile: g++ motorTest.cpp -lbcm2835

#include <iostream>
#include <stdlib.h>
#include "bcm2835.h"
#include <unistd.h>

using namespace std;

#define kMotorLeftEnablePin 16
#define kMotorLeftStepPin 20
#define kMotorLeftDirectionPin 21
#define kMotorLeftDirection 1

#define kMotorRightEnablePin 13
#define kMotorRightStepPin 19
#define kMotorRightDirectionPin 26
#define kMotorRightDirection 1

#define kSteps 200

int main(int argc, const char * argv[]) {
    
    if (!bcm2835_init())
        return 1;
    
    bcm2835_gpio_fsel(kMotorLeftEnablePin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(kMotorLeftStepPin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(kMotorLeftDirectionPin, BCM2835_GPIO_FSEL_OUTP);
    
    bcm2835_gpio_fsel(kMotorRightEnablePin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(kMotorRightStepPin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(kMotorRightDirectionPin, BCM2835_GPIO_FSEL_OUTP);
    
    bcm2835_gpio_write(kMotorLeftEnablePin, LOW); // Enable
    bcm2835_gpio_write(kMotorRightEnablePin, LOW); // Enable
    
    // Reset
    bcm2835_gpio_write(kMotorLeftStepPin, LOW);
    bcm2835_gpio_write(kMotorLeftDirectionPin, LOW);
    bcm2835_gpio_write(kMotorRightStepPin, LOW);
    bcm2835_gpio_write(kMotorRightDirectionPin, LOW);
    
    bcm2835_gpio_write(kMotorLeftDirectionPin, kMotorLeftDirection > 0 ? LOW : HIGH); // Left direction
    bcm2835_gpio_write(kMotorRightDirectionPin, kMotorRightDirection < 0 ? LOW : HIGH); // Right direction
    
    usleep(1000);
    
    for (int i = 0; i < kSteps; i++) {
        
        // cout << "step: " << i + 1 << endl;
        
        bcm2835_gpio_write(kMotorLeftStepPin, HIGH);
        bcm2835_gpio_write(kMotorRightStepPin, HIGH);
        usleep(10000);
        bcm2835_gpio_write(kMotorLeftStepPin, LOW);
        bcm2835_gpio_write(kMotorRightStepPin, LOW);
        usleep(10000);
    }
    
    usleep(10000);
    
    bcm2835_gpio_write(kMotorLeftDirectionPin, kMotorLeftDirection < 0 ? LOW : HIGH);
    bcm2835_gpio_write(kMotorRightDirectionPin, kMotorRightDirection > 0 ? LOW : HIGH);
    
    usleep(1000);
    
    for (int i = 0; i < kSteps; i++) {
        
        // cout << "step: " << i + 1 << endl;
        
        bcm2835_gpio_write(kMotorLeftStepPin, HIGH);
        bcm2835_gpio_write(kMotorRightStepPin, HIGH);
        usleep(10000);
        bcm2835_gpio_write(kMotorLeftStepPin, LOW);
        bcm2835_gpio_write(kMotorRightStepPin, LOW);
        usleep(10000);
    }
    
    bcm2835_gpio_write(kMotorLeftEnablePin, HIGH); // Disable
    bcm2835_gpio_write(kMotorRightEnablePin, HIGH); // Disable
    
    return 0;
}
