//
//  motorTest.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 01/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

// Compile: g++ motorTest.cpp -o motorTest -lbcm2835

#include <iostream>
#include <stdlib.h>
#include "bcm2835.h"
#include <unistd.h>
#include "../../Utils/InputParser.hpp"

using namespace std;

int main(int argc, const char *argv[]) {
    
    string exampleCommand = "Example command: ./motorTest -steps 200 -delay 10000 -mlep 16 -mlsp 20 -mldp 21 -mld 1 -mrep 13 -mrsp 19 -mrdp 26 -mrd 1 -backward 1\n"
    "-delay: between steps in microseconds\n"
    "-mlep: motor left enable pin\n"
    "-mlsp: motor left step pin\n"
    "-mldp: motor left direction pin\n"
    "-mld: motor left direction, may be 1 or -1\n"
    "-backward: set 1 if you need reverse moving";
    
    InputParser input(argc, argv);
    
    const std::string &stepsString = input.getCmdOption("-steps");
    int steps = atoi(stepsString.c_str());
    
    const std::string &delayString = input.getCmdOption("-delay");
    int delay = atoi(delayString.c_str());
    
    const std::string &motorLeftEnablePinString = input.getCmdOption("-mlep");
    int motorLeftEnablePin = atoi(motorLeftEnablePinString.c_str());
    
    const std::string &motorLeftStepPinString = input.getCmdOption("-mlsp");
    int motorLeftStepPin = atoi(motorLeftStepPinString.c_str());
    
    const std::string &motorLeftDirectionPinString = input.getCmdOption("-mldp");
    int motorLeftDirectionPin = atoi(motorLeftDirectionPinString.c_str());
    
    const std::string &motorLeftDirectionString = input.getCmdOption("-mld");
    int motorLeftDirection = atoi(motorLeftDirectionString.c_str());
    
    const std::string &motorRightEnablePinString = input.getCmdOption("-mrep");
    int motorRightEnablePin = atoi(motorRightEnablePinString.c_str());
    
    const std::string &motorRightStepPinString = input.getCmdOption("-mrsp");
    int motorRightStepPin = atoi(motorRightStepPinString.c_str());
    
    const std::string &motorRightDirectionPinString = input.getCmdOption("-mrdp");
    int motorRightDirectionPin = atoi(motorRightDirectionPinString.c_str());
    
    const std::string &motorRightDirectionString = input.getCmdOption("-mrd");
    int motorRightDirection = atoi(motorRightDirectionString.c_str());
    
    const std::string &motorBackwardString = input.getCmdOption("-backward");
    int motorBackward = atoi(motorBackwardString.c_str());
    
    if (!steps || !delay ||
        !motorLeftEnablePin || !motorLeftStepPin || !motorLeftDirectionPin || !motorLeftDirection ||
        !motorRightEnablePin || !motorRightStepPin || !motorRightDirectionPin || !motorRightDirection) {
        
        cout << "Error: something is not defined" << endl;
        cout << exampleCommand << endl;
        
        return 1;
    }
    
    if (!bcm2835_init())
        return 1;
        
    bcm2835_gpio_fsel(motorLeftEnablePin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(motorLeftStepPin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(motorLeftDirectionPin, BCM2835_GPIO_FSEL_OUTP);
    
    bcm2835_gpio_fsel(motorRightEnablePin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(motorRightStepPin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(motorRightDirectionPin, BCM2835_GPIO_FSEL_OUTP);
    
    bcm2835_gpio_write(motorLeftEnablePin, LOW); // Enable
    bcm2835_gpio_write(motorRightEnablePin, LOW); // Enable
    
    // Reset
    bcm2835_gpio_write(motorLeftStepPin, LOW);
    bcm2835_gpio_write(motorLeftDirectionPin, LOW);
    bcm2835_gpio_write(motorRightStepPin, LOW);
    bcm2835_gpio_write(motorRightDirectionPin, LOW);
    
    bcm2835_gpio_write(motorLeftDirectionPin, motorLeftDirection > 0 ? LOW : HIGH); // Left direction
    bcm2835_gpio_write(motorRightDirectionPin, motorRightDirection < 0 ? LOW : HIGH); // Right direction
    
    usleep(1000);
    
    // Forward
    
    for (int i = 0; i < steps; i++) {
        
        // cout << "step: " << i + 1 << endl;
        
        bcm2835_gpio_write(motorLeftStepPin, HIGH);
        bcm2835_gpio_write(motorRightStepPin, HIGH);
        usleep(delay);
        bcm2835_gpio_write(motorLeftStepPin, LOW);
        bcm2835_gpio_write(motorRightStepPin, LOW);
        usleep(delay);
    }
        
    usleep(10000);
    
    if (motorBackward > 0) {
        
        // Backward
        
        bcm2835_gpio_write(motorLeftDirectionPin, motorLeftDirection < 0 ? LOW : HIGH);
        bcm2835_gpio_write(motorRightDirectionPin, motorRightDirection > 0 ? LOW : HIGH);
        
        for (int i = 0; i < steps; i++) {
            
            // cout << "step: " << i + 1 << endl;
            
            bcm2835_gpio_write(motorLeftStepPin, HIGH);
            bcm2835_gpio_write(motorRightStepPin, HIGH);
            usleep(delay);
            bcm2835_gpio_write(motorLeftStepPin, LOW);
            bcm2835_gpio_write(motorRightStepPin, LOW);
            usleep(delay);
        }
    }
    
    bcm2835_gpio_write(motorLeftEnablePin, HIGH); // Disable
    bcm2835_gpio_write(motorRightEnablePin, HIGH); // Disable
    
    return 0;
}
