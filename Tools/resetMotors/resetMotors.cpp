//
//  resetMotors.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 04/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

// Using at OS start

// Compile: g++ resetMotors.cpp -o ../../Builds/resetMotors -lbcm2835
// Add to autostart: sudo ln -s resetMotors /etc/init.d/

#include <iostream>
#include "bcm2835.h"
#include "../../Utils/InputParser.hpp"

using namespace std;

#define kMotor1EnablePin 16
#define kMotor2EnablePin 13

int main(int argc, const char * argv[]) {
    
    string exampleCommand = "Example command: ./resetMotors -m1p 16 -m2p 13\n"
    "-m1p - motor 1 enable pin";
    
    if (argc < 3) {
        cout << "Error: wrong args number" << endl;
        cout << exampleCommand << endl;
        
        return 1;
    }
    
    InputParser input(argc, argv);

    const std::string &motor1EnablePinString = input.getCmdOption("-m1p");
    int motor1EnablePin = atoi(motor1EnablePinString.c_str());
    
    const std::string &motor2EnablePinString = input.getCmdOption("-m2p");
    int motor2EnablePin = atoi(motor2EnablePinString.c_str());
    
    if (!motor1EnablePin || !motor2EnablePin) {
        
        cout << "Error: some pin is not defined" << endl;
        cout << exampleCommand << endl;

        return 1;
    }
    
    if (!bcm2835_init())
        return 1;
    
    bcm2835_gpio_fsel(motor1EnablePin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(motor2EnablePin, BCM2835_GPIO_FSEL_OUTP);
    
    bcm2835_gpio_write(motor1EnablePin, LOW);
    bcm2835_gpio_write(motor2EnablePin, LOW);
    bcm2835_gpio_write(motor1EnablePin, HIGH);
    bcm2835_gpio_write(motor2EnablePin, HIGH);
    
    return 0;
}
