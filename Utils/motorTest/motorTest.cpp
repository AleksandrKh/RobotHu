//
//  motorTest.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 01/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include <iostream>
#include "wiringPi.h"
#include <stdlib.h>

using namespace std;

int main(int argc, const char * argv[]) {
    
    if (argc < 2) {
        cout << "Example command: ./motorTest -pin=0<output number> --steps=10<number of steps> --direction=1<maybe 1 or -1>\n";
        return 0;
    }
    
    int pin = atoi(argv[1]);
    
    int stepsNum = argc > 2 ? atoi(argv[2]) : 10;
    int direction = argc > 3 ? atoi(argv[3]) : 1;

    cout << "Pin: " << pin << " Steps: " << stepsNum << " Direction: " << direction << endl;
    
    wiringPiSetup ();
    pinMode(pin, OUTPUT);
    
    for (int i = 0; i < stepsNum; i++) {
        
        direction == 1 ? digitalWrite(pin, HIGH) : digitalWrite(pin, LOW);
    }
    
    return 0;
}
