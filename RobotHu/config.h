//
//  config.h
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 12/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#ifndef config_h
#define config_h

#define kCameraID 0
#define kCameraFrameWidth 640
#define kCameraFrameHeight 480
#define kCameraParametersFile "camera.yml" // see calibrateCamera.cpp
#define kMarkerDictionaryID 8
#define kMarkerID 10
#define kMarkerLengthInMeters 0.1
#define kMaxMarkerXYRotDeviationInDegrees 20 // marker should be posed with upwards Y axis // TODO: make invariant

#define kDefaultHoldingPoseDistanceInMeters 1.0
#define kDefaultSpeedInMeterPerSec 0.1

#define kMinZDistanceDeviationInMeters 0.15
#define kMinXDistanceDeviationInMeters 0.1
#define kMinXZAnlgeDeviationInDeg 5
#define kMinXDistanceDeviationBetweenPosesInMeters 0.03
#define kMinZDistanceDeviationBetweenPosesInMeters 0.03

// 17HS4401 stepper (RED 2B GREEN 2A) (YELLOW 1A BLUE 1B) in parentheses may be reversed
#define kMotorStepsPerRevolution 200

#define kWheelsDiameterInMeters 0.064
#define kDistanceBetweenWheelsInMeters 0.18
#define kMotorStepInMetersGoCalibFactor 1.5
#define kDefaultGoSpeedInMeterPerSec 0.1

#define kLeftMotorEnablePin 16
#define kLeftMotorStepPin 20
#define kLeftMotorDirPin 21
#define kRightMotorEnablePin 13
#define kRightMotorStepPin 19
#define kRightMotorDirPin 26

#define xSideFactor 1.0 // 1 - Left side of the camera under zero, right is over zero, -1 - vice versa
#define ySideFactor 1.0 // 1 - Upper side of the camera under zero, lower is over zero, -1 - vice versa

#endif /* config_h */
