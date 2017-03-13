#!/bin/bash

g++ ../Tools/motorTest/motorTest.cpp -o motorTest -lbcm2835

g++ ../Tools/resetMotors/resetMotors.cpp -o resetMotors -lbcm2835

g++ -std=c++14 -o robotHu ../RobotHu/main.cpp ../RobotHu/Controller.cpp ../RobotHu/PoseEstimator.cpp ../RobotHu/MotionController.cpp ../RobotHu/Utils/Motor.cpp ../Utils/Utils.cpp ../Utils/InputParser.hpp -lbcm2835 `pkg-config --cflags --libs opencv` -lpthread
