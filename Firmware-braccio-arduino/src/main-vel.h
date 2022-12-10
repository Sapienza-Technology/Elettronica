#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "pinmap.h"
#include <ros.h>
#include "utils.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <ArduinoSTL.h>
#include <Servo.h>
#include <AccelStepper.h>
//#include <BasicLinearAlgebra.h>
//#include <elapsedMillis.h>

void setTargetPos_cb(const std_msgs::Float32MultiArray& cmd);
void setTargetVel_cb(const std_msgs::Float32MultiArray& cmd);
void pirulatore_cb(const std_msgs::Float32& cmd);
void pinza_cb(const std_msgs::Float32& cmd);
int angleToStep(float angle);
void ReadRawAngle();
void correctAngle();
void checkQuadrant();
void checkMagnetPresence();

#endif