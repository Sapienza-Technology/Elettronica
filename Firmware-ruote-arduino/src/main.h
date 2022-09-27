#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "config.h"
#include "pinmap.h"
#include <ros.h>
#include "utils.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

void setLeftSpeed(float x);
void setRightSpeed(float x);

#endif