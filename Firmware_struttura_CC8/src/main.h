#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "pinmap.h"
#include <ros.h>
#include "utils.h"
#include <AccelStepper.h>
#include <ArduinoSTL.h>
#include <std_msgs/Float32MultiArray.h>

#define WHEEL_RADIUS 0.08
#define MAX_V 1
#define MAX_W MAX_V/WHEEL_RADIUS

class Wheel {
    public:
        int pinForward;
        int pinBackward;

        //constructor
        Wheel(int pinForward, int pinBackward) {
            this->pinForward = pinForward;
            this->pinBackward = pinBackward;

            pinMode(pinForward, OUTPUT);
            digitalWrite(pinForward, LOW);

            pinMode(pinBackward, OUTPUT);
            digitalWrite(pinBackward, LOW);
        }

        void setSpeed(float x) {
            float vel = abs(x);
            if (vel < 0.01) {
                analogWrite(pinForward, 0);
                analogWrite(pinBackward, 0);
            }
           
            int pwm = fmap(vel, 0, MAX_W, 0, 255);
            if (x>0) {
                analogWrite(pinForward, pwm);
                analogWrite(pinBackward, 0);
            } else {
                analogWrite(pinForward, 0);
                analogWrite(pinBackward, pwm);
            }
        }


};


class SteeringWheel{
    public:
        int pinStep;
        int pinDir;
        int pinEnable;
        int stepper_resolution=STP_RESOLUTION;
        int microstep=MICROSTEP;
        int reduction_ratio=REDUCTION_RATIO;

        //constructor
        void SteeringWheel(int pinStep, int pinDir) {
            this->pinStep = pinStep;
            this->pinDir = pinDir;

            AccelStepper stepper(AccelStepper::DRIVER, pinStep, pinDir);
            stepper.setMinPulseWidth(20);
            stepper.setMaxSpeed(200);
            stepper.setAcceleration(200.0);
        }

        void moveToAngle(float angle) {
            int steps = angleToStep(angle);
            stepper.moveTo(steps);
        }

        void run() {
            stepper.run();
        }

        private:
            int angleToStep(float angle){
                //convert the desired angle for the stepper motor into
                // the number of steps to reach the desired position
                angle=angle/PI *180.0;
                return (int)(angle/stepper_resolution*microstep)*reduction_ratio;
            }

            float stepToAngle(int step){
                //convert the number of steps into the angle
                float angle=(float)(step*stepper_resolution/microstep)/180*PI;
                return angle/reduction_ratio;
            }
};





#endif