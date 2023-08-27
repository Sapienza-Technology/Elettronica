#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "pinmap.h"
#include <ros.h>
#include "utils.h"
#include <AccelStepper.h>
#include <std_msgs/Float32MultiArray.h>

#define WHEEL_RADIUS 0.08
#define MAX_V 1 //max allowed linear velocity in m/s
#define MAX_W MAX_V/WHEEL_RADIUS //max allowed angular velocity in rad/s

class Wheel {
    public:
        int pinCW;
        int pinCCW;
        int pinSpeed;

        //constructor
        Wheel(int pinCW, int pinCCW, int pinSpeed) {
            this->pinCW = pinCW;
            this->pinCCW = pinCCW;
            this->pinSpeed =pinSpeed;

            pinMode(pinCW, OUTPUT);
            digitalWrite(pinCW, HIGH);

            pinMode(pinCCW, OUTPUT);
            digitalWrite(pinCCW, LOW);

            pinMode(pinSpeed, OUTPUT);
            analogWrite(pinSpeed, 0);
       }

        void setForward(){
            digitalWrite(pinCW, HIGH);
            digitalWrite(pinCCW, LOW);
        }

        void setBackward(){
            digitalWrite(pinCW, LOW);
            digitalWrite(pinCCW, HIGH);
        }

        void setSpeed(float x) {
            float vel = abs(x);
            if (vel < 0.01) {
                analogWrite(pinSpeed, 0);
            }
           
            int pwm = fmap(vel, 0, MAX_W, 0, 255);
            if (x>0) {
                setForward();
            } else {
                setBackward();
            }
            analogWrite(pinSpeed, pwm);
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
        AccelStepper* stepper;

        //constructor
        SteeringWheel(int pinStep, int pinDir) {
            this->pinStep = pinStep;
            this->pinDir = pinDir;

            stepper=  new AccelStepper(AccelStepper::DRIVER, pinStep, pinDir);
            stepper->setMinPulseWidth(20);
            stepper->setMaxSpeed(800);
            stepper->setAcceleration(4800.0);
        }

        void moveToAngle(float angle) {
            int steps = angleToStep(angle);
            stepper->moveTo(steps);
        }

        void run() {
            stepper->run();
        }

        int angleToStep(float angle){
            //convert the desired angle for the stepper motor into
            // the number of steps to reach the desired position
            angle= angle*180 / 3.1415;
            return (int)(angle/stepper_resolution*microstep)*reduction_ratio;
        }

        float stepToAngle(int step){
            //convert the number of steps into the angle
            float angle=(float)(step*stepper_resolution/microstep)/180*PI;
            return angle/reduction_ratio;
        }
};





#endif