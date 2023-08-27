// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>


#include "utils.h"
#include "pinmap.h"
#define WHEEL_RADIUS 0.08
#define MAX_V 1 //max allowed linear velocity in m/s
#define MAX_W MAX_V/WHEEL_RADIUS //max allowed angular velocity in rad/s
ros::NodeHandle nh;

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


int angleToStep(float angle){
      //convert the desired angle for the stepper motor into
      // the number of steps to reach the desired position
      angle= angle*180 / PI;
      return (int)(angle/1.8*4)*20;
  }

  Wheel wheel_FL(DRV_F_L_CW,DRV_F_L_CCW,DRV_F_L_S);
  Wheel wheel_FR(DRV_F_R_CW,DRV_F_R_CCW,DRV_F_R_S);
  Wheel wheel_RL(DRV_R_L_CW,DRV_R_L_CCW,DRV_R_L_S);
  Wheel wheel_RR(DRV_R_R_CW,DRV_R_R_CCW,DRV_R_R_S);
  Wheel wheel_ML(DRV_M_L_CW,DRV_M_L_CCW,DRV_M_L_S);
  Wheel wheel_MR(DRV_M_R_CW,DRV_M_R_CCW,DRV_M_R_S);
//array with all the wheels
Wheel wheels[6] = {wheel_FL, wheel_FR, wheel_ML, wheel_MR,wheel_RL, wheel_RR};

//SteeringWheel* steering[4] = {steering_FL, steering_FR, steering_RL, steering_RR};
  AccelStepper steering_FL(AccelStepper::DRIVER,STEER_F_L_S, STEER_F_L_D);
  AccelStepper steering_FR(AccelStepper::DRIVER,STEER_F_R_S, STEER_F_R_D);
  AccelStepper steering_RL(AccelStepper::DRIVER,STEER_R_L_S, STEER_R_L_D);
  AccelStepper steering_RR(AccelStepper::DRIVER,STEER_R_R_S, STEER_R_R_D);
AccelStepper steering[4] = {steering_FL, steering_FR, steering_RL, steering_RR};
//AccelStepper stepper=AccelStepper(AccelStepper::DRIVER, STEER_R_L_S, STEER_R_L_D);



//a float32 multiarray is received from ROS
void vel_cb(const std_msgs::Float32MultiArray& cmd) {

    for (int i=0; i<6; i++) {
      wheels[i].setSpeed(cmd.data[i]);
    }
  for (int i=0; i<4; i++){
    steering[i].moveTo(angleToStep(-cmd.data[i+6]));
  }
 // steering[0].moveTo(angleToStep(cmd.data[6]));
 // steering[1].moveTo(angleToStep(cmd.data[7]));
 // steering[2].moveTo(angleToStep(cmd.data[8]));
 // steering[3].moveTo(angleToStep(cmd.data[9]));
}
ros::Subscriber<std_msgs::Float32MultiArray> velSub("wheel_velocities", vel_cb);
void setup(){  
  Serial.begin(57600);
  for (int i=0; i<4; i++){
    steering[i].setMinPulseWidth(20);
    steering[i].setAcceleration(4800.0);
    steering[i].setMaxSpeed(2400);
    //steering[i].moveTo(angleToStep(-PI/4));	
  }
  //wheel_ML.setSpeed(7);
  nh.initNode();
  nh.subscribe(velSub);

}

void loop()
{  
    for (int i=0; i<4; i++){
      steering[i].run();
  }
  nh.spinOnce();

}
