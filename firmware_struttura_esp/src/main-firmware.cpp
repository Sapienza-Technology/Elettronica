#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <Adafruit_PWMServoDriver.h>
#include <AccelStepperI2C.h>

#include "utils.h"
#include "pinmap.h"
#define WHEEL_RADIUS 0.08
#define MAX_V 1 //max allowed linear velocity in m/s
#define MAX_W MAX_V/WHEEL_RADIUS //max allowed angular velocity in rad/s
ros::NodeHandle nh;
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver(0x40);
I2Cwrapper wrapper(0x40); 


class Wheel {
    public:
        int pinCW;
        int pinCCW;
        int pinSpeed;
        uint8_t n;
        

        //constructor
        Wheel(int pinCW, int pinCCW, uint8_t n) {
            this->pinCW = pinCW;
            this->pinCCW = pinCCW;
            
            this->n = n;
            pinMode(pinCW, OUTPUT);
            digitalWrite(pinCW, HIGH);

            pinMode(pinCCW, OUTPUT);
            digitalWrite(pinCCW, LOW);
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
              pwm.setPWM(n, 0, 0);
            }
           
            int speed = fmap(vel, 0, MAX_W, 0, 4095);
            if (x>0) {
                setForward();
            } else {
                setBackward();
            }
            pwm.setPWM(n, 0, speed);
        }


};


int angleToStep(float angle){
      //convert the desired angle for the stepper motor into
      // the number of steps to reach the desired position
      angle= angle*180 / PI;
      return (int)(angle/1.8*4)*20;
  }





Wheel wheel_FL(DRV_F_L_CW,DRV_F_L_CCW,DRV_F_L_IDX);
Wheel wheel_FR(DRV_F_R_CW,DRV_F_R_CCW,DRV_F_R_IDX);
Wheel wheel_RL(DRV_R_L_CW,DRV_R_L_CCW,DRV_R_L_IDX);
Wheel wheel_RR(DRV_R_R_CW,DRV_R_R_CCW,DRV_R_R_IDX);
Wheel wheel_ML(DRV_M_L_CW,DRV_M_L_CCW,DRV_M_L_IDX);
Wheel wheel_MR(DRV_M_R_CW,DRV_M_R_CCW,DRV_M_R_IDX);
//array with all the wheels
Wheel wheels[6] = {wheel_FL, wheel_FR, wheel_ML, wheel_MR,wheel_RL, wheel_RR};


//SteeringWheel* steering[4] = {steering_FL, steering_FR, steering_RL, steering_RR};
AccelStepperI2C steering_FL(&wrapper);
AccelStepperI2C steering_FR(&wrapper);
AccelStepperI2C steering_RL(&wrapper);
AccelStepperI2C steering_RR(&wrapper);



AccelStepperI2C steering[4] = {steering_FL, steering_FR, steering_RL, steering_RR};


//a float32 multiarray is received from ROS
void vel_cb(const std_msgs::Float32MultiArray& cmd) {

  for (int i=0; i<6; i++) {
    wheels[i].setSpeed(cmd.data[i]);
  }

  for (int i=0; i<4; i++){
    steering[i].moveTo(angleToStep(-cmd.data[i+6]));
  }

  //TODO check if here or on loop
  for (int i=0; i<4; i++){
    steering[i].runState(); //should not depend on calling frequency
  }

}

ros::Subscriber<std_msgs::Float32MultiArray> velSub("wheel_velocities", vel_cb);

void setup(){  
  Serial.begin(57600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  wrapper.reset(); // reset the target device

  steering_FL.attach(AccelStepper::DRIVER,STEER_F_L_S, STEER_F_L_D);
  steering_FR.attach(AccelStepper::DRIVER,STEER_F_R_S, STEER_F_R_D);
  steering_RL.attach(AccelStepper::DRIVER,STEER_R_L_S, STEER_R_L_D);
  steering_RR.attach(AccelStepper::DRIVER,STEER_R_R_S, STEER_R_R_D);
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

  nh.spinOnce();

}
