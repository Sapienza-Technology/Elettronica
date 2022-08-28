#include "main.h"
//https://hackaday.io/project/183279-accelstepper-the-missing-manual/details

ros::NodeHandle nh;
//number of joints and which joint is a servo motor
#define N_JOINT 6
#define SERVO_IDX 5

//arm servo motor parameters
//TODO: find correct number
#define ENDSTOP_M_A 600
#define ENDSTOP_M_B 2400

//end effector parameters
//TODO: check this stuff
#define ENDSTOP_A 600
#define ENDSTOP_B 2400


//res= step/n_revolution
float stepper_resolution=1.8;
//check if resolution is different for each stepper

//TODO: check sintax
//list of stepper motors
AccelStepper[] joints_motors;

//velocities and position to reach for the arm
float[] targetVelocities;
float[] targetPositions;
float EEPos=0;


int angleToStep(float angle){
  //convert the desired angle for the stepper motor into
  // the number of steps to reach the desired position
  return (int)(angle/stepper_resolution);
}

//define stepper motors
// Motor Connections (constant current, step/direction bipolar motor driver)
//AccelStepper::DRIVER works for a4988 (Bipolar, constant current, step/direction driver)
AccelStepper stepper1(AccelStepper::DRIVER, STP0, DIR0);           
joints_motors.push_back(stepper1);

AccelStepper stepper2(AccelStepper::DRIVER, STP1, DIR1); 
joints_motors.push_back(stepper2);

AccelStepper stepper3(AccelStepper::DRIVER, STP2, DIR2); 
joints_motors.push_back(stepper3);

AccelStepper stepper4(AccelStepper::DRIVER, STP3, DIR3); 
joints_motors.push_back(stepper4);

AccelStepper stepper5(AccelStepper::DRIVER, STP4, DIR4); 
joints_motors.push_back(stepper5);

//define servo motor
Servo S;

//end effector motors
Servo EE1;
Servo EE2;

//subscribe to ros topic
ros::Subscriber<std_msgs::Float32MultiArray> armPosSub("firmware_arm_pos", setTargetPos_cb);
ros::Subscriber<std_msgs::Float32MultiArray> armVelSub("firmware_arm_vel", setTargetVel_cb);
ros::Subscriber<std_msgs::Float32> EEPSub("firmware_EEP_pos", pirulatore_cb);
ros::Subscriber<std_msgs::Float32> EEMSub("firmware_EEM_pos", pinza_cb);

//received a desired position for the arm
void setTargetPos_cb(const std_msgs::Float32MultiArray& cmd) {
    //extract data from message
    for (int i=0; i<N_JOINT; i++){
        targetPositions[i]=angleToStep(cmd.data[i]);
    }

    //set desired position
    for(int i=0; i<6; i++){
        if (targetPositions[i]){
            if (i!=SERVO_IDX){
                //set desired position for stepper
                joints_motors[i].moveTo(joint_target_pos[i]);
                float vel= (float)targetVelocities[i];
                if (vel) joints_motors[i].setSpeed(vel);
            }
            else {
                //move servo
                S.write(joint_target_pos[i]);
            }
        }
    }
}

//received a desired velocity for the arm
void setTargetVel_cb(const std_msgs::Float32MultiArray& cmd) {
    //extract data from message
    for (int i=0; i<N_JOINT; i++){
        targetVelocities[i]=cmd.data[i];
    }

    //set desired position
    for(int i=0; i<6; i++){
        if (targetVelocities[i])
            float vel= (float)targetVelocities[i];
            if (vel>0 && i!=SERVO_IDX){
                //set desired velcoity for stepper
                joints_motors[i].setSpeed(vel);
            }
            else {
                //velocity for servo?

            }
    }
}

//received a desired position for the end effector 0 (close) 1 (open)
//receive only 0,1 ideally
void pirulatore_cb(const std_msgs::Float32& cmd) {
    //extract data from message
    EEPos=cmd.data;
    //move servo
    int pos=fmap(EEPos, 0, 1, 0, 90);
    EE1.write(pos);
}

//received a desired position for the end effector 0 (close) 1 (open)
void pinza_cb(const std_msgs::Float32& cmd) {
    //extract data from message
    EEPos=cmd.data;
    //move servo
    int pos= fmap(EEPos, 0, 1, 0, 90);
    EE1.write(pos)
    EE2.write(180 - pos);
}

void setup() {
    // ROS
    nh.initNode();
    nh.subscribe(armPosSub);
    nh.subscribe(armVelSub);
    nh.subscribe(EESub);

    //set max speed for stepper motors
    for(int i=0; i<N_JOINT; i++){
        if (i!=SERVO_IDX) joints_motors[i].setMaxSpeed(100.0);
    }

    //init servo
    //TODO: check if this is the correct pin
    S.attach(MSERVO0, ENDSTOP_M_A, ENDSTOP_M_B);
    //S.write(start_pos)
    s1.attach(SERVO0, ENDSTOP_A, ENDSTOP_B);
    s1.write(pos);
    s2.attach(SERVO1, ENDSTOP_A, ENDSTOP_B);
    s2.write(180 - pos);
    //debug stuff
    //Serial.begin(9600); //what is this?
    //Serial.print(F("Posizione servo: "));
    //Serial.println(pos);
}

void loop() {
    //for each stepper verify if position is reached
    //if not, run stepper
    for (int i = 0; i < joints_motors.size(); i++) {
       if (i!=SERVO_IDX) {
            if(joints_motors[i].distanceToGo()!=0)  {
                joints_motors[i].run();
            }
        }
        else {
            // loop for servo? should not be necessary
            //maybe only check endstop
        }
    }
    
    //ros node stuff
    nh.spinOnce();
    delay(1);
}
