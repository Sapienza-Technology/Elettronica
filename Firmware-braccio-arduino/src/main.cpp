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
AccelStepper stepper_motors[5];

//velocities and position to reach for the arm
float targetVelocities[6];
float targetPositions[6];
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
AccelStepper stepper2(AccelStepper::DRIVER, STP1, DIR1); 
AccelStepper stepper3(AccelStepper::DRIVER, STP2, DIR2); 
AccelStepper stepper4(AccelStepper::DRIVER, STP3, DIR3); 
AccelStepper stepper5(AccelStepper::DRIVER, STP4, DIR4); 

//define servo motor
Servo S;
Servo s1;
Servo s2;

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
    //digitalWrite(LED_BUILTIN, HIGH);
    //sleep
    //delay(100);
    printf("received data:");
    //extract data from message
    for (int i=0; i<N_JOINT; i++){
        //std::cout << cmd << std::endl;
        //Serial.println(cmd.data[i]);
        printf("%f ", cmd.data[i]);
        if(i==SERVO_IDX){
            targetPositions[i]=cmd.data[i];
        }
        else{
            //targetPositions[i]=angleToStep(cmd.data[i]);
            targetPositions[i]=cmd.data[i];
        }
    }
        digitalWrite(LED_BUILTIN, HIGH);
    
    //set desired position
    for(int i=0; i<5; i++){
        //set desired position for stepper

        stepper_motors[i].moveTo(targetPositions[i]);
        stepper_motors[i].setSpeed(400.0);
        stepper_motors[i].setMaxSpeed(500.0);

        //float vel= (float)targetVelocities[i];
        //if (vel) stepper_motors[i].setSpeed(vel);
    }
    S.write(targetPositions[SERVO_IDX]);
    
}

//received a desired velocity for the arm
void setTargetVel_cb(const std_msgs::Float32MultiArray& cmd) {
    //extract data from message
    for (int i=0; i<N_JOINT; i++){
        targetVelocities[i]=cmd.data[i];
    }

    //set desired position
    for(int i=0; i<5; i++){
        if (targetVelocities[i]) {
            float vel= (float)targetVelocities[i];
            if (vel>0){
                //set desired velcoity for stepper
                stepper_motors[i].setSpeed(vel);
            }
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
    EE1.write(pos);
    EE2.write(180 - pos);
}

void setup() {
    ////Serial.println("pierino");
    // ROS
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    //debug print teensy

    
    //std::cout << "Hello world!" << std::endl;
    nh.initNode();
    nh.subscribe(armPosSub);
    nh.subscribe(armVelSub);
    nh.subscribe(EEPSub);
    nh.subscribe(EEMSub);
    stepper_motors[0]=stepper1;
    stepper_motors[1]=stepper2;
    stepper_motors[2]=stepper3;
    stepper_motors[3]=stepper4;
    stepper_motors[4]=stepper5;


    //set max speed for stepper motors
    for(int i=0; i<5; i++){
        stepper_motors[i].setMaxSpeed(2000.0);
        //stepper_motors[i].setAcceleration(100.0);
        stepper_motors[i].setMinPulseWidth(10);

        //stepper_motors[i].moveTo(0);
        //stepper_motors[i].setAcceleration(50.0);
    }
    /*
    //init servo
    //TODO: check if this is the correct pin
    S.attach(MSERVO0, ENDSTOP_M_A, ENDSTOP_M_B);
    //S.write(start_pos)
    s1.attach(SERVO0, ENDSTOP_A, ENDSTOP_B);
    int pos=0;
    s1.write(pos);
    s2.attach(SERVO1, ENDSTOP_A, ENDSTOP_B);
    s2.write(180 - pos);
    
    //debug stuff
    //Serial.begin(9600); //what is this?
    //Serial.print("setup");
    //std::cout << "setup" << std::endl;
    ////Serial.println(pos);
    */
    /*
    stepper1.setMaxSpeed(2000.0);

    //stepper1.setAcceleration(100.0);
    stepper1.setMinPulseWidth(10);
    stepper1.setAcceleration(50.0);

    
    stepper1.moveTo(1000);
    //stepper1.setSpeed(2000);
    */
    }

void loop() {
    //for each stepper verify if position is reached
    //if not, run stepper

    
    for (int i = 0; i < 5; i++) {
        if(stepper_motors[i].run())  {
            //Serial.println("stepper running");
        }
    }
    
    
    /*
    if (!stepper1.run()){
        stepper1.moveTo(-stepper1.currentPosition());
        //stepper1.setSpeed(2000);
    }    
    */
    //ros node stuff
    nh.spinOnce();
    //delay(1);
}
