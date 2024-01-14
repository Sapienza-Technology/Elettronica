//#include<Wire.h>
#include<AS5600.h>    // libreria: robtillaart/AS5600@^0.4.1           
//#include<HardwareSerial.h> //necessario per l'esp a quanto pare
//#include<TCA9548.h>   // libreria: robtillaart/TCA9548@^0.1.5 (il bro si diverte un sacco con queste schede)
#include <SimpleKalmanFilter.h>// boh, l'ho messa solo per provare: letteralmente la prima che mi è uscita dalla ricerca su Platformio
//https://hackaday.io/project/183279-accelstepper-the-missing-manual/details
#include <Arduino.h>
//#include "config.h"
#include "pinmap.h"
//#include <ros.h>
#include "utils.h"
//#include <std_msgs/UInt16.h>
//#include <std_msgs/Float32.h>
//#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/String.h>

#include <AccelStepper.h>
//#include <elapsedMillis.h>

//void setTargetPos_cb(const std_msgs::Float32MultiArray& cmd);
//void setTargetVel_cb(const std_msgs::Float32MultiArray& cmd);
//void pirulatore_cb(const std_msgs::Float32& cmd);
//void pinza_cb(const std_msgs::Float32& cmd);
int angleToStep(float angle);

float S1,S2,S3;
float estdeg;
float estrad;
float command_speed;
float K=0.5;
float eststep;
//#define ch TCA9548
SimpleKalmanFilter kalmanrot(1,1,0.1);


//TCA9548 MP(0x70); //0x70 è l'indirizzo della TCA
AS5600 as5600;

//ros::NodeHandle nh;
//number of joints and which joint is a servo motor
#define N_JOINT 6

class MyStepper{
    public:
    double target_position;
    double target_vel=0;
    int stp_pin;
    int dir_pin;
    float stepper_resolution;
    int microstep;
    float motor_reduction;
    float max_speed;
    float acceleration;
    float min_pulse_width;
    int channel_number;
    AccelStepper stepper;

    long int angleToStep(float angle){
        //convert the desired angle for the stepper motor into
        // the number of steps to reach the desired position
        angle=angle/PI *180.0;
        return (long int)(((angle/stepper_resolution)*microstep)*motor_reduction);
    }

    MyStepper(int stp_pin, int dir_pin, float stepper_resolution, int microstep, float motor_reduction, float max_speed, float acceleration, float min_pulse_width, int channel_number){
        //pinMode(stp_pin, OUTPUT);
        //pinMode(dir_pin, OUTPUT);
        //stepper=AccelStepper(AccelStepper::DRIVER, stp_pin, dir_pin);

        this->stepper_resolution=stepper_resolution;
        this->microstep=microstep;
        this->motor_reduction=motor_reduction;
        this->channel_number=channel_number;
        int speedMultiplier=1;

        //stepper.setMinPulseWidth(min_pulse_width);
        float max_step_speed=angleToStep(max_speed);
        //stepper.setMaxSpeed(max_step_speed*speedMultiplier);
        
        //stepper.setSpeed(angleToStep(PI/36));

        float acceleration_steps=angleToStep(acceleration);
        //stepper.setAcceleration(acceleration_steps);
    }



    float stepToAngle(int step){
        //convert the number of steps into the angle
        
        return (float)(step*stepper_resolution/microstep)/(180*motor_reduction)*PI;
    }

    void setPos(float angle){
        //set the desired position for the stepper motor
        //angle is in radians
        target_position=angleToStep(angle);
    }

    void setVel(float vel){
        target_vel=angleToStep(vel);
        //stepper.setSpeed(target_vel);

    }

    void setPos_stepper(float pos){
        //set the desired position for the stepper motor
        //angle is in radians
        target_position=pos/0.1016;
        //stepper.moveTo(target_position);
    }
    

    void run() {
        //MP.selectChannel(channel_number);  //la funzione selectChannel() attiva solo il canale richiesto e disattiva automaticamente tutti gli altri
        S1=as5600.readAngle()*AS5600_RAW_TO_DEGREES; // costante di conversione della libreria in gradi, disponibile anche in radianti
        estdeg=kalmanrot.updateEstimate(S1);
        estrad = estdeg*PI/180;
        eststep = angleToStep(estrad);

        command_speed = (10000 - eststep)*motor_reduction*K;
        //stepper.setSpeed(command_speed);

        //stepper.runSpeed();

        Serial.println("Angle estimation");
        Serial.print(estdeg);
        Serial.print("°  \n");
        Serial.println("Command speed");
        Serial.print(command_speed);
        Serial.print("step/s  \n");
        Serial.println("Error angle");
        Serial.print(stepToAngle((10000 - eststep)));
        Serial.print("°\n");
    }

    

};

//res= step/n_revolution
float stepper_resolution=1.8;
float motor_reduction[]={60,30,20,5,1.43,1.74,1};
int microstep[]={8,8,8,8,8,8,8};
//velocities and position to reach for the arm
float targetVelocities[6];
float targetPositions[6];
float gripperPosition;
//long currentPositions[6];


//previously acceleration was set to 7200, now setting in rad/s^2 (considering reduction and microsteps)
//same for maxSpeed
//arguments: (stp_pin, dir_pin, motor_resolution, motor_microsteps, motor_reduction, max_speed, acceleration, min_pulse_width)
MyStepper stepper1(STP0, DIR0, stepper_resolution, microstep[0], motor_reduction[0], 2*PI, PI, 20, 1);
//MyStepper stepper2(STP1, DIR1, stepper_resolution, microstep[1], motor_reduction[1], 2*PI, PI, 20, 2);
//MyStepper stepper3(STP2, DIR2, stepper_resolution, microstep[2], motor_reduction[2], 2*PI, PI, 20, 3);
//MyStepper stepper4(STP3, DIR3, stepper_resolution, microstep[3], motor_reduction[3], 2*PI, PI, 20, 4);
//MyStepper stepper5(STP4, DIR4, stepper_resolution, microstep[4], motor_reduction[4], 2*PI, PI, 20, 5);
//MyStepper stepper6(STP5, DIR5, stepper_resolution, microstep[5], motor_reduction[5], 2*PI, PI, 20, 6);
//MyStepper stepper7(STP6, DIR6, stepper_resolution, microstep[6], motor_reduction[6], PI/2, PI, 20, 7);

//array of stepper motors
//MyStepper steppers[]={stepper1, stepper2, stepper3, stepper4,stepper5, stepper6};

//suR5cribe to ros topic
//ros::Subscriber<std_msgs::Float32MultiArray> armPosSub("firmware_arm_pos", setTargetPos_cb);
//ros::Subscriber<std_msgs::Float32MultiArray> armVelSub("firmware_arm_vel", setTargetVel_cb);
//ros::Subscriber<std_msgs::Float32> armPinzaSub("firmware_arm_pinza", pinza_cb);
//
//received a desired position for the armstepper5
//void setTargetPos_cb(const std_msgs::Float32MultiArray& cmd) {
//    //extract data from message
//    for (int i=0; i<N_JOINT; i++){
//        //digitalWrite(LED_BUILTIN, HIGH);
//        //delay(100);
//        targetPositions[i]=cmd.data[i];    
//        //digitalWrite(LED_BUILTIN, LOW);
//        //delay(100);
//    }
//
//
//    stepper1.setPos(targetPositions[0]);
//    stepper2.setPos(targetPositions[1]);
//    stepper3.setPos(targetPositions[2]);
//    stepper4.setPos(targetPositions[3]);
//    stepper5.setPos(targetPositions[4]);
//    stepper6.setPos(targetPositions[5]);
//
//
//}
//
////received a desired velocity for the arm
//void setTargetVel_cb(const std_msgs::Float32MultiArray& cmd) {
//    //extract data from message
//    for (int i=0; i<N_JOINT; i++){
//        targetVelocities[i]=cmd.data[i];
//    }
//
//    //set desired position
//    for(int i=0; i<N_JOINT; i++){
//        //if (targetVelocities[i]) {
//            float vel = (float)targetVelocities[i];
//            steppers[i].setVel(vel);
//        //}
//    }
//}
//
////received a desired position for the armstepper5
//void pinza_cb(const std_msgs::Float32& cmd) {
//    //extract data from message
//    gripperPosition = cmd.data;
//
//    stepper7.setPos_stepper(gripperPosition);
//}


void setup() {
    S1,S2,S3=0;
    Serial.begin(9600);
    Serial.println("Ciaooooooo\n");
    Wire.begin(); // specify SDA and SCL pins

    as5600.begin();  // pin dichiarati per lo stesso motivo di cui sopra
    as5600.setDirection(0); //lettura con incremento in senso orario
    delay(50);
    //nh.getHardware()->setBaud(115200);
    //nh.initNode();
    //nh.subscribe(armPosSub);
    //nh.subscribe(armVelSub);
    //nh.subscribe(armPinzaSub);
    }

void loop() {
    //run the stepper motors

    stepper1.run();
    delay(5);
    //stepper2.run();
    //delay(5);
    //stepper3.run();
    //delay(5);
    //stepper4.run();
    //delay(5);
    //stepper5.run();
    //delay(5);
    //stepper6.run();
    //delay(5);
    //stepper7.run();
    //delay(5);

    //Serial.print(estdeg); // il primo dato l'ho fatto passare per kalman per vedere la differenza con la lettura cruda
    //Serial.print("°   ");
    //Serial.print(S2);
    //Serial.print("°   ");
    //Serial.print(S3);
    //Serial.print("°\n"); //non il modo migliore per stampare, ma si fa quel che si può
    
    //delay(10);

    /*
    //add positions to current positions
    posMultiArray.data[0] = (float) stepToAngle(stepper1.currentPosition())/motor_reduction[0];
    posMultiArray.data[1] = (float) stepToAngle(stepper2.currentPosition())/motor_reduction[1];
    posMultiArray.data[2] = (float) stepToAngle(stepper3.currentPosition())/motor_reduction[2];
    posMultiArray.data[3] = (float) stepToAngle(stepper4.currentPosition())/motor_reduction[3];
    posMultiArray.data[4] = (float) stepToAngle(stepper5.currentPosition())/motor_reduction[5];
    
    posPub.publish(&posMultiArray);
    */
    
    //nh.spinOnce();
    ////delay(100);
}
