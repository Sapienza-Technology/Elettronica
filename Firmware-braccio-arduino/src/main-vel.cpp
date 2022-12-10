#include "main-vel.h"
//https://hackaday.io/project/183279-accelstepper-the-missing-manual/details

ros::NodeHandle nh;
//number of joints and which joint is a servo motor
#define N_JOINT 6
#define SERVO_IDX 4

//end effector parameters
//TODO: find correct number
#define ENDSTOP_M_A 800
#define ENDSTOP_M_B 2300

//arm servo motor parameters
//TODO: check this stuff
#define ENDSTOP_A 500
#define ENDSTOP_B 2500

// i2C OLED - for encoder
#define I2C_ADDRESS 0x3C ;
#define RST_PIN -1;

//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value
float startAngle = 0; //starting angle
float totalAngle = 0; //total absolute angular displacement
float previoustotalAngle = 0; //for the display printing
unsigned long T;
float DT=0;


//res= step/n_revolution
float stepper_resolution=1.8;
float microstep=1;


int step_per_rev=1600;

//check if resolution is different for each stepper

//TODO: check sintax
//list of stepper motors
AccelStepper stepper_motors[5];
int motor_dir[]={1,-1,1,1,1,1};
int motor_reduction[]={20,20,(int) 50*7/4,1,1,1};
float motor_speed_coeff[]={1.0,1.0,1.0,1.0,1.0,1.0};
//velocities and position to reach for the arm
float targetVelocities[6];
float targetPositions[6];
long currentPositions[6];
float EEPos=0;

// Determine which joints to move
int joint_flags[] = {1, 1, 1, 1, 1, 1};

int servo_delay=100;
int servo_target_pos=0;
int servo_mov=0;
int servo_current_pos=0;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
int interval = 100;
int servo_pos = 0;

int mini_servo_delay=5;
int mini_servo_target_pos=0;
int mini_servo_mov=0;
int mini_servo_current_pos=0;
//pirulatore: 30 ° chiuso, 200° aperto, delay 20

int servo_start=0;

int pinza_target_pos=0;

float q_des = 67;
float q = 0;
float q_err = 0;
float vel_des = 0;
float vel = 0;
float K = 0.2;


std_msgs::Float32MultiArray posMultiArray;

int angleToStep(float angle){
  //convert the desired angle for the stepper motor into
  // the number of steps to reach the desired position
  angle=angle/PI *180.0;
  return (int)(angle/stepper_resolution*microstep);
}

float stepToAngle(int step){
  //convert the number of steps into the angle
  
  return (float)(step*stepper_resolution/microstep)/180*PI;
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
/*
ros::Subscriber<std_msgs::Float32MultiArray> armPosSub("firmware_arm_pos", setTargetPos_cb);
ros::Subscriber<std_msgs::Float32MultiArray> armVelSub("firmware_arm_vel", setTargetVel_cb);
ros::Subscriber<std_msgs::Float32> EEPSub("firmware_EEP_pos", pirulatore_cb);
ros::Subscriber<std_msgs::Float32> EEMSub("firmware_EEM_pos", pinza_cb);
ros::Publisher posPub("arm_pos_feedback", &posMultiArray);

//create and send string trhugh publisher 
std_msgs::String msg;
ros::Publisher chatter("chatter", &msg);
*/

//ros::Publisher posPub=nh.advertise<std_msgs::Float32MultiArray>("arm_pos_feedback", 1000);

//received a desired position for the armstepper5
void setTargetPos_cb(const std_msgs::Float32MultiArray& cmd) {
    //digitalWrite(LED_BUILTIN, HIGH);
    //sleep
    ////delay(100);
    //printf("received data:");

    //extract data from message
    int i = 0;

    for (i=0; i<N_JOINT; i++){
        //digitalWrite(LED_BUILTIN, HIGH);
        //delay(500);
        //std::cout << cmd << std::endl;
        //Serial.println(cmd.data[i]);
        //printf("%f ", cmd.data[i]);
        if(i==SERVO_IDX){
            //radians to angle
            targetPositions[i]=cmd.data[i]/PI*180;
            if(servo_start==0 && targetPositions[SERVO_IDX]!=0) servo_start=1;
        }
        else{
            //targetPositions[i]=angleToStep(cmd.data[i]);
            targetPositions[i]=angleToStep(cmd.data[i]);
        }
        //digitalWrite(LED_BUILTIN, LOW);
        //delay(500);
    }
    
    /*
    for (i=N_JOINT; i<N_JOINT*2; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
        joint_flags[i-N_JOINT] = (int) cmd.data[i];
    }
    
    //delay(500);
    //set desired position

    //digitalWrite(LED_BUILTIN, HIGH);
    //delay(100);

    for (int j = 0; j < N_JOINT; j++) {
        if (joint_flags[j] == 0) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000);
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
        }
        if (joint_flags[j] == 1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(250);
            digitalWrite(LED_BUILTIN, LOW);
            delay(250);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(250);
            digitalWrite(LED_BUILTIN, LOW);
            delay(250);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(250);
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
        }
    }
    */

    if (joint_flags[0]) stepper1.moveTo(targetPositions[0]*motor_reduction[0]);
    if (joint_flags[1]) stepper2.moveTo(targetPositions[1]*motor_reduction[1]);
    if (joint_flags[2]) stepper3.moveTo(targetPositions[2]*motor_reduction[2]);
    if (joint_flags[3]) stepper4.moveTo(targetPositions[3]*motor_reduction[3]);
    if (joint_flags[5]) stepper5.moveTo(targetPositions[5]*motor_reduction[5]);
    //stepper1.moveTo(200);
    //stepper2.moveTo(200);
    //stepper3.moveTo(200);
    //stepper4.moveTo(200);
    //stepper5.moveTo(200);

    //digitalWrite(LED_BUILTIN, LOW);
    //delay(100);

    //float vel= (float)targetVelocities[i];
    //if (vel) stepper_motors[i].setSpeed(vel);

    //S.write(targetPositions[SERVO_IDX]);
    if (joint_flags[4]) servo_target_pos=targetPositions[SERVO_IDX];


}

//received a desired velocity for the arm
void setTargetVel_cb(const std_msgs::Float32MultiArray& cmd) {
    //extract data from message
    for (int i=0; i<N_JOINT; i++){
        targetVelocities[i]=cmd.data[i];
    }

    //set desired position
    for(int i=0; i<N_JOINT; i++){
        if (i != 4) {
            if (targetVelocities[i]) {
                float vel= (float)targetVelocities[i];
                if (vel>0){
                    //set desired velcoity for stepper
                    stepper_motors[i].setSpeed(vel);
                }
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
    int pos=(int)fmap(EEPos, 0, 1, 0, 200);
    //EE1.write(pos);
    mini_servo_target_pos=pos;
}

//received a desired position for the end effector 0 (close) 1 (open)
void pinza_cb(const std_msgs::Float32& cmd) {
    //extract data from message
    EEPos=cmd.data;
    //move servo
    int pos= (int) fmap(EEPos, 0, 1, 0, 90);
    //EE1.write(pos);
    //EE2.write(180 - pos);
    mini_servo_target_pos=pos;
}

void setup() {
    //Serial.begin(9600); //start serial - tip: don't use serial if you don't need it (speed considerations)
    //Serial.println("Antonio Bozza>>>>>");
    Wire.begin(); //start i2C  
    Wire.setClock(800000L); //fast clock

    checkMagnetPresence(); //check the magnet (blocks until magnet is found)

    ReadRawAngle(); //make a reading so the degAngle gets updated
    startAngle = degAngle; //update startAngle with degAngle - for taring
    
    ////Serial.println("pierino");
    // ROS
    //int maxSpeed=50;
    //float acceleration=100;

    /*
    stepper1.setMinPulseWidth(10);
    stepper1.setMaxSpeed(200);
    stepper1.setAcceleration(200.0);

    stepper2.setMinPulseWidth(10);
    stepper2.setMaxSpeed(400);
    stepper2.setAcceleration(400.0);

    stepper3.setMinPulseWidth(10);
    stepper3.setMaxSpeed(3200);
    stepper3.setAcceleration(3200.0);

    stepper4.setMinPulseWidth(10);
    stepper4.setMaxSpeed(100);
    stepper4.setAcceleration(50.0);

    stepper5.setMinPulseWidth(10);
    stepper5.setMaxSpeed(50);
    stepper5.setAcceleration(10.0);
    */

    stepper2.setMinPulseWidth(10);
    stepper2.setMaxSpeed(800);
    //stepper2.setSpeed(2000);


    S.attach(12,ENDSTOP_A,ENDSTOP_B);
    servo_pos=S.read();
    // HO CAMBIATO IL PIN DEL SERVINO DA 13 A 14 PERCHÉ 13 È QUELLO DEL BUILTIN LED CHE USO PER DEBUG
    // SE NON FUNZIONA RICAMBIARE
    s1.attach(14, ENDSTOP_M_A, ENDSTOP_M_B); //destra guardando i motori
    s2.attach(30, ENDSTOP_M_A, ENDSTOP_M_B); //sinistra guardando i motori
    //s1.write(180);
    //S.write(servo_target_pos);
    //s1.write(pinza_target_pos);
    //s2.write(180 - pinza_target_pos);
    
    //stepper5.setSpeed(400);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    //debug print teensy
    //posMultiArray.data=array_setup;
    //std::cout << "Hello world!" << std::endl;

    //nh.getHardware()->setBaud(115200);
    /*
    nh.initNode();
    nh.subscribe(armPosSub);
    nh.subscribe(armVelSub);
    nh.subscribe(EEPSub);
    nh.subscribe(EEMSub);
    nh.advertise(posPub);
    nh.advertise(chatter);
    */

    //stepper1.moveTo(angleToStep(-PI/2)*motor_reduction[0]);


    float array_setup[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float array[6];

    posMultiArray.data = array;

    for (int i = 0; i < 6; i++) {
        posMultiArray.data[i] = array_setup[i];
    }

    posMultiArray.data_length = 6;
    
    //use malloc to allocate 6 float in memory
    //posMultiArray.data = (float*)malloc(6*sizeof(float));

    //copy array_setup to posMultiArray.data
    //memcpy(posMultiArray.data, array_setup, 6*sizeof(float));
    //posMultiArray.data=&array_setup;
        //append to array
        //posMultiArray.data.(array_setup[i]);

    //free memory
    //free(posMultiArray.data);



    //s1.attach(SERVO0, ENDSTOP_A, ENDSTOP_B);
    
    //s1.write(0);
    ////delay(1000);
    //s1.write(90);
    /*
    stepper_motors[0]=stepper1;
    stepper_motors[1]=stepper2;
    stepper_motors[2]=stepper3;
    stepper_motors[3]=stepper4;
    stepper_motors[4]=stepper5;
    */

   /*
    //set max speed for stepper motors
    for(int i=0; i<5; i++){
        stepper_motors[i].setMaxSpeed(2000.0);
        //stepper_motors[i].setAcceleration(100.0);
        stepper_motors[i].setMinPulseWidth(10);

        //stepper_motors[i].moveTo(0);
        //stepper_motors[i].setAcceleration(50.0);
    }
    */
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
   //stepper2.moveTo(0);
   //s1.write(0);
   
    //int t=5;
    //get position of servo
    //s1.write(180);
    //s2.write(0);
    
    //while(1){
    //S.write(0);
    
    //apri pinza
    int t=5;
    //while(t--){

 /*    for(int i=0; i<80;i++) {
        //S.write(i);
        s2.write(i+5);
        s1.write(180-i); 
        delay(20);
    }
    
    //chiudi pinza
    for(int i=80; i>0;i--) {
        //S.write(i);
        s2.write(i+5);
        s1.write(180-i);
        delay(20);
    } */
    
    
        
    //stepper4.moveTo(angleToStep(0.17));
    //S.write(110);

    //stepper1.moveTo(angleToStep(0.34)*motor_reduction[0]);
    
    }

void loop() {
    //for each stepper verify if position is reached
    //if not, run stepper
    //move servo with //delay
    

    
    currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        if(servo_start){
        if (servo_pos < targetPositions[SERVO_IDX]) {
            S.write(servo_pos);
            servo_pos++;
            
        }
        if (servo_pos > targetPositions[SERVO_IDX]) {
            S.write(servo_pos);
            servo_pos--;
        }
        }
        previousMillis = currentMillis;
    }


    /*
    if (currentMillis - previousMillis >= interval) {
        if (servo_pos > 0) {
            S.write(servo_pos);
            servo_pos--;
            
        }
        previousMillis = currentMillis;
    }
    */
    

    //S.write(0);
        
    
        
    //stepper2.setSpeed(1600);
    //stepper2.runSpeed();
    
    //move motors
    //move servo to 200
    
    /*
    if (joint_flags[0]) stepper1.run();
    if (joint_flags[1]) stepper2.run();
    if (joint_flags[2]) stepper3.run();
    if (joint_flags[3]) stepper4.run();
    if (joint_flags[5]) stepper5.run();
    */

    ReadRawAngle(); //ask the value from the sensor
    correctAngle(); //tare the value
    checkQuadrant(); //check quadrant, check rotations, check absolute angular position
    DT=(millis()-T)/1000.0;
    T=millis();



    vel = vel_des + K*(q_des-totalAngle);

    vel = (int)((vel/stepper_resolution)*microstep*motor_reduction[1]);

    if (vel > 100*motor_reduction[1]) {
        vel = 100*motor_reduction[1]; 
    }

    /*
    Serial.print("Angle: ");
    Serial.println(totalAngle);

    Serial.print("Velocity: ");
    Serial.println(vel);
    */

    stepper2.setSpeed(vel);
    stepper2.runSpeed();

    //stepper4.setSpeed(-800);
    //stepper4.runSpeed();
    //stepper2.setSpeed(1600);
    //stepper2.runSpeed();

    //stepper5.setSpeed(400);
    //stepper5.runSpeed();



    //move servo
    //move pirulatore with delay
    // DECOMMENTA PER FAR FUNZIONARE I SERVIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
    
    if(mini_servo_mov%mini_servo_delay==0){
        if(mini_servo_current_pos>mini_servo_target_pos){
            mini_servo_current_pos--;
        }
        else if(mini_servo_current_pos<mini_servo_target_pos){
            mini_servo_current_pos++;
        }
        s2.write(mini_servo_current_pos);
        //only if pinza connected
        s1.write(180 - mini_servo_current_pos);
    }
    if (mini_servo_current_pos!=mini_servo_target_pos){
        mini_servo_mov++;
    }



    /*
    if(servo_mov%servo_delay==0){
        if(servo_current_pos>servo_target_pos){
            servo_current_pos--;
        }
        else if(servo_current_pos<servo_target_pos){
            servo_current_pos++;
        }
        //servo.write(servo_current_pos);
    }
    if (servo_current_pos!=servo_target_pos){
        servo_mov++;
    }
    */


    //add positions to current positions
    posMultiArray.data[0] = (float) stepToAngle(stepper1.currentPosition())/motor_reduction[0];
    posMultiArray.data[1] = (float) stepToAngle(stepper2.currentPosition())/motor_reduction[1];
    posMultiArray.data[2] = (float) stepToAngle(stepper3.currentPosition())/motor_reduction[2];
    posMultiArray.data[3] = (float) stepToAngle(stepper4.currentPosition())/motor_reduction[3];
    posMultiArray.data[4] = (float) stepToAngle(stepper5.currentPosition())/motor_reduction[5];
    
    //posPub.publish(&posMultiArray);

    //delay without pause


    

    /*
    if (!stepper1.run()){
        stepper1.moveTo(-stepper1.currentPosition());
        //stepper1.setSpeed(2000);
    }    
    */
    //ros node stuff
    
    //nh.spinOnce();
    ////delay(100);
}


void ReadRawAngle()
{ 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625; 
  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  
}

void correctAngle()
{
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  //Serial.print("Corrected angle: ");
  //Serial.println(correctedAngle, 2); //print the corrected/tared angle  
}

void checkQuadrant()
{
  /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */

  //Quadrant 1
  if(correctedAngle >= 0 && correctedAngle <=90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if(correctedAngle > 90 && correctedAngle <=180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if(correctedAngle > 180 && correctedAngle <=270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if(correctedAngle > 270 && correctedAngle <360)
  {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  
  }  
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  //Serial.println("Magnet found!");
  delay(1000);  
}