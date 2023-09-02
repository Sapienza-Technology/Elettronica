#include "main.h"
//https://hackaday.io/project/183279-accelstepper-the-missing-manual/details

ros::NodeHandle nh;
//number of joints and which joint is a servo motor
#define N_JOINT 6

class MyStepper{
    private:
    double target_position;
    double target_vel;

    public:
    int stp_pin;
    int dir_pin;
    float stepper_resolution;
    int microstep;
    float motor_reduction;
    float max_speed;
    float acceleration;
    float min_pulse_width;
    AccelStepper stepper;

    int angleToStep(float angle){
        //convert the desired angle for the stepper motor into
        // the number of steps to reach the desired position
        angle=angle/PI *180.0;
        return (int)(((angle/stepper_resolution)*microstep)*motor_reduction);
    }

    MyStepper(int stp_pin, int dir_pin, float stepper_resolution, int microstep, float motor_reduction, float max_speed, float acceleration, float min_pulse_width){
        pinMode(stp_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        stepper=AccelStepper(AccelStepper::DRIVER, stp_pin, dir_pin);

        this->stepper_resolution=stepper_resolution;
        this->microstep=microstep;
        this->motor_reduction=motor_reduction;
        int speedMultiplier=1;

        stepper.setMinPulseWidth(min_pulse_width);
        float max_step_speed=angleToStep(max_speed);
        stepper.setMaxSpeed(max_step_speed*speedMultiplier);
        //stepper.setSpeed(angleToStep(PI/36));

        float acceleration_steps=angleToStep(acceleration);
        stepper.setAcceleration(acceleration_steps);
    }



    float stepToAngle(int step){
        //convert the number of steps into the angle
        
        return (float)(step*stepper_resolution/microstep)/(180*PI*motor_reduction);
    }

    void setPos(float angle){
        //set the desired position for the stepper motor
        //angle is in radians
        target_position=angleToStep(angle);
        stepper.moveTo(target_position);
    }

    void setVel(float vel){
        target_vel=angleToStep(vel);
        stepper.setSpeed(target_vel);

    }

    void run() {
        stepper.run();
    }

    

};

//res= step/n_revolution
float stepper_resolution=1.8;
float motor_reduction[]={60,20,20,5,1.5,1.5};
int microstep[]={4,4,4,4,4,4};
//velocities and position to reach for the arm
float targetVelocities[6];
float targetPositions[6];
//long currentPositions[6];


//previously acceleration was set to 7200, now setting in rad/s^2 (considering reduction and microsteps)
//same for maxSpeed
//arguments: (stp_pin, dir_pin, motor_resolution, motor_microsteps, motor_reduction, max_speed, acceleration, min_pulse_width)
MyStepper stepper1(STP0, DIR0, stepper_resolution, microstep[0], motor_reduction[0], 2*PI, PI, 20);
MyStepper stepper2(STP1, DIR1, stepper_resolution, microstep[1], motor_reduction[1], 2*PI, PI, 20);
MyStepper stepper3(STP2, DIR2, stepper_resolution, microstep[2], motor_reduction[2], 2*PI, PI, 20);
MyStepper stepper4(STP3, DIR3, stepper_resolution, microstep[3], motor_reduction[3], 2*PI, PI, 20);
MyStepper stepper5(STP4, DIR4, stepper_resolution, microstep[4], motor_reduction[4], 2*PI, PI, 20);
MyStepper stepper6(STP5, DIR5, stepper_resolution, microstep[5], motor_reduction[5], 2*PI, PI, 20);

//array of stepper motors
MyStepper steppers[]={stepper1, stepper2, stepper3, stepper4,stepper5, stepper6};

//suR5cribe to ros topic
ros::Subscriber<std_msgs::Float32MultiArray> armPosSub("firmware_arm_pos", setTargetPos_cb);
ros::Subscriber<std_msgs::Float32MultiArray> armVelSub("firmware_arm_vel", setTargetVel_cb);

//received a desired position for the armstepper5
void setTargetPos_cb(const std_msgs::Float32MultiArray& cmd) {
    //extract data from message
    for (int i=0; i<N_JOINT; i++){
        //digitalWrite(LED_BUILTIN, HIGH);
        //delay(100);
        targetPositions[i]=cmd.data[i];    
        //digitalWrite(LED_BUILTIN, LOW);
        //delay(100);
    }


    stepper1.setPos(targetPositions[0]);
    stepper2.setPos(targetPositions[1]);
    stepper3.setPos(targetPositions[2]);
    stepper4.setPos(targetPositions[3]);
    stepper5.setPos(targetPositions[4]);
    stepper6.setPos(targetPositions[5]);


}

//received a desired velocity for the arm
void setTargetVel_cb(const std_msgs::Float32MultiArray& cmd) {
    //extract data from message
    for (int i=0; i<N_JOINT; i++){
        targetVelocities[i]=cmd.data[i];
    }

    //set desired position
    for(int i=0; i<N_JOINT; i++){
        //if (targetVelocities[i]) {
            float vel= (float)targetVelocities[i];
            steppers[i].setVel(vel);
        //}
    }
}


void setup() {
   
    //nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(armPosSub);
    nh.subscribe(armVelSub);
   
    }

void loop() {
    
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    stepper5.run();
    stepper6.run();
    


    /*
    //add positions to current positions
    posMultiArray.data[0] = (float) stepToAngle(stepper1.currentPosition())/motor_reduction[0];
    posMultiArray.data[1] = (float) stepToAngle(stepper2.currentPosition())/motor_reduction[1];
    posMultiArray.data[2] = (float) stepToAngle(stepper3.currentPosition())/motor_reduction[2];
    posMultiArray.data[3] = (float) stepToAngle(stepper4.currentPosition())/motor_reduction[3];
    posMultiArray.data[4] = (float) stepToAngle(stepper5.currentPosition())/motor_reduction[5];
    
    posPub.publish(&posMultiArray);
    */
    
    nh.spinOnce();
    ////delay(100);
}
