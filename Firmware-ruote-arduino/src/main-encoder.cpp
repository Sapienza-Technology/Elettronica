#include "main.h"
#include <Encoder.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
ros::NodeHandle nh;

#define MAX_V 10.0

//encoders
//L=left R=right F=front B=back M=middle
Encoder wheel_LF(ENC_LF_A, ENC_LF_B);
Encoder wheel_LB(ENC_LB_A, ENC_LB_B);
Encoder wheel_LM(ENC_LM_A, ENC_LM_B);
Encoder wheel_RF(ENC_RF_A, ENC_RF_B);
Encoder wheel_RB(ENC_RB_A, ENC_RB_B);
Encoder wheel_RM(ENC_RM_A, ENC_RM_B);

std_msgs::Float32MultiArray posMultiArray;

//if two interrupt pins=4, else is 2
//a complete rotation is encoder_resolution*encoder_multiplier
int encoder_std_multiplier=2;
int encoder_interrupt_multiplier=2;
int encoder_multiplier=encoder_std_multiplier*encoder_interrupt_multiplier;
float enc_res=356.3;

void cb2(const std_msgs::Float32& cmd) {
    setLeftSpeed(cmd.data);
}

void cb1(const std_msgs::Float32& cmd) {
    setRightSpeed(cmd.data);
}

ros::Subscriber<std_msgs::Float32> rightSub("destra", cb1);
ros::Subscriber<std_msgs::Float32> leftSub("sinistra", cb2);
ros::Publisher posPub("wheel_encoder_feedback", &posMultiArray);


float convert_enc_value(int enc_value){
    return ((enc_value/encoder_multiplier)/enc_res )*360;
}

void setup() {
    // Teensy LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Motor drivers
    pinMode(DRV1_A1, OUTPUT);
    digitalWrite(DRV1_A1, LOW);
    pinMode(DRV1_A2, OUTPUT);
    digitalWrite(DRV1_A2, LOW);
    pinMode(DRV1_B1, OUTPUT);
    digitalWrite(DRV1_B1, LOW);
    pinMode(DRV1_B2, OUTPUT);
    digitalWrite(DRV1_B2, LOW);

    
    float array_setup[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float array[6];

    posMultiArray.data = array;
    
    for (int i = 0; i < 6; i++) {
        posMultiArray.data[i] = array_setup[i];
    }

    posMultiArray.data_length = 6;



    // ROS
    nh.initNode();
    nh.subscribe(rightSub);
    nh.subscribe(leftSub);
    nh.advertise(posPub);
}

void loop() {
    nh.spinOnce();
    posMultiArray.data[0] = convert_enc_value(wheel_LF.read());
    posMultiArray.data[1] = convert_enc_value(wheel_LM.read());
    posMultiArray.data[2] = convert_enc_value(wheel_LB.read());
    posMultiArray.data[3] = convert_enc_value(-wheel_RF.read());
    posMultiArray.data[4] = convert_enc_value(-wheel_RM.read());
    posMultiArray.data[5] = convert_enc_value(-wheel_RB.read());
    posPub.publish(&posMultiArray);

}

void setLeftSpeed(float x) {
    
    float vel = abs(x);
    if ( vel < 0.01) {
        analogWrite(DRV1_B2, 0);
        analogWrite(DRV1_B1, 0);
    }
    boolean dir = sgn(x);
    int pwm = fmap(vel, 0, MAX_V, 0, 255);
    if (dir == true) {
        analogWrite(DRV1_B1, 0);
        analogWrite(DRV1_B2, pwm);
    } else {
        analogWrite(DRV1_B2, 0);
        analogWrite(DRV1_B1, pwm);
    }

}

void setRightSpeed(float x) {
    
    float vel = abs(x);
    if (  vel < 0.01) {

        analogWrite(DRV1_A2, 0);
        analogWrite(DRV1_A1, 0);
        
    }
    boolean dir = sgn(x);
    int pwm = (int) fmap(vel, 0.0, MAX_V, 0, 255);
    if (dir == true) {
        analogWrite(DRV1_A1, 0);
        analogWrite(DRV1_A2, pwm);
    } else {
        analogWrite(DRV1_A2, 0);
        analogWrite(DRV1_A1, pwm);

    }
}
