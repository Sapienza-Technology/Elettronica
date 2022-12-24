#include "main.h"

ros::NodeHandle nh;

#define MAX_V 10.0

/*
MAX_V va settato rispetto alla velocità massima.
MAX_V è la velocità massima del motore, quindi per trovarlo usando la velocità massima faccio:
    v=w*r --> w=v/r= velocità rover/0.08
Calcolare coefficiente correttivo
*/

void cb2(const std_msgs::Float32& cmd) {
    setLeftSpeed(cmd.data);
}

void cb1(const std_msgs::Float32& cmd) {
    setRightSpeed(cmd.data);
}

ros::Subscriber<std_msgs::Float32> rightSub("destra", cb1);
ros::Subscriber<std_msgs::Float32> leftSub("sinistra", cb2);

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
    //pinMode(DRV1_E, OUTPUT);
    //digitalWrite(DRV1_E, LOW);

    // ROS
    nh.initNode();
    nh.subscribe(rightSub);
    nh.subscribe(leftSub);
}

void loop() {
    nh.spinOnce();
    delay(1);
}

void setLeftSpeed(float x) {
    
    float vel = abs(x);
    if (vel < 0.01) {
        analogWrite(DRV1_B2, 0);
        analogWrite(DRV1_B1, 0);
        return;
    }
    boolean dir = sgn(x);
    int pwm = fmap(vel, 0, MAX_V, 0, 255);
    pwm=255;
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
    if (vel < 0.01) {
        analogWrite(DRV1_A2, 0);
        analogWrite(DRV1_A1, 0);  
        return;
    }
    boolean dir = sgn(x);
    int pwm = (int) fmap(vel, 0.0, MAX_V, 0, 255);
    pwm=255;
    if (dir == true) {
        analogWrite(DRV1_A1, 0);
        analogWrite(DRV1_A2, pwm);
    } else {
        analogWrite(DRV1_A2, 0);
        analogWrite(DRV1_A1, pwm);
    }
}
