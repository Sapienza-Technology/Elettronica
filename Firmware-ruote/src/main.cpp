#include "main.h"

ros::NodeHandle nh;

#define MAX_V 7.0

void right_cb(const std_msgs::Float32& cmd) {
    setRightSpeed(cmd.data);
}

void left_cb(const std_msgs::Float32& cmd) {
    setLeftSpeed(cmd.data);
}

ros::Subscriber<std_msgs::Float32> rightSub("destra", right_cb);
ros::Subscriber<std_msgs::Float32> leftSub("sinistra", left_cb);

void setup() {
    // Teensy LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Signal tower
    pinMode(LIGHT_DNG, OUTPUT);
    digitalWrite(LIGHT_DNG, HIGH);
    pinMode(LIGHT_WRN, OUTPUT);
    digitalWrite(LIGHT_WRN, LOW);
    pinMode(LIGHT_OK, OUTPUT);
    digitalWrite(LIGHT_OK, LOW);

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

    pinMode(DRV2_A1, OUTPUT);
    digitalWrite(DRV2_A1, LOW);
    pinMode(DRV2_A2, OUTPUT);
    digitalWrite(DRV2_A2, LOW);
    pinMode(DRV2_B1, OUTPUT);
    digitalWrite(DRV2_B1, LOW);
    pinMode(DRV2_B2, OUTPUT);
    digitalWrite(DRV2_B2, LOW);
    //pinMode(DRV2_E, OUTPUT);
    //digitalWrite(DRV2_E, LOW);

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
        analogWrite(DRV1_A2, 0);
        analogWrite(DRV1_A1, 0);
        analogWrite(DRV1_B2, 0);
        analogWrite(DRV1_B1, 0);
    }
    boolean dir = sgn(x);
    int pwm = fmap(vel, 0, MAX_V, 0, 255);
    if (dir == true) {
        analogWrite(DRV1_A2, 0);
        analogWrite(DRV1_A1, pwm);
        analogWrite(DRV1_B2, 0);
        analogWrite(DRV1_B1, pwm);
    } else {
        analogWrite(DRV1_A1, 0);
        analogWrite(DRV1_A2, pwm);
        analogWrite(DRV1_B1, 0);
        analogWrite(DRV1_B2, pwm);
    }
}

void setRightSpeed(float x) {
    float vel = abs(x);
    if (vel < 0.01) {
        analogWrite(DRV2_A2, 0);
        analogWrite(DRV2_A1, 0);
        analogWrite(DRV2_B2, 0);
        analogWrite(DRV2_B1, 0);
    }
    boolean dir = sgn(x);
    int pwm = (int) fmap(vel, 0.0, MAX_V, 0, 255);
    if (dir == true) {
        analogWrite(DRV2_A2, 0);
        analogWrite(DRV2_A1, pwm);
        analogWrite(DRV2_B2, 0);
        analogWrite(DRV2_B1, pwm);
    } else {
        analogWrite(DRV2_A1, 0);
        analogWrite(DRV2_A2, pwm);
        analogWrite(DRV2_B1, 0);
        analogWrite(DRV2_B2, pwm);
    }
}
