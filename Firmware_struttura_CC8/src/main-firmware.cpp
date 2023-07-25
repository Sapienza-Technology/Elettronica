#include "main.h"

ros::NodeHandle nh;

//front
Wheel* wheel_FL;
Wheel* wheel_FR;

//rear
Wheel* wheel_RL;
Wheel* wheel_RR;

//middle
Wheel* wheel_ML;
Wheel* wheel_MR;

//Steering wheels
SteeringWheel* steering_FL;
SteeringWheel* steering_FR;
SteeringWheel* steering_RL;
SteeringWheel* steering_RR;

//array with all the wheels
Wheel* wheels[6] = {wheel_FL, wheel_FR, wheel_RL, wheel_RR, wheel_ML, wheel_MR};
SteeringWheel* steering[4] = {steering_FL, steering_FR, steering_RL, steering_RR};

//time variable, if too much time passes without receiving a message from ROS, the robot stops
unsigned long lastMsgTime = 0;

//a float32 multiarray is received from ROS
void vel_cb(const std_msgs::Float32MultiArray& cmd) {

  lastMsgTime = millis();

  int len = cmd.data_length;
  if (len!=10) {
    return;
  }
  for (int i=0; i<6; i++) {
    wheels[i]->setSpeed(cmd.data[i]);
  }
  for (int i=6; i<10; i++) {
    steering[i-6]->moveToAngle(cmd.data[i]);
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> velSub("wheel_velocities", vel_cb);


void setup() {

  //front
  wheel_FL = new Wheel(DRV_F_L_F, DRV_F_L_B);
  wheel_FR = new Wheel(DRV_F_R_F, DRV_F_R_B);

  //rear
  wheel_RL = new Wheel(DRV_R_L_F, DRV_R_L_B);
  wheel_RR = new Wheel(DRV_R_R_F, DRV_R_R_B);

  //middle
  wheel_ML = new Wheel(DRV_M_L_F, DRV_M_L_B);
  wheel_MR = new Wheel(DRV_M_R_F, DRV_M_R_B);


  //Steering wheels
  steering_FL = new SteeringWheel(STEER_F_L_S, STEER_F_L_D);
  steering_FR = new SteeringWheel(STEER_F_R_S, STEER_F_R_D);
  steering_RL = new SteeringWheel(STEER_R_L_S, STEER_R_L_D);
  steering_RR = new SteeringWheel(STEER_R_R_S, STEER_R_R_D);

  // ROS
  nh.initNode();
  nh.subscribe(velSub);

}

void loop() {
    nh.spinOnce();
    delay(1);

    //if too much time passes without receiving a message from ROS, the robot stops
    if (millis() - lastMsgTime > 1000) {
      for (int i=0; i<6; i++) {
        wheels[i]->setSpeed(0);
      }
    }
    else {
      // if everything ok update steering wheels
      for (int i=0; i<4; i++) {
        steering[i]->run();
      }
    }
}
