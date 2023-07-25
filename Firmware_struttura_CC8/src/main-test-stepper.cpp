#include "main.h"


//Steering wheels
SteeringWheel* stepper;

void setup() {


  //Steering wheels
  stepper = new SteeringWheel(STEER_F_L_S, STEER_F_L_D);
  stepper->moveToAngle(degToRad(90));


}

void loop() {
    delay(1);

    stepper->run();
    
}
