
#ifndef PINMAP_H
#define PINMAP_H

/*
DRV_X_Y_Z:
X= FRONT(F), REAR(R), MIDDLE(M)
Y= LEFT(L), RIGHT(R)
Z=FORWARD(F), BACKWARD(B)
*/

//Wheels

//front
#define DRV_F_L_F 2
#define DRV_F_L_B 3

#define DRV_F_R_F 4
#define DRV_F_R_B 5

//rear
#define DRV_R_L_F 6
#define DRV_R_L_B 7

#define DRV_R_R_F 8
#define DRV_R_R_B 9

//middle
#define DRV_M_L_F 10
#define DRV_M_L_B 11

#define DRV_M_R_F 12
#define DRV_M_R_B 13


// Steering wheels

#define MICROSTEP 1
#define STP_RESOLUTION 1.8
#define REDUCTION_RATIO 20

/*
STEER_X_Y_Z:
X= FRONT(F), REAR(R)
Y= LEFT(L), RIGHT(R)
*/

//front
#define STEER_F_L_S 14
#define STEER_F_L_D 15

#define STEER_F_R_S 16
#define STEER_F_R_D 17

//rear
#define STEER_R_L_S 18
#define STEER_R_L_D 19

#define STEER_R_R_S 20
#define STEER_R_R_D 21








#endif