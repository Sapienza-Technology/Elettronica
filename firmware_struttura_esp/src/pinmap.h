/*
DRV_X_Y_Z:
X= FRONT(F), REAR(R), MIDDLE(M)
Y= LEFT(L), RIGHT(R)
Z=CW CCW S (Speed)
*/
//front
#define DRV_F_L_CW 2
#define DRV_F_L_CCW 3
#define DRV_F_L_IDX 7


#define DRV_F_R_CW 4
#define DRV_F_R_CCW 5
#define DRV_F_R_IDX 2

//rear
#define DRV_R_L_CW 10
#define DRV_R_L_CCW 11
#define DRV_R_L_IDX 5

#define DRV_R_R_CW 12
#define DRV_R_R_CCW 13
#define DRV_R_R_IDX 6

//middle
#define DRV_M_L_CW 6
#define DRV_M_L_CCW 7
#define DRV_M_L_IDX 3

#define DRV_M_R_CW 8
#define DRV_M_R_CCW 9
#define DRV_M_R_IDX 8

//front
#define STEER_F_L_S 1
#define STEER_F_L_D 45

#define STEER_F_R_S 9
#define STEER_F_R_D 48

//rear
#define STEER_R_L_S 10
#define STEER_R_L_D 40

#define STEER_R_R_S 4
#define STEER_R_R_D 42

//DRILLING

//dc motor to rotate
#define DRILL_CW 14
#define DRILL_CCW 18
#define DRILL_IDX 15



//stepper motor to move up and down
#define DRILL_STEP 15
#define DRILL_DIR 21

#define DRILL_STEP2 16
#define DRILL_DIR2 17
