#include "config.h"
#ifndef PINMAP_H
#define PINMAP_H
#if PINMAP == 0

#define DRV1_B1 5
#define DRV1_B2 7
#define DRV1_A1 4 
#define DRV1_A2 6

//pin for encoders of six wheels
//L=left R=right F=front B=back M=middle
//interrupt 2, 3, 18, 19, 20, 21
//a in interrupt

//in encoder: 
//fila destra BLU: CH.B              VERDE: CH.A        ARANCIO: VCC        GIALLO: GND
//fila sinistra VIOLA: CH.B          GRIGIO: CH.A        NERO: VCC:          BIANCO GND

#define ENC_LF_A 19
#define ENC_LF_B 10
#define ENC_LB_A 20
#define ENC_LB_B 24
#define ENC_LM_A 18
#define ENC_LM_B 26

#define ENC_RF_A 3
#define ENC_RF_B 28
#define ENC_RB_A 21
#define ENC_RB_B 30
#define ENC_RM_A 2
#define ENC_RM_B 32


#define LIGHT_DNG 26
#define LIGHT_WRN 27
#define LIGHT_OK 28

/*

#define DRV1_AC 23
#define DRV1_BC 22
#define DRV2_AC 21
#define DRV2_BC 20

#define PWM0 24
#define PWM1 25


#define SCALE0_D 17
#define SCALE0_C 16
#define SCALE1_D 15
#define SCALE1_C 14
#define SCALE2_D 41
#define SCALE2_C 40

#define ANALOG0 39
#define ANALOG1 38

#define BTN0 37
#define BTN1 36

#define AUX0 32
#define AUX1 31
#define AUX2 30
#define AUX3 29
*/

#else
/*
 * Pinmap for the robot arm Teensy
 */
#define SERVO0 2
#define SERVO1 3

#define MSERVO0 4
#define MSERVO1 5
#define MSERVO2 6
#define STEP_EN 8

#define STP0 9
#define DIR0 10
#define STP1 11
#define DIR1 12
#define STP2 24
#define DIR2 25
#define STP3 26
#define DIR3 27
#define STP4 28
#define DIR4 29
#define ALRM 30
#define PEND 31

#define ENDSTOP0 23
#define ENDSTOP1 22
#define ENDSTOP2 21
#define ENDSTOP3 20
#define ENDSTOP4 17
#define ENDSTOP5 16
#define ENDSTOP6 15
#define ENDSTOP7 14

#define AUX4 41
#define AUX5 40
#define AUX6 39
#define AUX7 38
#define AUX8 37
#define AUX9 36
#define AUX10 35
#define AUX11 34
#endif
#endif