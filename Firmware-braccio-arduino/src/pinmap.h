#include "config.h"
#ifndef PINMAP_H
#define PINMAP_H
#if PINMAP == 0
/*
 * Pinmap for the main Teensy
 */
#define DRV1_A1 2
#define DRV1_A2 3
#define DRV1_B1 4
#define DRV1_B2 5
#define DRV1_E 6
#define DRV2_A1 7
#define DRV2_A2 8
#define DRV2_B1 9
#define DRV2_B2 10
#define DRV2_E 11

#define DRV1_AC 23
#define DRV1_BC 22
#define DRV2_AC 21
#define DRV2_BC 20

#define PWM0 24
#define PWM1 25

#define LIGHT_DNG 26
#define LIGHT_WRN 27
#define LIGHT_OK 28

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

#else
/*
 * Pinmap for the robot arm Teensy
 */
#define SERVO0 12
#define SERVO1 3

#define MSERVO0 4
#define MSERVO1 5
#define MSERVO2 6
#define STEP_EN 8

#define DIR0 2
#define STP0 3
#define DIR1 44
#define STP1 45
#define DIR2 6
#define STP2 7
#define DIR3 8
#define STP3 9
#define DIR4 10
#define STP4 11
#define DIR5 12
#define STP5 13
#define DIR6 22
#define STP6 23
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