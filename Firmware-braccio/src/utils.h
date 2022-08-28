#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

float fmap(float x, float in_min, float in_max, float out_min, float out_max);

boolean sgn(float x);

//check if number is in array
boolean inArray(int num, int *array, int size); 


#endif