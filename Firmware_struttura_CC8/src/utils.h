#include <Arduino.h>

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return constrain((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
}

//angle in radians
float degToRad(float deg) {
    return deg*PI/180;
}

//angle in degrees
float radToDeg(float rad) {
    return rad*180/PI;
}

