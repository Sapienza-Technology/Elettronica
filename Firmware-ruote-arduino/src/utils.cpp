#include "utils.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return constrain((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
}

boolean sgn(float x) { return x > 0; }
