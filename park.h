#ifndef PARK_H
#define PARK_H

#include "math_utils.h"

#define park_transform(alpha, beta, sin_theta, cos_theta, d, q) \
    d = (_Q16mpy(alpha, cos_theta) + _Q16mpy(beta, sin_theta)); \
    q = (_Q16mpy(beta, cos_theta) - _Q16mpy(alpha, sin_theta));

#define inverse_park_transform(d, q, sin_theta, cos_theta, alpha, beta) \
    alpha = (_Q16mpy(d, cos_theta) - _Q16mpy(q, sin_theta));            \
    beta = (_Q16mpy(q, cos_theta) + _Q16mpy(d, sin_theta));

#endif //PARK_H
