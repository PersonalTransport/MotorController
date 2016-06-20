#ifndef PARK_H
#define PARK_H

#include <libq.h>

static inline void park_transform(_Q16 alpha, _Q16 beta, _Q16 sin_theta, _Q16 cos_theta, _Q16* d, _Q16* q)
{
    *d = (_Q16mpy(alpha,cos_theta) + _Q16mpy(beta,sin_theta)); // TODO mac
    *q = (_Q16mpy(beta,cos_theta)  - _Q16mpy(alpha,sin_theta)); // TODO mac
    //*d = alpha * cos_theta + beta * sin_theta;
    //*q = beta * cos_theta - alpha * sin_theta;
}

static inline void inverse_park_transform(_Q16 d, _Q16 q, _Q16 sin_theta, _Q16 cos_theta, _Q16* alpha, _Q16* beta)
{
    *alpha = (_Q16mpy(d,cos_theta) - _Q16mpy(q,sin_theta)); // TODO mac
    *beta  = (_Q16mpy(q,cos_theta) + _Q16mpy(d,sin_theta)); // TODO mac
    //*alpha = d * cos_theta - q * sin_theta;
    //*beta = q * cos_theta + d * sin_theta;
}

#endif //PARK_H
