#ifndef CLARKE_H
#define CLARKE_H

#include "math_utils.h"

#define clarke_transform(a, b, alpha, beta) \
    alpha = a;                              \
    beta = _Q16mpy(_Q16_1_DIV_SQRT_3, (a + _Q16mpy(_Q16_TWO, b)));

#define inverse_clarke_transform(alpha, beta, a, b, c)            \
    a = alpha;                          \
    b = _Q16mpy((-alpha + _Q16mpy(_Q16_SQRT_3, beta)), _Q16_1_DIV_2); \
    c = _Q16mpy((-alpha - _Q16mpy(_Q16_SQRT_3, beta)), _Q16_1_DIV_2);

#define modified_inverse_clarke_transform(alpha, beta, a, b, c) \
    a = _Q16mpy((_Q16mpy(_Q16_SQRT_3, alpha) - beta), _Q16_1_DIV_2); \
    b = beta; \
    c = _Q16mpy((_Q16mpy(_Q16_SQRT_3, -alpha) - beta), _Q16_1_DIV_2);
    

#endif // CLARKE_H
