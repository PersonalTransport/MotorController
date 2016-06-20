#ifndef CLARKE_H
#define CLARKE_H

#include <libq.h>

#define _Q16_TWO 131072L
#define _Q16_SQRT_3 113511L
#define _Q16_1_DIV_SQRT_3 37837L

static inline void clarke_transform(_Q16 a, _Q16 b, _Q16* alpha, _Q16* beta)
{
    *alpha = a;
    *beta = _Q16mpy(_Q16_1_DIV_SQRT_3,(a + _Q16mpy(_Q16_TWO,b))); // TODO mac
    //*beta = (1 / sqrt_3) * (a + 2 * b);
}

static inline void inverse_clarke_transform(_Q16 alpha, _Q16 beta, _Q16* a, _Q16* b, _Q16* c)
{
    *a = alpha;
    *b = _Q16div((-alpha + _Q16mpy(_Q16_SQRT_3,beta)),_Q16_TWO); // TODO mac
    *c = _Q16div((-alpha - _Q16mpy(_Q16_SQRT_3,beta)),_Q16_TWO); // TODO mac
    //*b = (-alpha + sqrt_3 * beta) / 2.0;
    //*c = (-alpha - sqrt_3 * beta) / 2.0;
}

#endif // CLARKE_H
