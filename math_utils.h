#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <libq.h>

#define _Q16_1_DIV_1023 64L
#define _Q16_TWO 131072L
#define _Q16_1_DIV_2 32768L
#define _Q16_SQRT_3 113511L
#define _Q16_1_DIV_SQRT_3 37837L
#define _Q16_1_DIV_2_SQRT_3 18918L

#define my_abs(x) ((x < 0) ? -x : x)
#define my_max(x,y) ((x > y) ? x : y)
#define my_min(x,y) ((x < y) ? x : y)

#define saturate_positive_one(x) my_max(0,my_min(x,65536L))
#define sign(x) (x > 0 ? 1 : 0)

#endif // MATH_UTILS_H