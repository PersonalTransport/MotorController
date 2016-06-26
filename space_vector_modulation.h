#ifndef SPACE_VECTOR_MODULATION_H
#define SPACE_VECTOR_MODULATION_H

#include "mcc_generated_files/pwm.h"
#include <libq.h>



#define _Q16_PWM_PERIOD 392429568L

/*static inline void write_space_vector_modulation(_Q16 theta, _Q16 a, _Q16 b, _Q16 c)
{
    _Q16 T1, T2, A, B, C;
    if (theta >= _Q16_5_PI_DIV_3) { // Sector 6
        T1 = -b;
        T2 = a;
        A = T1 + T2;
        B = 0;
        C = T1;
    } else if (theta >= _Q16_4_PI_DIV_3) { // Sector 5
        T1 = -c;
        T2 = -a;
        A = T2;
        B = 0;
        C = T1 + T2;
    } else if (theta >= _Q16_PI) { // Sector 4
        T1 = c;
        T2 = -b;
        A = 0;
        B = T1;
        C = T1 + T2;
    } else if (theta >= _Q16_2_PI_DIV_3) { // Sector 3
        T1 = -a;
        T2 = b;
        A = 0;
        B = T1 + T2;
        C = T2;
    } else if (theta >= _Q16_PI_DIV_3) { // Sector 2
        T1 = a;
        T2 = c;
        A = T1;
        B = T1 + T2;
        C = 0;
    } else { // Sector 1
        T1 = b;
        T2 = -c;
        A = T1 + T2;
        B = T2;
        C = 0;
    }

    // TODO check that A,B, and C are always positive.
    PDC1 = _Q16mpy(_Q16_PWM_PERIOD, A);
    PDC2 = _Q16mpy(_Q16_PWM_PERIOD, B);
    PDC3 = _Q16mpy(_Q16_PWM_PERIOD, C);
}*/

#endif //SPACE_VECTOR_MODULATION_H
