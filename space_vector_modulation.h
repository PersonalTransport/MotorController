#ifndef SPACE_VECTOR_MODULATION_H
#define SPACE_VECTOR_MODULATION_H

#include "mcc_generated_files/mcc.h"

#include "math_utils.h"

#define _Q16_PWM_PERIOD 392429568L

enum sector {
    SECTOR_ONE = 3,
    SECTOR_TWO = 2,
    SECTOR_THREE = 6,
    SECTOR_FOUR = 4,
    SECTOR_FIVE = 5,
    SECTOR_SIX = 1
};

static inline void write_space_vector_modulation(_Q16 a, _Q16 b, _Q16 c)
{
    unsigned int N = 4 * sign(c) + 2*sign(b) + sign(a);
    _Q16 T1 = 0, T2 = 0, A = 0, B = 0, C = 0;
    switch(N) {
        case SECTOR_ONE: {
            T1 = a;
            T2 = b;
            A = T1 + T2;
            B = T2;
            C = 0;
            break;
        }
        case SECTOR_TWO: {
            T1 = -c;
            T2 = -a;
            A = T1;
            B = T1 + T2;
            C = 0;
            break;
        }
        case SECTOR_THREE: {
            T1 = b;
            T2 = c;
            A = 0;
            B = T1 + T2;
            C = T2;
            break;
        }
        case SECTOR_FOUR: {
            T1 = -a;
            T2 = -b;
            A = 0;
            B = T1;
            C = T1 + T2;
            break;
        }
        case SECTOR_FIVE: {
            T1 = c;
            T2 = a;
            A = T2;
            B = 0;
            C = T1 + T2;
            break;
        }
        case SECTOR_SIX: {
            T1 = -b;
            T2 = -c;
            A = T1 + T2;
            B = 0;
            C = T1;
            break;
        }
    }
    
    PDC1 = my_min(_Q16mpy(_Q16_PWM_PERIOD, A ) / 65536L,PWM_PERIOD);
    PDC2 = my_min(_Q16mpy(_Q16_PWM_PERIOD, B ) / 65536L,PWM_PERIOD);
    PDC3 = my_min(_Q16mpy(_Q16_PWM_PERIOD, C ) / 65536L,PWM_PERIOD);
}

#endif //SPACE_VECTOR_MODULATION_H
