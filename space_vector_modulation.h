#ifndef SPACE_VECTOR_MODULATION_H
#define SPACE_VECTOR_MODULATION_H

#include "mcc_generated_files/pwm.h"
#include <dsp.h>

static inline void write_space_vector_modulation(float theta, float a, float b, float c)
{
    float T1, T2, A, B, C;
    if (theta >= 5.0f * PI / 3.0f) { // Sector 6
        T1 = -b;
        T2 = a;
        A = T1 + T2;
        B = 0;
        C = T1;
    } else if (theta >= 4.0f * PI / 3.0f) { // Sector 5
        T1 = -c;
        T2 = -a;
        A = T2;
        B = 0;
        C = T1 + T2;
    } else if (theta >= PI) { // Sector 4
        T1 = c;
        T2 = -b;
        A = 0;
        B = T1;
        C = T1 + T2;
    } else if (theta >= 2.0f * PI / 3.0f) { // Sector 3
        T1 = -a;
        T2 = b;
        A = 0;
        B = T1 + T2;
        C = T2;
    } else if (theta >= PI / 3.0f) { // Sector 2
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
    PDC1 = PWM_PERIOD * A;
    PDC2 = PWM_PERIOD * B;
    PDC3 = PWM_PERIOD * C;
}

#endif //SPACE_VECTOR_MODULATION_H
