#ifndef IGBT_THERMISTOP_H
#define IGBT_THERMISTOP_H

#include <stdint.h>

static const uint16_t igbt_thermistor_data[]  = {0, 1, 3, 6, 8, 10, 12, 14, 15, 17, 19, 21, 23, 24, 26, 27, 29, 30, 32, 33, 35, 36, 38, 39, 40, 42, 43, 44, 45, 46, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 85, 86, 87, 88, 89, 90, 92, 93, 94, 96, 97, 98, 100, 101, 103, 104, 106, 107, 109, 111, 113, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 137, 139, 141, 144, 146, 149, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 185, 188, 191, 195};


static inline uint16_t calculate_igbt_thermistor_temperature(uint16_t ad)
{
    if (ad <= 72)
        return 0;
    return igbt_thermistor_data[(ad - 72) >> 3];
}

#endif //IGBT_THERMISTOP_H
