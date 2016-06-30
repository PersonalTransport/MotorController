#ifndef SPI2_H
#define SPI2_H

#include <stdint.h>

void SPI2_Initialize();

uint16_t SPI2_Exchange16bit(uint16_t data);

#endif //SPI2_H
