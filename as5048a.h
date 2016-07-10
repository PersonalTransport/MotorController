#ifndef AS5048A_H
#define	AS5048A_H

#include "math_utils.h"

#define SPI_CMD_READ 0x4000 // flag indicating read attempt
#define SPI_CMD_WRITE 0x8000 // flag indicating write attempt
#define SPI_REG_AGC 0x3ffd // agc register when using SPI
#define SPI_REG_MAG 0x3ffe // magnitude register when using SPI
#define SPI_REG_DATA 0x3fff // data register when using SPI
#define SPI_REG_CLRERR 0x1 // clear error register when using SPI
#define SPI_REG_ZEROPOS_HI 0x0016 // zero position register high byte
#define SPI_REG_ZEROPOS_LO 0x0017 // zero position register low byte

_Q16 as5048a_read_angle();

#endif //AS5048A_H

