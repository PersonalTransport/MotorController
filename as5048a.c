#include "as5048a.h"
#include "mcc_generated_files/mcc.h"

static uint16_t as5048a_calc_even_parity(uint16_t value)
{
    uint16_t cnt = 0;
    uint16_t i;
    for (i = 0; i < 16; i++)
    {
        if (value & 0x1) 
            cnt++;
        value >>= 1;
    }
    return cnt & 0x1;
}

uint16_t as5048a_Exchange16bit(uint16_t txData) {
    LATBbits.LATB6 = 0; // TODO move this to use PIN manager
    uint16_t out = SPI2_Exchange16bit(txData);
    LATBbits.LATB6 = 1;
    // Delay >= 400ns
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    return out;
}

_Q16 as5048a_read_angle()
{  
    uint16_t cmd = SPI_CMD_READ | SPI_REG_DATA;
    cmd |= as5048a_calc_even_parity(cmd) << 15;
    as5048a_Exchange16bit(cmd);
    
    return (as5048a_Exchange16bit(0x0000) & 0x3FFF) * 25L;
}

