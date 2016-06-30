#include "spi2.h"
#include <xc.h>

void SPI2_Initialize()
{
    IFS2bits.SPI2IF = 0; // Clear the Interrupt flag
    IEC2bits.SPI2IE = 0; // Disable the interrupt
    
    SPI2CON1bits.PPRE = 3; // Primary prescale 1:1
    SPI2CON1bits.SPRE = 1; // Secondary prescale 7:1
    
    SPI2CON1bits.DISSCK = 0; // Internal serial clock is enabled
    SPI2CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI2CON1bits.MODE16 = 1; // Communication is word-wide (16 bits)
    
    SPI2CON1bits.SMP = 0;
    SPI2CON1bits.CKE = 0;
    SPI2CON1bits.CKP = 0;
    
    SPI2CON1bits.MSTEN = 1; // Master mode enabled
    SPI2STATbits.SPIEN = 1; // Enable SPI module
}

uint16_t SPI2_Exchange16bit(uint16_t txData)
{   
    SPI2BUF = txData;
    while (SPI2STATbits.SPITBF); 
    while (!SPI2STATbits.SPIRBF);
    return SPI2BUF;
}
