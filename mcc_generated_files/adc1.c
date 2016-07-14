#include "adc1.h"
#include <xc.h>
#include <libpic30.h>

void ADC1_Initialize()
{
    AD1CON1bits.ADON = 0; // ADC off
    AD1CON1bits.ADSIDL = 0; // Continue module operation in Idle mode;
    AD1CON1bits.ADDMABM = 0; // DMA buffers are written in Scatter/Gather mode. The module provides a Scatter/Gather address
    // to the DMA channel, based on the index of the analog input and the size of the DMA buffer.
    AD1CON1bits.AD12B = 0; // 10-bit, 4-channel ADC operation
    AD1CON1bits.FORM = 0; // Integer (DOUT = 0000 00dd dddd dddd)
    AD1CON1bits.SSRC = 0b111; // Internal counter ends sampling and starts conversion (auto-convert)
    AD1CON1bits.SSRCG = 0; // See SSRC for details.
    AD1CON1bits.SIMSAM = 1; // Samples CH0, CH1, CH2, CH3 simultaneously
    AD1CON1bits.ASAM = 0; // Sampling begins when SAMP bit is set

    AD1CON2bits.VCFG = 0; // AVDD Avss
    AD1CON2bits.CSCNA = 0; // Do not scan inputs
    AD1CON2bits.CHPS = 3; // Converts CH0, CH1, CH2 and CH3
    AD1CON2bits.SMPI = 0; // Generates interrupt after completion of every sample/conversion operation
    AD1CON2bits.BUFM = 0; // Always starts filling the buffer from the start address.
    AD1CON2bits.ALTS = 0; // Always uses channel input selects for Sample A

    AD1CON3bits.ADRC = 0; // Clock Derived From System Clock
    AD1CON3bits.SAMC = 27; // 31 * TAD
    AD1CON3bits.ADCS = 11; // 12 * TCY = TAD

    AD1CON4bits.ADDMAEN = 0; // Conversion results stored in ADC1BUF0 through ADC1BUFF registers; DMA will not be used
    AD1CON4bits.DMABL = 0; // Allocates 1 word of buffer to each analog input

    AD1CHS0bits.CH0NB = 0; // Channel 0 negative input is VREFL
    AD1CHS0bits.CH0SB = 0; // Channel 0 positive input is AN0
    AD1CHS0bits.CH0NA = 0; // Channel 0 negative input is VREFL
    AD1CHS0bits.CH0SA = 3; // Channel 0 positive input is AN3

    AD1CHS123bits.CH123NB = 0; // CH1, CH2, CH3 negative input is VREFL
    AD1CHS123bits.CH123SB = 0; // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
    AD1CHS123bits.CH123NA = 0; // CH1, CH2, CH3 negative input is VREFL
    AD1CHS123bits.CH123SA = 0; // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2

    AD1CSSH = 0x0000;
    AD1CSSL = 0x0000;

    AD1CON1bits.ADON = 1; // ADC on
    __delay_us(20);
}
