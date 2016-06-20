#include "mcc.h"
#include "adc1.h"
#include "pin_manager.h"
#include "pwm.h"

#pragma config FNOSC = FRCPLL
#pragma config IESO = OFF
#pragma config FCKSM = CSECMD
#pragma config OSCIOFNC = OFF
#pragma config POSCMD = NONE
#pragma config PWMLOCK = OFF // PWM Lock Enable bit (PWM registers may be written without key sequence)

void SYSTEM_Initialize()
{
    PIN_MANAGER_Initialize();
    OSCILLATOR_Initialize();
    ADC1_Initialize();
    PWM_Initialize();
}

void OSCILLATOR_Initialize()
{
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBD = 63; // M = 65
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    CLKDIVbits.PLLPRE = 0; // N1 = 2

    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001) {
    }

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) {
    }
}
