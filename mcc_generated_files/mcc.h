#ifndef MCC_H
#define MCC_H
#include "interrupt_manager.h"
#include "pin_manager.h"
#include "pwm.h"
#include "spi2.h"
#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#define _XTAL_FREQ 119762500UL

void SYSTEM_Initialize();

void OSCILLATOR_Initialize();

inline static void WDT_WatchdogtimerSoftwareEnable()
{
    RCONbits.SWDTEN = 1;
}

inline static void WDT_WatchdogtimerSoftwareDisable()
{
    RCONbits.SWDTEN = 0;
}

inline static void WDT_WatchdogTimerClear()
{
    ClrWdt();
}

#endif //MCC_H
