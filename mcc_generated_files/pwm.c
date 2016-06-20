#include "pwm.h"
#include <xc.h>

static inline void PWM1_Initialize()
{
    PWMCON1bits.FLTIEN = 0; // Fault interrupt is disabled and FLTSTAT bit is cleared
    PWMCON1bits.CLIEN = 0; // Current-limit interrupt disabled and CLSTAT bit is cleared
    PWMCON1bits.TRGIEN = 0; // Trigger event interrupts are disabled and TRGSTAT bit is cleared
    PWMCON1bits.ITB = 1; // PHASEx register provides time base period for this PWM generator
    PWMCON1bits.MDCS = 0; // PDCx register provides duty cycle information for this PWM generator
    PWMCON1bits.DTC = 2; //  Dead-time function is disabled
    PWMCON1bits.DTCP = 0; // DTC not used
    PWMCON1bits.MTBS = 0; // PWM generator uses the primary master time base for synchronization and as the clock source
    // for the PWM generation logic
    PWMCON1bits.CAM = 1; // Center-Aligned mode is enabled
    PWMCON1bits.XPRES = 0; // External pins do not affect PWM time base
    PWMCON1bits.IUE = 0; // Updates to the active MDC/PDCx/DTx/ALTDTRx/PHASEx registers are synchronized to the
    // PWM time base

    IOCON1bits.PENH = 1; // PWM module controls PWMxH pin
    IOCON1bits.PENL = 1; // PWM module controls PWMxL pin
    IOCON1bits.POLH = 0; // PWMxH pin is active-high
    IOCON1bits.POLL = 0; // PWMxL pin is active-high
    IOCON1bits.PMOD = 0; // PWM I/O pin pair is in the Complementary Output mode
    IOCON1bits.OVRENH = 0; // PWM generator controls PWMxH pin
    IOCON1bits.OVRENL = 0; // PWM generator controls PWMxL pin
    IOCON1bits.OVRDAT = 0; // ???
    IOCON1bits.FLTDAT = 0; // ???
    IOCON1bits.CLDAT = 0; // ???
    IOCON1bits.SWAP = 0; //  PWMxH and PWMxL pins are mapped to their respective pins
    IOCON1bits.OSYNC = 0; // Output overrides via the OVDDAT<1:0> bits occur on the next CPU clock boundary*/

    FCLCON1bits.IFLTMOD = 0; // Normal Fault mode: Current-Limit mode maps CLDAT<1:0> bits to the PWMxH and PWMxL
    // outputs. The PWM Fault mode maps FLTDAT<1:0> to the PWMxH and PWMxL outputs.
    FCLCON1bits.CLSRC = 0; // Fault 1 (default)
    FCLCON1bits.CLPOL = 0; // The selected current-limit source is active-high
    FCLCON1bits.CLMOD = 0; // Current-Limit mode is disabled
    FCLCON1bits.FLTSRC = 0; // Fault 1
    FCLCON1bits.FLTPOL = 0; // The selected Fault source is active-high
    FCLCON1bits.FLTMOD = 0; // The selected Fault source forces PWMxH, PWMxL pins to FLTDAT values (latched condition)

    PHASE1 = PWM_PERIOD;
}

static inline void PWM2_Initialize()
{
    PWMCON2bits.FLTIEN = 0; // Fault interrupt is disabled and FLTSTAT bit is cleared
    PWMCON2bits.CLIEN = 0; // Current-limit interrupt disabled and CLSTAT bit is cleared
    PWMCON2bits.TRGIEN = 0; // Trigger event interrupts are disabled and TRGSTAT bit is cleared
    PWMCON2bits.ITB = 1; // PHASEx register provides time base period for this PWM generator
    PWMCON2bits.MDCS = 0; // PDCx register provides duty cycle information for this PWM generator
    PWMCON2bits.DTC = 2; //  Dead-time function is disabled
    PWMCON2bits.DTCP = 0; // DTC not used
    PWMCON2bits.MTBS = 0; // PWM generator uses the primary master time base for synchronization and as the clock source
    // for the PWM generation logic
    PWMCON2bits.CAM = 1; // Center-Aligned mode is enabled
    PWMCON2bits.XPRES = 0; // External pins do not affect PWM time base
    PWMCON2bits.IUE = 0; // Updates to the active MDC/PDCx/DTx/ALTDTRx/PHASEx registers are synchronized to the
    // PWM time base

    IOCON2bits.PENH = 1; // PWM module controls PWMxH pin
    IOCON2bits.PENL = 1; // PWM module controls PWMxL pin
    IOCON2bits.POLH = 0; // PWMxH pin is active-high
    IOCON2bits.POLL = 0; // PWMxL pin is active-high
    IOCON2bits.PMOD = 0; // PWM I/O pin pair is in the Complementary Output mode
    IOCON2bits.OVRENH = 0; // PWM generator controls PWMxH pin
    IOCON2bits.OVRENL = 0; // PWM generator controls PWMxL pin
    IOCON2bits.OVRDAT = 0; // ???
    IOCON2bits.FLTDAT = 0; // ???
    IOCON2bits.CLDAT = 0; // ???
    IOCON2bits.SWAP = 0; //  PWMxH and PWMxL pins are mapped to their respective pins
    IOCON2bits.OSYNC = 0; // Output overrides via the OVDDAT<1:0> bits occur on the next CPU clock boundary*/

    FCLCON2bits.IFLTMOD = 0; // Normal Fault mode: Current-Limit mode maps CLDAT<1:0> bits to the PWMxH and PWMxL
    // outputs. The PWM Fault mode maps FLTDAT<1:0> to the PWMxH and PWMxL outputs.
    FCLCON2bits.CLSRC = 0; // Fault 1 (default)
    FCLCON2bits.CLPOL = 0; // The selected current-limit source is active-high
    FCLCON2bits.CLMOD = 0; // Current-Limit mode is disabled
    FCLCON2bits.FLTSRC = 0; // Fault 1
    FCLCON2bits.FLTPOL = 0; // The selected Fault source is active-high
    FCLCON2bits.FLTMOD = 0; // The selected Fault source forces PWMxH, PWMxL pins to FLTDAT values (latched condition)

    PHASE2 = PWM_PERIOD;
}

static inline void PWM3_Initialize()
{
    PWMCON3bits.FLTIEN = 0; // Fault interrupt is disabled and FLTSTAT bit is cleared
    PWMCON3bits.CLIEN = 0; // Current-limit interrupt disabled and CLSTAT bit is cleared
    PWMCON3bits.TRGIEN = 0; // Trigger event interrupts are disabled and TRGSTAT bit is cleared
    PWMCON3bits.ITB = 1; // PHASEx register provides time base period for this PWM generator
    PWMCON3bits.MDCS = 0; // PDCx register provides duty cycle information for this PWM generator
    PWMCON3bits.DTC = 2; //  Dead-time function is disabled
    PWMCON3bits.DTCP = 0; // DTC not used
    PWMCON3bits.MTBS = 0; // PWM generator uses the primary master time base for synchronization and as the clock source
    // for the PWM generation logic
    PWMCON3bits.CAM = 1; // Center-Aligned mode is enabled
    PWMCON3bits.XPRES = 0; // External pins do not affect PWM time base
    PWMCON3bits.IUE = 0; // Updates to the active MDC/PDCx/DTx/ALTDTRx/PHASEx registers are synchronized to the
    // PWM time base

    IOCON3bits.PENH = 1; // PWM module controls PWMxH pin
    IOCON3bits.PENL = 1; // PWM module controls PWMxL pin
    IOCON3bits.POLH = 0; // PWMxH pin is active-high
    IOCON3bits.POLL = 0; // PWMxL pin is active-high
    IOCON3bits.PMOD = 0; // PWM I/O pin pair is in the Complementary Output mode
    IOCON3bits.OVRENH = 0; // PWM generator controls PWMxH pin
    IOCON3bits.OVRENL = 0; // PWM generator controls PWMxL pin
    IOCON3bits.OVRDAT = 0; // ???
    IOCON3bits.FLTDAT = 0; // ???
    IOCON3bits.CLDAT = 0; // ???
    IOCON3bits.SWAP = 0; //  PWMxH and PWMxL pins are mapped to their respective pins
    IOCON3bits.OSYNC = 0; // Output overrides via the OVDDAT<1:0> bits occur on the next CPU clock boundary*/

    FCLCON3bits.IFLTMOD = 0; // Normal Fault mode: Current-Limit mode maps CLDAT<1:0> bits to the PWMxH and PWMxL
    // outputs. The PWM Fault mode maps FLTDAT<1:0> to the PWMxH and PWMxL outputs.
    FCLCON3bits.CLSRC = 0; // Fault 1 (default)
    FCLCON3bits.CLPOL = 0; // The selected current-limit source is active-high
    FCLCON3bits.CLMOD = 0; // Current-Limit mode is disabled
    FCLCON3bits.FLTSRC = 0; // Fault 1
    FCLCON3bits.FLTPOL = 0; // The selected Fault source is active-high
    FCLCON3bits.FLTMOD = 0; // The selected Fault source forces PWMxH, PWMxL pins to FLTDAT values (latched condition)

    PHASE3 = PWM_PERIOD;
}

void PWM_Initialize()
{
    PTCONbits.PTEN = 0; // PWM module is disabled
    PTCONbits.PTSIDL = 0; // PWM time base runs in CPU Idle mode
    PTCONbits.SEIEN = 0; // Special Event Interrupt is disabled
    PTCONbits.EIPU = 0; // Active Period register updates occur on PWM cycle boundaries
    PTCONbits.SYNCPOL = 0; // SYNCI1/SYNCO1 is active-high
    PTCONbits.SYNCEN = 0; // SYNCO1 output is disabled
    PTCONbits.SYNCSRC = 0; // SYNCI 1 input from PPS
    PTCONbits.SEVTPS = 0; // 1:1 Postscaler generates Special Event Trigger on every compare match event

    PTCON2bits.PCLKDIV = 0; // Divide by 1, maximum PWM timing resolution (power-on default)

    PTPER = PWM_PERIOD;
    SEVTCMP = 0;

    PWM1_Initialize();
    PWM2_Initialize();
    PWM3_Initialize();

    PTCONbits.PTEN = 1;
}
