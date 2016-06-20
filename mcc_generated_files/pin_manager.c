#include "pin_manager.h"

void PIN_MANAGER_Initialize()
{
    ANSELA = ANSELB = 0x0000;
    TRISAbits.TRISA0 = 1; // AN0/RA0 is input
    TRISAbits.TRISA1 = 1; // AN1/RA1 is input
    TRISBbits.TRISB0 = 1; // AN2/RB0 is input
    TRISBbits.TRISB1 = 1; // AN3/RB1 is input
    ANSELAbits.ANSA0 = 1; // AN0/RA0 is analog
    ANSELAbits.ANSA1 = 1; // AN1/RA1 is analog
    ANSELBbits.ANSB0 = 1; // AN2/RB0 is analog
    ANSELBbits.ANSB1 = 1; // AN3/RB1 is analog
}
