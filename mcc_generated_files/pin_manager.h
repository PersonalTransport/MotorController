#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#include <xc.h>

#define CS_LWAKESetHigh() _LATB4 = 1
#define CS_LWAKESetLow() _LATB4 = 0
#define CS_LWAKEToggle() _LATB4 ^= 1
#define CS_LWAKEGetValue() _RB4
#define CS_LWAKESetDigitalInput() _TRISB4 = 1
#define CS_LWAKESetDigitalOutput() _TRISB4 = 0

#define NFAULT_TXESetHigh() _LATA3 = 1
#define NFAULT_TXESetLow() _LATA3 = 0
#define NFAULT_TXEToggle() _LATA3 ^= 1
#define NFAULT_TXEGetValue() _RA3
#define NFAULT_TXESetDigitalInput() _TRISA3 = 1
#define NFAULT_TXESetDigitalOutput() _TRISA3 = 0

#define U1TXSetHigh() _LATA4 = 1
#define U1TXSetLow() _LATA4 = 0
#define U1TXToggle() _LATA4 ^= 1
#define U1TXGetValue() _RA4
#define U1TXSetDigitalInput() _TRISA4 = 1
#define U1TXSetDigitalOutput() _TRISA4 = 0

#define U1RXSetHigh() _LATB5 = 1
#define U1RXSetLow() _LATB5 = 0
#define U1RXToggle() _LATB5 ^= 1
#define U1RXGetValue() _RB5
#define U1RXSetDigitalInput() _TRISB5 = 1
#define U1RXSetDigitalOutput() _TRISB5 = 0

#define U1TXSetHigh() _LATA4 = 1
#define U1TXSetLow() _LATA4 = 0
#define U1TXToggle() _LATA4 ^= 1
#define U1TXGetValue() _RA4
#define U1TXSetDigitalInput() _TRISA4 = 1
#define U1TXSetDigitalOutput() _TRISA4 = 0

void PIN_MANAGER_Initialize();

#endif //PIN_MANAGER_H
