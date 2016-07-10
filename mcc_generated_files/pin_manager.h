#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#include <xc.h>

#define UART1_CS_LWAKE_SetHigh() _LATB4 = 1
#define UART1_CS_LWAKE_SetLow() _LATB4 = 0
#define UART1_CS_LWAKE_Toggle() _LATB4 ^= 1
#define UART1_CS_LWAKE_GetValue() _RB4
#define UART1_CS_LWAKE_SetDigitalInput() _TRISB4 = 1
#define UART1_CS_LWAKE_SetDigitalOutput() _TRISB4 = 0

#define UART1_NFAULT_TXE_SetHigh() _LATA3 = 1
#define UART1_NFAULT_TXE_SetLow() _LATA3 = 0
#define UART1_NFAULT_TXE_Toggle() _LATA3 ^= 1
#define UART1_NFAULT_TXE_GetValue() _RA3
#define UART1_NFAULT_TXE_SetDigitalInput() _TRISA3 = 1
#define UART1_NFAULT_TXE_SetDigitalOutput() _TRISA3 = 0

#define UART1_TX_SetHigh() _LATA4 = 1
#define UART1_TX_SetLow() _LATA4 = 0
#define UART1_TX_Toggle() _LATA4 ^= 1
#define UART1_TX_GetValue() _RA4
#define UART1_TX_SetDigitalInput() _TRISA4 = 1
#define UART1_TX_SetDigitalOutput() _TRISA4 = 0

#define UART1_RX_SetHigh() _LATB5 = 1
#define UART1_RX_SetLow() _LATB5 = 0
#define UART1_RX_Toggle() _LATB5 ^= 1
#define UART1_RX_GetValue() _RB5
#define UART1_RX_SetDigitalInput() _TRISB5 = 1
#define UART1_RX_SetDigitalOutput() _TRISB5 = 0

#define UART1_TX_SetHigh() _LATA4 = 1
#define UART1_TX_SetLow() _LATA4 = 0
#define UART1_TX_Toggle() _LATA4 ^= 1
#define UART1_TX_GetValue() _RA4
#define UART1_TX_SetDigitalInput() _TRISA4 = 1
#define UART1_TX_SetDigitalOutput() _TRISA4 = 0

void PIN_MANAGER_Initialize();

#endif //PIN_MANAGER_H
