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
    
    TRISBbits.TRISB8 = 1; // SPI2 MISO
    TRISBbits.TRISB9 = 0; // SPI2 MOSI
    TRISBbits.TRISB6 = 0; // SPI2 SS2
    TRISBbits.TRISB7 = 0; // SPI2 CLK
    
    TRISBbits.TRISB5 = 1; // LIN RX
    TRISAbits.TRISA4 = 0; // LIN TX
    TRISBbits.TRISB4 = 0; // LIN CS/LWAKE
    TRISAbits.TRISA3 = 0; // NFAULT/TXE
    
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    
    //RxD --> U1RX --> RP37 RB5
    RPINR18bits.U1RXR = 37; //Assign U1RX To Pin RP37
    
    //TxD --> U1TX --> RP20 RA4
    RPOR0bits.RP20R = 1; //Assign U1TX To Pin RP20
    
    //CS/LWAKE --> RP36 RB4
    //FAULT/TXE --> RA3
    
    //SDI --> MISO --> RP40 RB8
    RPINR22bits.SDI2R = 40;
    
    //SDO --> MOSI --> RP41 RB9
    RPOR3bits.RP41R = 8;
            
    //CSn --> SS2 -->  RP38 RB6
    //RPOR2bits.RP38R = 10;
    
    //SCK --> CLK --> RP39 RB7
    RPOR2bits.RP39R = 9;
    
    //SCK --> CLK --> RP39 RB7
    RPINR22bits.SCK2R = 39;
    
    __builtin_write_OSCCONL(OSCCON | (1<<6));
    
    LATBbits.LATB6 = 1; // SPI2 SS2 starts high
}
