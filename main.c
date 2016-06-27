#include "mcc_generated_files/mcc.h"

#include <stdint.h>
#include <stdbool.h>

#include "clarke.h"
#include "park.h"
#include "pid.h"
#include "space_vector_modulation.h"

#include <libq.h>
#include <libpic30.h>

#define _Q16_1_DIV_1023 64L

#define _Q16_Kp 65536L
#define _Q16_Ki 65536L
#define _Q16_Kd 65536L

#define _Q16_1_DIV_MAX_CURRENT 6553L // 1/10 A

#define convert_current(x) _Q16mpy(x * 65536L,3932L) - 2027188L - 1724L
#define convert_current_scaled(x) _Q16mpy(convert_current(x),_Q16_1_DIV_MAX_CURRENT)

#define my_abs(x) ((x < 0) ? -x : x)
#define my_max(x,y) ((x > y) ? x : y)
#define my_min(x,y) ((x < y) ? x : y)

#define saturate_positive_one(x) my_max(0,my_min(x,65536L))
#define sign(x) (x > 0 ? 1 : 0)

enum sector {
    SECTOR_ONE = 3,
    SECTOR_TWO = 2,
    SECTOR_THREE = 6,
    SECTOR_FOUR = 4,
    SECTOR_FIVE = 5,
    SECTOR_SIX = 1,
};

static inline void write_space_vector_modulation(_Q16 a, _Q16 b, _Q16 c)
{
    unsigned int N = 4 * sign(c) + 2*sign(b) + sign(a);
    _Q16 T1, T2, A, B, C;
    switch(N) {
        case SECTOR_ONE: {
            T1 = a;
            T2 = b;
            A = T1 + T2;
            B = T2;
            C = 0;
            break;
        }
        case SECTOR_TWO: {
            T1 = -c;
            T2 = -a;
            A = T1;
            B = T1 + T2;
            C = 0;
            break;
        }
        case SECTOR_THREE: {
            T1 = b;
            T2 = c;
            A = 0;
            B = T1 + T2;
            C = T2;
            break;
        }
        case SECTOR_FOUR: {
            T1 = -a;
            T2 = -b;
            A = 0;
            B = T1;
            C = T1 + T2;
            break;
        }
        case SECTOR_FIVE: {
            T1 = c;
            T2 = a;
            A = T2;
            B = 0;
            C = T1 + T2;
            break;
        }
        case SECTOR_SIX: {
            T1 = -b;
            T2 = -c;
            A = T1 + T2;
            B = 0;
            C = T1;
            break;
        }
    }
    
    PDC1 = my_min(_Q16mpy(_Q16_PWM_PERIOD, A ) / 65536L,PWM_PERIOD);
    PDC2 = my_min(_Q16mpy(_Q16_PWM_PERIOD, B ) / 65536L,PWM_PERIOD);
    PDC3 = my_min(_Q16mpy(_Q16_PWM_PERIOD, C ) / 65536L,PWM_PERIOD);
}

uint16_t SPI2_Exchange16bit( uint16_t txData )
{
    
    SPI2BUF = txData;
    while (SPI2STATbits.SPITBF); 
    while (!SPI2STATbits.SPIRBF);
    return SPI2BUF;
}

#define SPI_CMD_READ 0x4000 // flag indicating read attempt
#define SPI_CMD_WRITE 0x8000 // flag indicating write attempt
#define SPI_REG_AGC 0x3ffd // agc register when using SPI
#define SPI_REG_MAG 0x3ffe // magnitude register when using SPI
#define SPI_REG_DATA 0x3fff // data register when using SPI
#define SPI_REG_CLRERR 0x1 // clear error register when using SPI
#define SPI_REG_ZEROPOS_HI 0x0016 // zero position register high byte
#define SPI_REG_ZEROPOS_LO 0x0017 // zero position register low byte

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
    LATBbits.LATB6 = 0;
    uint16_t out = SPI2_Exchange16bit(txData);
    LATBbits.LATB6 = 1;
    // Delay 400ns
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    return out;
}

long as5048a_read_angle()
{  
    uint16_t cmd = SPI_CMD_READ | SPI_REG_DATA;
    cmd |= as5048a_calc_even_parity(cmd) << 15;
    as5048a_Exchange16bit(cmd);
    
    return as5048a_Exchange16bit(0x0000) & 0x3FFF;
}

int main()
{
    TRISBbits.TRISB5 = 0;
    
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB7 = 0;
    
    LATBbits.LATB6 = 1;
    
    SYSTEM_Initialize();
    
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
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
  
    static struct PID_data d_current_pid;
    static struct PID_data q_current_pid;

    pid_setup(d_current_pid, _Q16_Kp, _Q16_Ki, _Q16_Kd);
    pid_setup(q_current_pid, _Q16_Kp, _Q16_Ki, _Q16_Kd);

    while (1) {
        AD1CON1bits.SAMP = 1; // Start sampling
        while (!AD1CON1bits.DONE) {
        } // Wait for the conversion to complete
        
        // TODO over current brakes things very very badly
        _Q16 in_i_a = convert_current_scaled(ADC1BUF1);//-_Q16sin(in_theta);
        _Q16 in_i_b = convert_current_scaled(ADC1BUF2);//-_Q16sin(in_theta - 137258L);
        _Q16 commanded_q_current = saturate_positive_one(ADC1BUF3 * 72L);
        
        _Q16 in_theta = as5048a_read_angle() * 25L;

        _Q16 sin_theta = _Q16sin(in_theta);
        _Q16 cos_theta = _Q16cos(in_theta);

        _Q16 alpha, beta;
        clarke_transform(in_i_a, in_i_b, alpha, beta);

        _Q16 in_i_d, in_i_q;
        park_transform(alpha, beta, sin_theta, cos_theta, in_i_d, in_i_q);

        _Q16 out_i_d , out_i_q;
        out_i_d = in_i_d;
        out_i_q = in_i_q;
        //pid_step(d_current_pid, in_i_d, 0, out_i_d);
        //pid_step(q_current_pid, in_i_q, commanded_q_current, out_i_q);

        inverse_park_transform(out_i_d, out_i_q, sin_theta, cos_theta, alpha, beta);

        _Q16 out_i_a, out_i_b, out_i_c;
        inverse_clarke_transform(alpha, beta, out_i_a, out_i_b, out_i_c);

        write_space_vector_modulation(out_i_a, out_i_b, out_i_c);        
        
        LATBbits.LATB5 ^= 1;
    }

    return -1;
}
