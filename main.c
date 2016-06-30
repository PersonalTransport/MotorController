#include "mcc_generated_files/mcc.h"
#include "lin_generated_files/motor_controller.h"

#include "math_utils.h"
#include "clarke.h"
#include "park.h"
#include "pid.h"
#include "space_vector_modulation.h"

#include <stdint.h>

#define _Q16_Kp 65536L
#define _Q16_Ki 65536L
#define _Q16_Kd 65536L

#define _Q16_1_DIV_MAX_CURRENT 6553L // 1/10 A

#define convert_current(x) _Q16mpy(x * 65536L,3932L) - 2027188L - 1724L
#define convert_current_scaled(x) _Q16mpy(convert_current(x),_Q16_1_DIV_MAX_CURRENT)

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
    SYSTEM_Initialize();
    
    // Initialize the LIN interface
    if (l_sys_init())
        return -1;

    // Initialize the interface
    if (l_ifc_init_UART1())
        return -1;

    // Set UART TX to interrupt level 5
    // Set UART RX to interrupt level 5
    struct l_irqmask irqmask = { 5, 5 };
    l_sys_irq_restore(irqmask);

    static struct PID_data d_current_pid;
    static struct PID_data q_current_pid;

    pid_setup(d_current_pid, _Q16_Kp, _Q16_Ki, _Q16_Kd);
    pid_setup(q_current_pid, _Q16_Kp, _Q16_Ki, _Q16_Kd);

    int x = 0;
    while (1) {
        AD1CON1bits.SAMP = 1; // Start sampling
        _Q16 in_theta = as5048a_read_angle() * 25L - 0x5CAE; // TODO convert to electrical angle.
        while (!AD1CON1bits.DONE); // Wait for the conversion to complete
        
        // TODO over current brakes things very very badly
        _Q16 in_i_a = convert_current_scaled(ADC1BUF1);//-_Q16sin(in_theta);
        _Q16 in_i_b = convert_current_scaled(ADC1BUF2);//-_Q16sin(in_theta - 137258L);
        _Q16 commanded_q_current = saturate_positive_one(ADC1BUF3 * 72L);

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
    }

    return -1;
}

struct l_irqmask l_sys_irq_disable()
{
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;
    struct l_irqmask mask = { IPC2bits.U1RXIP, IPC3bits.U1TXIP };
    return mask;
}

void l_sys_irq_restore(struct l_irqmask previous)
{
    IPC2bits.U1RXIP = previous.rx_level;
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;

    IPC3bits.U1TXIP = previous.tx_level;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1TXIE = 1;
}