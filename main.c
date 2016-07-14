#include "mcc_generated_files/mcc.h"
#include "lin_generated_files/motor_controller.h"

#include "as5048a.h"
#include "clarke.h"
#include "park.h"
#include "pid.h"
#include "space_vector_modulation.h"

#define _Q16_Kp 0//65536L
#define _Q16_Ki 0//65536L
#define _Q16_Kd 0//65536L

#define _Q16_1_DIV_MAX_CURRENT 6553L // 1/10 A

#define convert_current(x) _Q16mpy(x * 65536L,3932L) - 2027188L - 1724L
#define convert_current_scaled(x) _Q16mpy(convert_current(x),_Q16_1_DIV_MAX_CURRENT)

volatile static struct PID_data d_current_pid;
volatile static struct PID_data q_current_pid;
    
int main()
{   
    SYSTEM_Initialize();
    PDC1 = 0;
    PDC2 = 0;
    PDC3 = 0;
    
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
   
    pid_setup(d_current_pid, _Q16_Kp, _Q16_Ki, _Q16_Kd);
    pid_setup(q_current_pid, _Q16_Kp, _Q16_Ki, _Q16_Kd);
    
    while (1) {
        
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

void __attribute__((interrupt, auto_psv)) _PWMSpEventMatchInterrupt() {
    if(IFS3bits.PSEMIF) {
        AD1CON1bits.SAMP = 1; // Start sampling
        
        volatile _Q16 in_theta = _Q16mpy(as5048a_read_angle() - 0x000051D6,458752UL); // Read the angle while waiting for the A/D sampling and conversion
        volatile _Q16 sin_theta = _Q16sin(in_theta);
        volatile _Q16 cos_theta = _Q16cos(in_theta);
        
        while (!AD1CON1bits.DONE); // Wait for the conversion to complete
        
        // TODO over current brakes things very very badly
        volatile _Q16 in_i_a = convert_current_scaled(ADC1BUF1);
        volatile _Q16 in_i_b = convert_current_scaled(ADC1BUF2);
        volatile _Q16 commanded_d_current = 0;
        volatile _Q16 commanded_q_current = saturate_positive_one(ADC1BUF3 * 76L);
        
        volatile _Q16 alpha = 0, beta = 0;
        volatile _Q16 in_i_d = 0, in_i_q = 0;
        
        volatile _Q16 out_i_d = 0, out_i_q = 0;
        volatile _Q16 out_i_a = 0, out_i_b = 0, out_i_c = 0;
            
        if(commanded_q_current > 655) {
            clarke_transform(in_i_a, in_i_b, alpha, beta);
            park_transform(alpha, beta, sin_theta, cos_theta, in_i_d, in_i_q);

            pid_step(d_current_pid, in_i_d, 0, out_i_d);
            pid_step(q_current_pid, in_i_q, commanded_q_current, out_i_q);

            inverse_park_transform(out_i_d, out_i_q, sin_theta, cos_theta, alpha, beta);
            inverse_clarke_transform(alpha, beta, out_i_a, out_i_b, out_i_c);
        }
        write_space_vector_modulation(out_i_a, out_i_b, out_i_c);
        
        if(l_flg_tst_motor_controller_duty_cycle()) {
            l_flg_clr_motor_controller_duty_cycle();
            l_u16_wr_motor_controller_duty_cycle(commanded_q_current);
        }
        if(l_flg_tst_motor_theta()) {
            l_flg_clr_motor_theta();
            l_bytes_wr_motor_theta(0,4,(l_u8 *)&in_theta);
        }
        if(l_flg_tst_commanded_d_axis_current()) {
            l_flg_clr_commanded_d_axis_current();
            l_bytes_wr_commanded_d_axis_current(0,4,(l_u8 *)&commanded_d_current);
        }
        if(l_flg_tst_commanded_q_axis_current()) {
            l_flg_clr_commanded_q_axis_current();
            l_bytes_wr_commanded_q_axis_current(0,4,(l_u8 *)&commanded_q_current);
        }
        if(l_flg_tst_d_axis_current()) {
            l_flg_clr_d_axis_current();
            l_bytes_wr_d_axis_current(0,4,(l_u8 *)&in_i_d);
        }
        if(l_flg_tst_q_axis_current()) {
            l_flg_clr_q_axis_current();
            l_bytes_wr_q_axis_current(0,4,(l_u8 *)&in_i_q);
        }
        
        IFS3bits.PSEMIF = 0;
    }
}