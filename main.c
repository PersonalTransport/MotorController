#include "mcc_generated_files/mcc.h"
#include "lin_generated_files/motor_controller.h"

#include "math_utils.h"
    
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
    
    while (1) {
        
    }

    return -1;
}

void __attribute__((interrupt, auto_psv)) _PWMSpEventMatchInterrupt() {
    if(IFS3bits.PSEMIF) {
        IFS3bits.PSEMIF = 0;
        AD1CON1bits.SAMP = 1; // Start sampling
        
        volatile _Q16 in_theta = _Q16mpy(as5048a_read_angle(),-458752UL) + 137258L; // Read the angle while waiting for the A/D sampling and conversion
        in_theta = wrap(in_theta);
        
        while (!AD1CON1bits.DONE); // Wait for the conversion to complete
        
        volatile _Q16 commanded_q_current = saturate_positive_one(ADC1BUF3 * 76L);
        
        if(commanded_q_current > 655) {
            uint32_t x = commanded_q_current / 10; 
            if(in_theta <= 68629L) {
                PDC1 = x;
                PDC2 = x;
                PDC3 = 0;
            }
            else if(in_theta <= 137258L) {
                PDC1 = 0;
                PDC2 = x;
                PDC3 = 0;
            }
            else if(in_theta <= 205887L) {
                PDC1 = 0;
                PDC2 = x;
                PDC3 = x;
            }
            else if(in_theta <= 274517L) {
                PDC1 = 0;
                PDC2 = 0;
                PDC3 = x;
            }
            else if(in_theta <= 343146L) {
                PDC1 = x;
                PDC2 = 0;
                PDC3 = x;
            }
            else if(in_theta <= 411775L) {
                PDC1 = x;
                PDC2 = 0;
                PDC3 = 0;
            }
            else {
                PDC1 = 0;
                PDC2 = 0;
                PDC3 = 0;
            }
        }
        else {
            PDC1 = PDC2 = PDC3 = 0;
        }
    }
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
