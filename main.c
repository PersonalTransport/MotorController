#include "mcc_generated_files/mcc.h"
#include "lin_generated_files/motor_controller.h"

#include <libpic30.h>

int main()
{
    SYSTEM_Initialize();
    INTERRUPT_GlobalEnable();

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

void TMR5_CallBack() {
    // TODO get the angle,and currents.
}

struct l_irqmask l_sys_irq_disable()
{
    struct l_irqmask mask = { IPC2bits.U1RXIP, IPC3bits.U1TXIP };
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;
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