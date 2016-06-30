#ifndef INTERRUPT_MANAGER_H
#define INTERRUPT_MANAGER_H

void INTERRUPT_Initialize();

inline static void INTERRUPT_GlobalEnable()
{
    __builtin_enable_interrupts();
}

inline static void INTERRUPT_GlobalDisable()
{
    __builtin_disable_interrupts();
}

#endif //INTERRUPT_MANAGER_H
