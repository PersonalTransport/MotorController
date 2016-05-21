// CONFIG4
#pragma config DSWDTPS = DSWDTPSF // DSWDT Postscale Select (1:2,147,483,648 (25.7 days))
#pragma config DSWDTOSC = LPRC // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = SOSC // RTCC Reference Oscillator Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = ON // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON // Deep Sleep Watchdog Timer (DSWDT enabled)

// CONFIG3
#pragma config WPFP = WPFP63 // Write Protection Flash Page Segment Boundary (Highest Page (same as page 21))
#pragma config SOSCSEL = IO // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
#pragma config WUTSEL = LEG // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = NONE // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config I2C1SEL = PRI // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config OSCIOFNC = ON // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSDCMD // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
#pragma config FNOSC = FRC // Initial Oscillator Select (Fast RC Oscillator (FRC))
#pragma config PLL96MHZ = OFF // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled by user in software( controlled with the PLLEN bit))
#pragma config PLLDIV = NODIV // USB 96 MHz PLL Prescaler Select (Oscillator input used directly (4 MHz input))
#pragma config IESO = OFF // Internal External Switchover (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS32768 // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128 // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1 // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG port is disabled)

#include <igbt_thermistor.h>
#include <libpic30.h>
#include <motor_controller.h>
#include <xc.h>

void initPWM();
void percentDutyCycle(unsigned long int value);
void initTmr2PWM();

static l_bool configuration_ok = false;

int main()
{
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

    __builtin_write_OSCCONL(OSCCON & ~(1 << 6));
    RPOR7bits.RP14R = 18; //setting pin RP15 to output OC1(PWM)
    __builtin_write_OSCCONL(OSCCON | (1 << 6));

    AD1PCFGLbits.PCFG0 = 0;
    TRISAbits.TRISA0 = 1;

    AD1PCFGLbits.PCFG11 = 0;
    TRISBbits.TRISB13 = 1;

    AD1CON2bits.VCFG = 0;
    AD1CON3bits.ADCS = 0b00111111;

    AD1CON1bits.SSRC = 0b111;
    AD1CON3bits.SAMC = 0b11111;

    AD1CON1bits.FORM = 0b00;
    AD1CON2bits.SMPI = 0b0010;
    AD1CON1bits.ASAM = 1;

    AD1CON2bits.CSCNA = 1;
    AD1CON2bits.ALTS = 0;
    AD1CSSLbits.CSSL0 = 1;
    AD1CSSLbits.CSSL4 = 1;
    AD1CSSLbits.CSSL11 = 1;

    AD1CHSbits.CH0NA = 0;

    AD1CON1bits.ADON = 1;

    IFS0bits.AD1IF = 0;
    IPC3bits.AD1IP = 4;
    IEC0bits.AD1IE = 1;

    initPWM();
    initTmr2PWM();

    while (1) {
        if (l_ifc_read_status_UART1() & (1 << 6))
            configuration_ok = true;
    }
}

void initPWM()
{
    OC1R = 0;
    OC1RS = 0;
    IC1CON2bits.SYNCSEL = 0x1F;
    OC1CON2bits.OCTRIG = 0;
    OC1CON1bits.OCTSEL = 0b000;
    OC1CON1bits.OCM = 0b110;
}

void initTmr2PWM()
{
    T2CONbits.TCS = 0b0; //Timer2 Clock Source is Internal the clock (FOSC/2)
    T2CONbits.T32 = 0b0; //16 bit mode
    T2CONbits.TGATE = 0b0; //Gated time accumulation is disabled
    T2CONbits.TCKPS = 0b00; //Setting pre-scaler to 1

    TMR2 = 0; //Clearing timer register
    PR2 = 0xFFFF; //Setting the frequency to 61 Hz

    T2CONbits.TON = 0b1; //Turning timer2 on
}

void __attribute__((interrupt, auto_psv)) _ADC1Interrupt()
{
    if (IFS0bits.AD1IF) {
        IFS0bits.AD1IF = 0;
        int32_t value = (((int32_t)ADC1BUF0) - 350) * 145;
        if (value < 0)
            value = 0;
        else if (value > 0xFFFF)
            value = 0xFFFF;
        
        OC1R = value;
        
        l_u16 igbt1_temp = calculate_igbt_thermistor_temperature(ADC1BUF1);
        l_u16 igbt2_temp = calculate_igbt_thermistor_temperature(ADC1BUF2);
        int32_t igbt_temp_diff = ((int32_t) igbt1_temp) - ((int32_t) igbt2_temp);
        igbt_temp_diff = (igbt_temp_diff < 0) ? -igbt_temp_diff : igbt_temp_diff;
        
        if(igbt1_temp >= 110 || igbt2_temp >= 110 || igbt_temp_diff > 50) {
            OC1R = 0;
        }
        
        if (l_flg_tst_motor_controller_duty_cycle()) {
            l_flg_clr_motor_controller_duty_cycle();
            l_u16_wr_motor_controller_duty_cycle(value);
        }

        if (l_flg_tst_motor_controller_igbt1_temperature()) {
            l_flg_clr_motor_controller_igbt1_temperature();
            l_u16_wr_motor_controller_igbt1_temperature(igbt1_temp);
        }
        
        if (l_flg_tst_motor_controller_igbt2_temperature()) {
            l_flg_clr_motor_controller_igbt2_temperature();
            l_u16_wr_motor_controller_igbt2_temperature(igbt2_temp);
        }
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt()
{
    if (IFS0bits.U1TXIF) {
        IFS0bits.U1TXIF = 0;
        l_ifc_tx_UART1();
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    if (IFS0bits.U1RXIF) {
        IFS0bits.U1RXIF = 0;
        l_ifc_rx_UART1();
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
