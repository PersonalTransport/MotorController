#include "mcc_generated_files/mcc.h"
#include "lin_generated_files/motor_controller.h"

#include <math.h>
#include <dsp.h>
#include <libpic30.h>

#define PERIOD 0xFFB

static const float sqrt_3 = 1.73205080757f;

static inline void set_a_duty_cycle(unsigned int duty_cycle) {
    duty_cycle = duty_cycle / 2;
    unsigned int p_d = PERIOD - duty_cycle + 1;
    OC1_DualCompareValueSet(p_d,duty_cycle);
}

static inline void set_b_duty_cycle(unsigned int duty_cycle) {
    duty_cycle = duty_cycle / 2;
    unsigned int p_d = PERIOD - duty_cycle + 1;
    OC2_DualCompareValueSet(p_d,duty_cycle);
}

static inline void set_c_duty_cycle(unsigned int duty_cycle) {
    duty_cycle = duty_cycle / 2;
    unsigned int p_d = PERIOD - duty_cycle + 1;
    OC3_DualCompareValueSet(p_d,duty_cycle);
}

static inline void clarke_transform(float a, float b, float *alpha, float *beta) {
    *alpha = a;
    *beta = (1/sqrt_3)*(a+2*b);
}

static inline void inverse_clarke_transform(float alpha, float beta, float *a, float *b, float *c) {
    *a = alpha;
    *b = (-alpha + sqrt_3 * beta) / 2.0;
    *c = (-alpha - sqrt_3 * beta) / 2.0;
}

static inline void park_transform(float alpha, float beta, float sin_theta, float cos_theta, float *d, float *q) {
    *d = alpha*cos_theta + beta*sin_theta;
    *q = beta*cos_theta - alpha*sin_theta;
}

static inline void inverse_park_transform(float d, float q, float sin_theta, float cos_theta, float *alpha, float *beta) {
    *alpha = d*cos_theta - q*sin_theta;
    *beta = q*cos_theta + d*sin_theta;
}

struct PID_data {
    float kp,ki,kd;
    float error;
    float delta_error;
    float integral_error;
};

void pid_setup(struct PID_data *data,float kp,float ki,float kd) {
    data->kp =  kp;
    data->ki =  ki;
    data->kd =  kd;
    data->error = 0;
    data->delta_error = 0;
    data->integral_error = 0;
}

float pid_step(struct PID_data *data,float actual,float target) {
    //for tuning Ziegler?Nichols method
    //https://en.wikipedia.org/wiki/PID_controller
    
    float error = actual - target;
    data->integral_error += error;
    data->delta_error = error - data->error;
    data->error = error;
    return data->kp * data->error + (data->ki * data->integral_error) + (data->kd * data->delta_error);
}


#define Kp 0
#define Ki 0
#define Kd 0

static float in_theta = 0;
static float in_i_a = 0;
static float in_i_b = 0;
static float commanded_q_current = 0;

static struct PID_data d_current_pid;
static struct PID_data q_current_pid;
        
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
    
    pid_setup(&d_current_pid,Kp,Ki,Kd);
    pid_setup(&q_current_pid,Kp,Ki,Kd);
    
    while (1) {
        in_theta = PI*SPI2_Exchange16bit(0xFFFF)/180.0f;
    }

    return -1;
}

void TMR5_CallBack() {
    // TODO get the angle,and currents.
    in_i_a = .25; // TODO read real value
    in_i_b = .25; // TODO read real value
    commanded_q_current = .8; // TODO read real value

    float sin_theta = sinf(in_theta);
    float cos_theta = cosf(in_theta);

    float alpha,beta;
    clarke_transform(in_i_a,in_i_b,&alpha,&beta);

    float in_i_d,in_i_q;
    park_transform(alpha,beta,sin_theta,cos_theta,&in_i_d,&in_i_q);

    float out_i_d = pid_step(&d_current_pid,in_i_d,0);
    float out_i_q = pid_step(&q_current_pid,in_i_q,commanded_q_current);

    inverse_park_transform(out_i_d,out_i_q,sin_theta,cos_theta,&alpha,&beta);

    float out_i_a,out_i_b,out_i_c;
    inverse_clarke_transform(alpha,beta,&out_i_a,&out_i_b,&out_i_c);

    // TODO write out_i_a,out_i_b,out_i_c
    //set_a_duty_cycle(out_i_a);
    //set_b_duty_cycle(out_i_b);
    //set_c_duty_cycle(out_i_c);
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