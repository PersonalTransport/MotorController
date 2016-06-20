#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pwm.h"

#include <math.h>
#include <dsp.h>
#include <libpic30.h>

static const float sqrt_3 = 1.73205080757f;

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

static inline void write_space_vector_modulation(float theta,float a,float b,float c) {
    float T1,T2,A,B,C;
    if(theta >= 5.0f*PI/3.0f) { // Sector 6
        T1 = -b;
        T2 = a;
        A = T1+T2;
        B = 0;
        C = T1;
    }
    else if(theta >= 4.0f*PI/3.0f) { // Sector 5
        T1 = -c;
        T2 = -a;
        A = T2;
        B = 0;
        C = T1+T2;
    }
    else if(theta >= PI) { // Sector 4
        T1 = c;
        T2 = -b;
        A = 0;
        B = T1;
        C = T1+T2;
    }
    else if(theta >= 2.0f*PI/3.0f) { // Sector 3
        T1 = -a;
        T2 = b;
        A = 0;
        B = T1+T2;
        C = T2;
    }
    else if(theta >= PI/3.0f) { // Sector 2
        T1 = a;
        T2 = c;
        A = T1;
        B = T1+T2;
        C = 0;
    }
    else {  // Sector 1
        T1 = b;
        T2 = -c;
        A = T1+T2;
        B = T2;
        C = 0;
    }
    
    // TODO check that A,B, and C are always positive.
    PDC1 = PWM_PERIOD * A;
    PDC2 = PWM_PERIOD * B;
    PDC3 = PWM_PERIOD * C;
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
    
    TRISBbits.TRISB5 = 0;

    pid_setup(&d_current_pid,Kp,Ki,Kd);
    pid_setup(&q_current_pid,Kp,Ki,Kd);

    float x = 0;
    while (1) {
        float in_theta = x;//PI*SPI2_Exchange16bit(0xFFFF)/180.0f;
        x += 1e-6f;

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
        
        write_space_vector_modulation(in_theta,out_i_a,out_i_b,out_i_c);
        
        LATBbits.LATB5 ^= 1;
    }

    return -1;
}