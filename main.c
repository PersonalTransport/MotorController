#include "mcc_generated_files/mcc.h"

#include "clarke.h"
#include "park.h"
#include "pid.h"
#include "space_vector_modulation.h"

#define Kp 1.0
#define Ki 1.0
#define Kd 1.0

int main()
{
    SYSTEM_Initialize();

    TRISBbits.TRISB5 = 0;

    static struct PID_data d_current_pid;
    static struct PID_data q_current_pid;
    
    pid_setup(&d_current_pid, Kp, Ki, Kd);
    pid_setup(&q_current_pid, Kp, Ki, Kd);

    float in_theta = 0;
    while (1) {
        AD1CON1bits.SAMP = 1; // Start sampling
        while (!AD1CON1bits.DONE); // Wait for the conversion to complete
        
        in_theta += 1e-6f;
        
        float in_i_a = ADC1BUF1/1023.0f;
        float in_i_b = ADC1BUF2/1023.0f;
        float commanded_q_current = ADC1BUF3/1023.0f;
        
        float sin_theta = sinf(in_theta);
        float cos_theta = cosf(in_theta);

        float alpha, beta;
        clarke_transform(in_i_a, in_i_b, &alpha, &beta);

        float in_i_d, in_i_q;
        park_transform(alpha, beta, sin_theta, cos_theta, &in_i_d, &in_i_q);

        float out_i_d = pid_step(&d_current_pid, in_i_d, 0);
        float out_i_q = pid_step(&q_current_pid, in_i_q, commanded_q_current);

        inverse_park_transform(out_i_d, out_i_q, sin_theta, cos_theta, &alpha, &beta);

        float out_i_a, out_i_b, out_i_c;
        inverse_clarke_transform(alpha, beta, &out_i_a, &out_i_b, &out_i_c);

        write_space_vector_modulation(in_theta, out_i_a, out_i_b, out_i_c);

        LATBbits.LATB5 ^= 1;
    }

    return -1;
}
