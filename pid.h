#ifndef PID_H
#define PID_H

struct PID_data {
    float kp, ki, kd;
    float error;
    float delta_error;
    float integral_error;
};

static inline void pid_setup(struct PID_data* data, float kp, float ki, float kd)
{
    data->kp = kp;
    data->ki = ki;
    data->kd = kd;
    data->error = 0;
    data->delta_error = 0;
    data->integral_error = 0;
}

static inline float pid_step(struct PID_data* data, float actual, float target)
{
    //for tuning Ziegler?Nichols method
    //https://en.wikipedia.org/wiki/PID_controller

    float error = actual - target;
    data->integral_error += error;
    data->delta_error = error - data->error;
    data->error = error;
    return data->kp * data->error + (data->ki * data->integral_error) + (data->kd * data->delta_error);
}

#endif // PID_H
