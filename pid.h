#ifndef PID_H
#define PID_H

#include <libq.h>

struct PID_data {
    _Q16 kp, ki, kd;
    _Q16 error;
    _Q16 delta_error;
    _Q16 integral_error;
};

static inline void pid_setup(struct PID_data* data, _Q16 kp, _Q16 ki, _Q16 kd)
{
    data->kp = kp;
    data->ki = ki;
    data->kd = kd;
    data->error = 0;
    data->delta_error = 0;
    data->integral_error = 0;
}

static inline _Q16 pid_step(struct PID_data* data, _Q16 actual, _Q16 target)
{
    //for tuning Ziegler?Nichols method
    //https://en.wikipedia.org/wiki/PID_controller

    float error = actual - target;
    data->integral_error += error;
    data->delta_error = error - data->error;
    data->error = error;
    return _Q16mpy(data->kp,data->error) + _Q16mpy(data->ki,data->integral_error) + _Q16mpy(data->kd,data->delta_error);
    //return (data->kp * data->error) + (data->ki * data->integral_error) + (data->kd * data->delta_error);
}

#endif // PID_H
