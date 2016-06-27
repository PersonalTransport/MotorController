#ifndef PID_H
#define PID_H

#include "math_utils.h"

struct PID_data {
    _Q16 kp, ki, kd;
    _Q16 error;
    _Q16 delta_error;
    _Q16 integral_error;
};

//for tuning Ziegler?Nichols method
//https://en.wikipedia.org/wiki/PID_controller

#define pid_setup(data, Kp, Ki, Kd) \
    data.kp = Kp;                   \
    data.ki = Ki;                   \
    data.kd = Kd;                   \
    data.error = 0;                 \
    data.delta_error = 0;           \
    data.integral_error = 0;

#define pid_step(data, actual, target, output)                                                                              \
    {                                                                                                                       \
        _Q16 error = actual - target;                                                                                       \
        data.integral_error += error;                                                                                       \
        data.delta_error = error - data.error;                                                                              \
        data.error = error;                                                                                                 \
        output = _Q16mpy(data.kp, data.error) + _Q16mpy(data.ki, data.integral_error) + _Q16mpy(data.kd, data.delta_error); \
    }

#endif // PID_H
