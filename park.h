#ifndef PARK_H
#define PARK_H

static inline void park_transform(float alpha, float beta, float sin_theta, float cos_theta, float* d, float* q)
{
    *d = alpha * cos_theta + beta * sin_theta;
    *q = beta * cos_theta - alpha * sin_theta;
}

static inline void inverse_park_transform(float d, float q, float sin_theta, float cos_theta, float* alpha, float* beta)
{
    *alpha = d * cos_theta - q * sin_theta;
    *beta = q * cos_theta + d * sin_theta;
}

#endif //PARK_H
