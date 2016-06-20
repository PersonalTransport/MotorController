#ifndef CLARKE_H
#define CLARKE_H

static const float sqrt_3 = 1.73205080757f;

static inline void clarke_transform(float a, float b, float* alpha, float* beta)
{
    *alpha = a;
    *beta = (1 / sqrt_3) * (a + 2 * b);
}

static inline void inverse_clarke_transform(float alpha, float beta, float* a, float* b, float* c)
{
    *a = alpha;
    *b = (-alpha + sqrt_3 * beta) / 2.0;
    *c = (-alpha - sqrt_3 * beta) / 2.0;
}

#endif // CLARKE_H
