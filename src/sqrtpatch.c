/*
 * use hardware float for sqrtf
 */
float sqrtf(float x)
{
    __asm__ volatile (
        "vsqrt.f32 %[var], %[var]"
        : [var]"+t"(x)
        :
        :
    );
    return x;
}
