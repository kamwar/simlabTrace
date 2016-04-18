#include "misc.h"

#define CAL_FACTOR (12)
#define CALL_FACTOR_DIV (7)

//------------------------------------------------------------------------------
/// Delay in microseconds
//------------------------------------------------------------------------------
void delay_us(unsigned int interval)
{
    int iterations = (interval * CAL_FACTOR) / CALL_FACTOR_DIV;
    volatile int i;

    for (i = 0; i < iterations; ++i)
    {
        __asm__ volatile // gcc-ish syntax, don't know what compiler is used
            (
            "nop\n\t"
            "nop\n\t"
            :::
        );
    }
}

//------------------------------------------------------------------------------
/// Delay in miliseconds
//------------------------------------------------------------------------------
void delay_ms(unsigned int interval)
{
    volatile unsigned int i = 0;

    for (i = 0; i < interval; i++) {
        delay_us(1000); //1ms
    }
}