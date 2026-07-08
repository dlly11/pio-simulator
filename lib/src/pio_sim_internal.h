/* pio_sim_internal.h — helpers shared between the simulator and assembler
 * translation units. Private to lib/src; not installed with the public API. */
#ifndef PIO_SIM_INTERNAL_H
#define PIO_SIM_INTERNAL_H

#include <stdint.h>

/* 32-bit bit-order reversal (MOV ::/bit-reverse op and the assembler's `::`
 * expression operator share this definition). */
static inline uint32_t pio_reverse32(uint32_t v_in)
{
    uint32_t v = v_in;
    v = ((v & 0x55555555U) << 1U) | ((v >> 1U) & 0x55555555U);
    v = ((v & 0x33333333U) << 2U) | ((v >> 2U) & 0x33333333U);
    v = ((v & 0x0F0F0F0FU) << 4U) | ((v >> 4U) & 0x0F0F0F0FU);
    v = ((v & 0x00FF00FFU) << 8U) | ((v >> 8U) & 0x00FF00FFU);
    v = (v << 16U) | (v >> 16U);
    return v;
}

#endif /* PIO_SIM_INTERNAL_H */
