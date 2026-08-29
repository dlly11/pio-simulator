/* pio_sim_internal.h: helpers shared between the simulator and assembler
 * translation units. Private to lib/src; not installed with the public API. */
#ifndef PIO_SIM_INTERNAL_H
#define PIO_SIM_INTERNAL_H

#include "pio_sim.h"

#include <stdint.h>

/* Pad plumbing shared between pio_sim.c and pio_gpio.c. */

/* Chip-side drive after the function mux, IO_BANK0 overrides, pad OD and
 * (RP2350) isolation. What the chip presents to the pad, before external
 * drivers and pulls. */
void pio_pads_chip_drive(const pio_pads_t *p, uint64_t *oe_out, uint64_t *lvl_out);

/* Reset a pad set to the simulator's legacy-friendly defaults (IE on, no
 * pulls, FUNCSEL = LEGACY_ANY_PIO). Called by pio_sim_init / group init. */
void pio_pads_init_defaults(pio_pads_t *p);

/* Apply a write to funcsel[]. Refreshes the routing caches (pio_func_mask[],
 * periph_sel_mask), then re-resolves the pads. Both halves are one operation,
 * so this is the only entry point and a caller cannot do half of it. The
 * caches say which owner may drive a pad. pin_dirs says which one currently
 * does, and pio_pads_chip_drive reads pin_dirs. Implemented in pio_gpio.c;
 * pio_sim.c never changes funcsel, so only the pio_gpio API calls it. */
void pio_pads_apply_funcsel(pio_pads_t *p);

/* Recompute the PIO's claim on every pad (pin_dirs / pin_levels) from each
 * state machine's output registers, gated by the current function mux.
 * Implemented in pio_sim.c. */
void pio_pads_resolve(pio_pads_t *p);

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
