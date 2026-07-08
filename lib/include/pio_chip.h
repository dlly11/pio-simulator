/*
 * SPDX-License-Identifier: MIT
 * pio_chip.h — a thin chip-level umbrella composing every PIO block of the
 * device (2 on RP2040, 3 on RP2350) around one shared pad set, the DMA
 * controller, and the clock tree. Purely compositional: everything here can
 * be assembled by hand from pio_sim/pio_dma/pio_clock — this just wires the
 * common whole-chip case in one call.
 *
 * pio_chip_tick runs the PIO group first, then the DMA, so the controller
 * services the FIFO state the SMs produced this very tick (a fixed one-tick
 * service latency, deterministic by construction).
 */

#ifndef PIO_CHIP_H
#define PIO_CHIP_H

#include "pio_clock.h"
#include "pio_dma.h"
#include "pio_sim.h"

typedef struct {
    pio_sim_t blocks[PIO_SIM_NUM_PIO];
    pio_sim_group_t grp; /* shared pads + lockstep ticking */
    pio_dma_t dma;
    pio_clk_tree_t clk;
} pio_chip_t;

/** Initialise every block, share one pad set across them (owner slot n =
 * FUNC_PIOn), attach the DMA controller to all blocks, and load the default
 * clock tree (125/150 MHz). */
void pio_chip_init(pio_chip_t *c);

/** One clk_sys cycle for the whole chip: PIO group tick, then one DMA bus
 * cycle. */
void pio_chip_tick(pio_chip_t *c);

/** Run `n` chip ticks. */
void pio_chip_run(pio_chip_t *c, uint64_t n);

/** Block accessor (0 = PIO0 …). */
pio_sim_t *pio_chip_pio(pio_chip_t *c, uint8_t index);

/** Wall-clock duration of `ticks` chip cycles under the configured tree. */
uint64_t pio_chip_ticks_to_ns(const pio_chip_t *c, uint64_t ticks);

#endif /* PIO_CHIP_H */
