/*
 * SPDX-License-Identifier: MIT
 * pio_chip.h — a thin chip-level umbrella composing every PIO block of the
 * device (2 on RP2040, 3 on RP2350) around one shared pad set, the DMA
 * controller, and the clock tree. Purely compositional: everything here can
 * be assembled by hand from pio_sim/pio_dma/pio_clk — this just wires the
 * common whole-chip case in one call.
 *
 * pio_chip_tick runs the PIO group first, then the DMA, so the controller
 * services the FIFO state the SMs produced this very tick (a fixed one-tick
 * service latency, deterministic by construction).
 */

#ifndef PIO_CHIP_H
#define PIO_CHIP_H

/* Included directly (IWYU): this header uses uint*_t itself. */
#include <stdint.h>

#include "pio_clk.h"
#include "pio_dma.h"
#include "pio_sim.h"

/** Whole-chip composition: all PIO blocks + shared pads + DMA + clock tree. */
typedef struct {
    pio_sim_t blocks[PIO_SIM_NUM_PIO]; /**< the device's PIO blocks (2 RP2040, 3 RP2350) */
    pio_sim_group_t grp;               /**< shared pads + lockstep ticking */
    pio_dma_t dma;                     /**< DMA controller attached to all blocks */
    pio_clk_tree_t clk;                /**< clk_sys clock tree */
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

/** Block accessor (0 = PIO0 …). Non-const: the returned block is mutable
 * (you configure and drive it). */
pio_sim_t *pio_chip_pio(pio_chip_t *c, uint8_t index);

/** Wall-clock duration of `ticks` chip cycles under the configured tree, and the
 * inverse conversions — thin wrappers over the embedded pio_clk tree. As in
 * pio_clk.h, ticks_to_* round to nearest and *_to_ticks round up (ceiling, so a
 * derived deadline is never short). */
uint64_t pio_chip_ticks_to_ns(const pio_chip_t *c, uint64_t ticks);
uint64_t pio_chip_ticks_to_us(const pio_chip_t *c, uint64_t ticks);
uint64_t pio_chip_ns_to_ticks(const pio_chip_t *c, uint64_t ns);
uint64_t pio_chip_us_to_ticks(const pio_chip_t *c, uint64_t us);

#endif /* PIO_CHIP_H */
