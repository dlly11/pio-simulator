/*
 * SPDX-License-Identifier: MIT
 * pio_chip.c — chip-level composition of PIO blocks + DMA + clock tree.
 */

#include "pio_chip.h"

void pio_chip_init(pio_chip_t *c)
{
    pio_sim_t *ptrs[PIO_SIM_NUM_PIO];
    for (uint8_t i = 0; i < PIO_SIM_NUM_PIO; i++) {
        pio_sim_init(&c->blocks[i]);
        ptrs[i] = &c->blocks[i];
    }
    pio_sim_group_init_shared(&c->grp, ptrs, PIO_SIM_NUM_PIO);
    pio_dma_init(&c->dma, ptrs, PIO_SIM_NUM_PIO);
    pio_clk_init_default(&c->clk);
}

void pio_chip_tick(pio_chip_t *c)
{
    pio_sim_group_tick(&c->grp);
    (void)pio_dma_tick(&c->dma);
}

void pio_chip_run(pio_chip_t *c, uint64_t n)
{
    for (uint64_t i = 0; i < n; i++) {
        pio_chip_tick(c);
    }
}

/* Non-const container pointer: this hands back a *mutable* interior block
 * (callers configure and drive it), so a const parameter would force a
 * cast-away-const that -Wcast-qual rightly rejects. */
pio_sim_t *pio_chip_pio(pio_chip_t *c, uint8_t index)
{
    return &c->blocks[index % PIO_SIM_NUM_PIO];
}

uint64_t pio_chip_ticks_to_ns(const pio_chip_t *c, uint64_t ticks)
{
    return pio_clk_ticks_to_ns(&c->clk, ticks);
}
