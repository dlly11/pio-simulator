/*
 * SPDX-License-Identifier: MIT
 * pio.h — umbrella header pulling in the whole pio_sim public API in one
 * include: the instruction-set simulator, the assembler, the GPIO pad/mux
 * model, the clock tree, the DMA controller, and the chip-level umbrella.
 * Include the individual headers instead if you only need part of the surface.
 */

#ifndef PIO_H
#define PIO_H

#include "pio_sim.h"

#include "pio_asm.h"
#include "pio_chip.h"
#include "pio_clk.h"
#include "pio_dma.h"
#include "pio_gpio.h"

#endif /* PIO_H */
