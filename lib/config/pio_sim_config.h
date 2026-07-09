/*
 * SPDX-License-Identifier: MIT
 * pio_sim_config.h — compile-time configuration for the PIO simulator.
 *
 * Selects the target PIO hardware so the simulator and assembler expose only
 * the feature set that chip actually has:
 *
 *   PIO_SIM_PIO_VERSION 0  →  RP2040  (PIO "version 0")
 *   PIO_SIM_PIO_VERSION 1  →  RP2350  (PIO "version 1", a superset of v0)
 *
 * Set it in one of two ways:
 *   1. A compiler/project macro, e.g. -DPIO_SIM_PIO_VERSION=0 (or, via CMake,
 *      -DPIO_SIM_PLATFORM=RP2040). A command-line definition wins because the
 *      #ifndef below leaves it untouched.
 *   2. Vendoring your own copy of this header earlier on the include path
 *      (lib/config is on the public include path). Edit the default here.
 *
 * The integer maps 1:1 to `pioasm -v <0|1>`, so the encoding differential test
 * checks the in-tree assembler against the matching real-pioasm version.
 *
 * RP2350-only ("v1") API is removed from v0 builds, so using it on an RP2040
 * target is a compile error rather than a silent runtime surprise. The feature
 * aliases below name each gated capability so call sites read intent, not a
 * bare version number.
 */

#ifndef PIO_SIM_CONFIG_H
#define PIO_SIM_CONFIG_H

#ifndef PIO_SIM_PIO_VERSION
#define PIO_SIM_PIO_VERSION 1 /* default: RP2350 (superset; non-breaking) */
#endif

#if (PIO_SIM_PIO_VERSION != 0) && (PIO_SIM_PIO_VERSION != 1)
#error "PIO_SIM_PIO_VERSION must be 0 (RP2040) or 1 (RP2350)"
#endif

/* Feature gates derived from the selected PIO version. */
#define PIO_SIM_HAS_RXFIFO_MOV (PIO_SIM_PIO_VERSION >= 1)   /* mov rxfifo[]/osr   */
#define PIO_SIM_HAS_IRQ_STATUS (PIO_SIM_PIO_VERSION >= 1)   /* MOV STATUS <- IRQ  */
#define PIO_SIM_HAS_WAIT_JMPPIN (PIO_SIM_PIO_VERSION >= 1)  /* wait ... jmppin    */
#define PIO_SIM_HAS_IRQ_PREVNEXT (PIO_SIM_PIO_VERSION >= 1) /* irq/wait prev/next */
#define PIO_SIM_HAS_GPIO_BASE (PIO_SIM_PIO_VERSION >= 1)    /* GPIOBASE 0/16 win. */
#define PIO_SIM_HAS_MOV_PINDIRS (PIO_SIM_PIO_VERSION >= 1)  /* mov pindirs, <src> */
#define PIO_SIM_HAS_IN_PIN_COUNT (PIO_SIM_PIO_VERSION >= 1) /* IN pin count mask  */
#define PIO_SIM_HAS_INTR_IRQ8 (PIO_SIM_PIO_VERSION >= 1)    /* 8 IRQ flags in INTR */
#define PIO_SIM_HAS_PAD_ISO (PIO_SIM_PIO_VERSION >= 1)      /* pad isolation latch */
#define PIO_SIM_HAS_FUNCSEL_PIO2 (PIO_SIM_PIO_VERSION >= 1) /* FUNCSEL F8 = PIO2  */

/* Number of PIO blocks in the device: RP2040 has 2 (PIO0/1), RP2350 has 3
 * (PIO0/1/2). Used by the multi-PIO group helper. */
#if PIO_SIM_PIO_VERSION >= 1
#define PIO_SIM_NUM_PIO 3U
#else
#define PIO_SIM_NUM_PIO 2U
#endif

/* DMA controller shape: RP2040 has 12 channels and 2 system IRQ outputs;
 * RP2350 has 16 channels and 4. (Used by the pio_dma engine.) */
#if PIO_SIM_PIO_VERSION >= 1
#define PIO_SIM_DMA_NUM_CHANNELS 16U
#define PIO_SIM_DMA_NUM_IRQS 4U
#else
#define PIO_SIM_DMA_NUM_CHANNELS 12U
#define PIO_SIM_DMA_NUM_IRQS 2U
#endif

/* PLL_SYS parameter limits (used by the pio_clock module): the VCO floor is
 * 750 MHz on RP2040 and 400 MHz on RP2350; both cap at 1600 MHz. */
#if PIO_SIM_PIO_VERSION >= 1
#define PIO_SIM_PLL_VCO_MIN_HZ 400000000ULL
#else
#define PIO_SIM_PLL_VCO_MIN_HZ 750000000ULL
#endif
#define PIO_SIM_PLL_VCO_MAX_HZ 1600000000ULL

#endif /* PIO_SIM_CONFIG_H */
