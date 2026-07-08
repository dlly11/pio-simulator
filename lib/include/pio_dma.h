/*
 * SPDX-License-Identifier: MIT
 * pio_dma.h — RP2040/RP2350-style DMA controller model for pio_sim.
 *
 * Models the DMA block as firmware sees it: 12 channels on RP2040, 16 on
 * RP2350 (PIO_SIM_DMA_NUM_CHANNELS), each with read/write address generation
 * (increment on/off, power-of-two ring wrap on one side), 8/16/32-bit data
 * size, byte swap, DREQ pacing (PIO TX/RX requests, four fractional pacing
 * timers, or unpaced), chaining with null-trigger self-stop, IRQ_QUIET,
 * per-channel completion IRQs routed through INTR/INTE/INTF/INTS system
 * lines (2 on RP2040, 4 on RP2350), the CRC sniffer, channel abort, and
 * high/normal priority round-robin arbitration. MEM-to-MEM transfers work
 * with TREQ_FORCE. RP2350 MPU/security attributes are out of scope.
 *
 * Documented simplifications (deliberate, all noted at the relevant API):
 *  - One transfer per pio_dma_tick across the whole controller, chosen by
 *    priority-then-round-robin. Real silicon pipelines separate read/write
 *    masters; for PIO workloads the FIFOs are the bottleneck, and one
 *    element per clk_sys tick gives strict, reproducible ordering.
 *  - DREQ readiness is level-sensitive, re-sampled every tick (TX FIFO not
 *    full / RX FIFO not empty), not the edge-counted credit scheme silicon
 *    uses. At one transfer per tick the two are behaviour-equivalent for
 *    FIFO endpoints.
 *  - A transfer is atomic (read, byte-swap, sniff, write in one tick), so
 *    CHAN_ABORT completes immediately — no in-flight write to drain.
 *
 * Call pio_dma_tick once per system tick, after pio_sim_tick /
 * pio_sim_group_tick, so the DMA sees the FIFO state the SMs just produced
 * (pio_chip_tick in pio_chip.h does exactly this).
 */

#ifndef PIO_DMA_H
#define PIO_DMA_H

#include "pio_sim.h"

/* Included directly (IWYU): this header uses bool/uint*_t itself. */
#include <stdbool.h>
#include <stdint.h>

/* ── Transfer endpoints ────────────────────────────────────────────────────── */

typedef enum {
    PIO_DMA_SIZE_8 = 0,  /* byte transfers     */
    PIO_DMA_SIZE_16 = 1, /* halfword transfers */
    PIO_DMA_SIZE_32 = 2, /* word transfers     */
} pio_dma_size_t;

typedef enum {
    PIO_DMA_ADDR_MEM = 0,     /* host memory (a real pointer)          */
    PIO_DMA_ADDR_PIO_TXF = 1, /* a state machine's TX FIFO (write end) */
    PIO_DMA_ADDR_PIO_RXF = 2, /* a state machine's RX FIFO (read end)  */
} pio_dma_addr_kind_t;

/* A transfer endpoint: host memory or a PIO FIFO of an attached block.
 * FIFO endpoints never increment; the ring applies to MEM endpoints only. */
typedef struct {
    pio_dma_addr_kind_t kind;
    unsigned char *mem; /* MEM: byte cursor (advances if incr_*)    */
    uint8_t pio_index;  /* TXF/RXF: index into the attached blocks  */
    uint8_t sm;
} pio_dma_addr_t;

pio_dma_addr_t pio_dma_addr_mem(void *p);
pio_dma_addr_t pio_dma_addr_txf(uint8_t pio_index, uint8_t sm);
pio_dma_addr_t pio_dma_addr_rxf(uint8_t pio_index, uint8_t sm);

/* ── TREQ / DREQ numbering (matches the datasheet DREQ table) ──────────────── */

#define PIO_DMA_DREQ_PIO_TX(pio_index, sm) ((uint8_t)(((pio_index) * 8U) + (sm)))
#define PIO_DMA_DREQ_PIO_RX(pio_index, sm) ((uint8_t)(((pio_index) * 8U) + 4U + (sm)))
#define PIO_DMA_TREQ_TIMER(n) ((uint8_t)(0x3BU + (n))) /* pacing timer 0..3 */
#define PIO_DMA_TREQ_FORCE 0x3FU                       /* unpaced: always ready */

/* ── Channel configuration (mirrors CHx_CTRL) ──────────────────────────────── */

typedef struct {
    bool high_priority;        /* arbitration class                          */
    pio_dma_size_t data_size;  /* element size                               */
    bool incr_read;            /* advance the read MEM cursor per element    */
    bool incr_write;           /* advance the write MEM cursor per element   */
    uint8_t ring_size;         /* 0: none; else wrap 2^ring_size BYTES —     */
    bool ring_sel;             /*   false: wrap read addr, true: write addr. */
                               /*   The buffer must be 2^ring_size aligned,  */
                               /*   as on hardware.                          */
    uint8_t chain_to;          /* channel triggered on completion (== own    */
                               /*   channel number: no chain)                */
    uint8_t treq_sel;          /* DREQ/TREQ number (see the macros above)    */
    bool bswap;                /* byte-reverse each element                  */
    bool irq_quiet;            /* raise the completion IRQ only on a null    */
                               /*   trigger, for control-block chains        */
    bool sniff_en;             /* feed this channel's data to the sniffer    */
} pio_dma_channel_config_t;

/* ── Controller ────────────────────────────────────────────────────────────── */

struct pio_dma;
typedef void (*pio_dma_callback_t)(struct pio_dma *d, uint8_t ch, void *ctx);

typedef struct {
    pio_dma_channel_config_t ctrl;
    pio_dma_addr_t read_addr;
    pio_dma_addr_t write_addr;
    uint32_t trans_count;        /* transfers remaining in this run       */
    uint32_t trans_count_reload; /* value a (re)trigger loads             */
    bool en;                     /* CTRL.EN: channel participates at all  */
    bool busy;
    pio_dma_callback_t on_complete;
    void *cb_ctx;
} pio_dma_channel_t;

/* Sniffer CALC values (SNIFF_CTRL). The CRC register is SNIFF_DATA: seed it
 * with pio_dma_sniffer_set_data (0xFFFFFFFF for the standard CRC-32/CCITT
 * presets) and read the result with pio_dma_sniffer_get_data, which applies
 * OUT_REV/OUT_INV. */
typedef enum {
    PIO_DMA_SNIFF_CRC32 = 0x0,  /* poly 0x04C11DB7, MSB-first             */
    PIO_DMA_SNIFF_CRC32R = 0x1, /* bit-reversed data (reflected update)   */
    PIO_DMA_SNIFF_CRC16 = 0x2,  /* CRC-16-CCITT, poly 0x1021, MSB-first   */
    PIO_DMA_SNIFF_CRC16R = 0x3, /* bit-reversed data (reflected update)   */
    PIO_DMA_SNIFF_EVEN = 0xE,   /* per-bit-lane XOR (even parity)         */
    PIO_DMA_SNIFF_SUM = 0xF,    /* simple 32-bit summation                */
} pio_dma_sniff_calc_t;

typedef struct pio_dma {
    pio_dma_channel_t ch[PIO_SIM_DMA_NUM_CHANNELS];

    /* Attached PIO blocks: index n serves DREQ numbers n*8+…; must match the
     * FIFO endpoints' pio_index. */
    pio_sim_t *pio[PIO_SIM_NUM_PIO];
    uint8_t pio_count;

    /* Fractional pacing timers: fire at clk_sys × X / Y. */
    struct {
        uint16_t x, y;
        uint32_t accum;
        bool credit; /* level-latched: at most one pending request */
    } timer[4];

    struct {
        bool en;
        uint8_t chan;
        uint8_t calc; /* pio_dma_sniff_calc_t */
        bool out_rev, out_inv;
        uint32_t data; /* SNIFF_DATA register */
    } sniff;

    uint32_t intr;                          /* raw completion flags (W1C)   */
    uint32_t inte[PIO_SIM_DMA_NUM_IRQS];    /* per-line enable              */
    uint32_t intf[PIO_SIM_DMA_NUM_IRQS];    /* per-line force               */
    uint8_t rr_next;                        /* round-robin arbitration seat */
} pio_dma_t;

/** Attach the controller to `pio_count` PIO blocks (slot n serves the DREQs
 * and FIFO endpoints with pio_index n). All channels reset disabled. */
void pio_dma_init(pio_dma_t *d, pio_sim_t *const *pios, uint8_t pio_count);

/** The pico-sdk-shaped default config: 32-bit, increment read, don't
 * increment write, no ring, chain to itself (none), TREQ_FORCE, normal
 * priority, no bswap/quiet/sniff. */
void pio_dma_channel_get_default_config(pio_dma_channel_config_t *c, uint8_t ch);

/** Program a channel and optionally trigger it — the everything-at-once
 * helper matching dma_channel_configure. Enables the channel. */
void pio_dma_channel_configure(pio_dma_t *d, uint8_t ch, const pio_dma_channel_config_t *c,
                               pio_dma_addr_t write_addr, pio_dma_addr_t read_addr,
                               uint32_t trans_count, bool trigger);

/** CTRL.EN: a disabled channel keeps its state but never transfers. */
void pio_dma_channel_set_enabled(pio_dma_t *d, uint8_t ch, bool en);

/** Trigger the channels in `mask`: each loads its reload count and goes
 * busy. Triggering with a reload count of 0 is a NULL trigger: the channel
 * does not start, and in IRQ_QUIET mode the completion IRQ fires instead
 * (the control-block chain termination idiom). Addresses are NOT reset —
 * reprogram them (or use ring wrap) before retriggering, as on hardware. */
void pio_dma_channel_start_mask(pio_dma_t *d, uint32_t mask);

/** CHAN_ABORT: stop the channels in `mask` dead. Immediate in this model
 * (transfers are per-tick atomic); no completion IRQ is raised. */
void pio_dma_channel_abort(pio_dma_t *d, uint32_t mask);

bool pio_dma_channel_is_busy(const pio_dma_t *d, uint8_t ch);
uint32_t pio_dma_channel_transfer_count(const pio_dma_t *d, uint8_t ch);

/** Per-channel completion hook, called whenever the channel's INTR bit is
 * raised (i.e. respecting IRQ_QUIET). */
void pio_dma_channel_set_callback(pio_dma_t *d, uint8_t ch, pio_dma_callback_t fn, void *ctx);

/** One bus cycle: advance the pacing timers, pick the one ready channel
 * (high-priority round-robin first, then normal) and move one element.
 * Returns true if an element moved. Call once per system tick, after the
 * PIO tick, so the DMA services what the SMs just produced. */
bool pio_dma_tick(pio_dma_t *d);

/* ── Pacing timers ─────────────────────────────────────────────────────────── */

/** Timer `t` (0..3) requests transfers at clk_sys × x / y (y = 0 disables). */
void pio_dma_timer_set(pio_dma_t *d, uint8_t t, uint16_t x, uint16_t y);

/* ── Sniffer ───────────────────────────────────────────────────────────────── */

void pio_dma_sniffer_enable(pio_dma_t *d, uint8_t ch, pio_dma_sniff_calc_t calc, bool out_rev,
                            bool out_inv);
void pio_dma_sniffer_disable(pio_dma_t *d);
void pio_dma_sniffer_set_data(pio_dma_t *d, uint32_t seed);
/** SNIFF_DATA with OUT_REV (bit-reverse) / OUT_INV (invert) applied. */
uint32_t pio_dma_sniffer_get_data(const pio_dma_t *d);

/* ── Interrupts ────────────────────────────────────────────────────────────── */

/** Enable (or disable) the channels in `mask` on system IRQ line `line`
 * (0..PIO_SIM_DMA_NUM_IRQS-1) — the INTE register. */
void pio_dma_set_irq_enabled(pio_dma_t *d, uint8_t line, uint32_t mask, bool enabled);

/** Raw completion flags (INTR). */
uint32_t pio_dma_get_intr(const pio_dma_t *d);

/** Line status (INTS): (INTR & INTE) | INTF. */
uint32_t pio_dma_get_ints(const pio_dma_t *d, uint8_t line);

/** Force (or clear a forced) interrupt on a line — the INTF register. */
void pio_dma_irq_force(pio_dma_t *d, uint8_t line, uint32_t mask, bool on);

/** Write-1-to-clear the INTR bits in `mask`. */
void pio_dma_acknowledge_irq(pio_dma_t *d, uint32_t mask);

#endif /* PIO_DMA_H */
