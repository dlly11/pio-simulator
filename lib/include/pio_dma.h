/*
 * SPDX-License-Identifier: MIT
 * pio_dma.h — RP2040/RP2350-style DMA controller model for pio_sim.
 *
 * Models the DMA block as firmware sees it: 12 channels on RP2040, 16 on
 * RP2350 (PIO_SIM_DMA_NUM_CHANNELS), each with read/write address generation
 * (increment on/off, power-of-two ring wrap on one side), 8/16/32-bit data
 * size, byte swap, DREQ pacing (PIO TX/RX requests, fractional pacing timers,
 * or unpaced), chaining with null-trigger self-stop, IRQ_QUIET, per-channel
 * completion IRQs routed through INTR/INTE/INTF/INTS system lines (2 on
 * RP2040, 4 on RP2350), the CRC sniffer, channel abort, and high/normal
 * priority round-robin arbitration. MEM-to-MEM transfers work with TREQ_FORCE.
 * RP2350 MPU/security attributes are out of scope.
 *
 * API note: the surface mirrors the pico-sdk (channel_config_set_*,
 * dma_channel_configure, dma_irqn_set_channel_enabled, dma_sniffer_*,
 * dma_timer_set_fraction). Because there is no global hardware instance, the
 * functions that act on the controller take a pio_dma_t* first; the pure
 * config mutators use the exact SDK names so firmware code ports verbatim.
 * Simulator-only helpers (raw INTR/INTS read-back, INTF force, the callback
 * hook) are labelled "sim extension".
 *
 * Documented simplifications (deliberate):
 *  - One transfer per pio_dma_tick across the whole controller, chosen by
 *    priority-then-round-robin (real silicon pipelines separate masters).
 *  - DREQ readiness is level-sensitive, re-sampled every tick.
 *  - A transfer is atomic, so CHAN_ABORT completes immediately.
 *
 * Call pio_dma_tick once per system tick, after pio_sim_tick /
 * pio_sim_group_tick (pio_chip_tick does this).
 */

#ifndef PIO_DMA_H
#define PIO_DMA_H

#include "pio_sim.h"

/* Included directly (IWYU): this header uses bool/uint*_t itself. */
#include <stdbool.h>
#include <stdint.h>

/* ── Transfer endpoints ────────────────────────────────────────────────────── */

/* Element size — SDK-spelled (enum dma_channel_transfer_size). */
typedef enum {
    DMA_SIZE_8 = 0,  /* byte transfers     */
    DMA_SIZE_16 = 1, /* halfword transfers */
    DMA_SIZE_32 = 2, /* word transfers     */
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

/** Memory endpoint at `p`. The buffer must outlive every transfer that uses it
 *  (the engine holds the pointer, it does not copy). */
pio_dma_addr_t pio_dma_addr_mem(void *p);
/** TX-FIFO endpoint of state machine `sm` on attached block `pio_index`. */
pio_dma_addr_t pio_dma_addr_txf(uint8_t pio_index, uint8_t sm);
/** RX-FIFO endpoint of state machine `sm` on attached block `pio_index`. */
pio_dma_addr_t pio_dma_addr_rxf(uint8_t pio_index, uint8_t sm);

/* ── TREQ / DREQ numbering (matches the datasheet DREQ table) ──────────────── */

#define PIO_DMA_DREQ_PIO_TX(pio_index, sm) ((uint8_t)(((pio_index) * 8U) + (sm)))
#define PIO_DMA_DREQ_PIO_RX(pio_index, sm) ((uint8_t)(((pio_index) * 8U) + 4U + (sm)))
/* Pacing timer n in [0, PIO_SIM_DMA_NUM_TIMERS). */
#define PIO_DMA_TREQ_TIMER(n) ((uint8_t)(0x3BU + (n)))
#define PIO_DMA_TREQ_FORCE 0x3FU /* unpaced: always ready */

/* ── Channel configuration (mirrors CHx_CTRL) ──────────────────────────────────
 * Build a config with pio_dma_channel_get_default_config() then the
 * channel_config_set_* mutators below — as with the pico-sdk's
 * dma_channel_config, the fields are internal and should not be written
 * directly. (True compile-time hiding of a stack value type isn't possible in
 * C without aliasing hazards, so the fields remain declared, exactly as in the
 * SDK, but the setters are the supported interface.) */
typedef struct {
    bool high_priority;       /* private — use channel_config_set_high_priority */
    pio_dma_size_t data_size; /* private — use channel_config_set_transfer_data_size */
    bool incr_read;           /* private — use channel_config_set_read_increment  */
    bool incr_write;          /* private — use channel_config_set_write_increment */
    uint8_t ring_size;        /* private — use channel_config_set_ring */
    bool ring_sel;            /* private — 2^ring_size-byte-aligned wrap */
    uint8_t chain_to;         /* private — use channel_config_set_chain_to */
    uint8_t treq_sel;         /* private — use channel_config_set_dreq */
    bool bswap;               /* private — use channel_config_set_bswap */
    bool irq_quiet;           /* private — use channel_config_set_irq_quiet */
    bool sniff_en;            /* private — use channel_config_set_sniff_enable */
} dma_channel_config;

/* SDK-named config mutators (config value only, no controller object). */
void channel_config_set_read_increment(dma_channel_config *c, bool incr);
void channel_config_set_write_increment(dma_channel_config *c, bool incr);
/** Data-request source: a PIO_DMA_DREQ_PIO_* level, PIO_DMA_TREQ_TIMER(n), or
 *  PIO_DMA_TREQ_FORCE. An unrecognised value leaves the channel permanently
 *  un-paced (never ready) rather than erroring — pick from the macros above. */
void channel_config_set_dreq(dma_channel_config *c, uint8_t dreq);
void channel_config_set_chain_to(dma_channel_config *c, uint8_t chain_to);
void channel_config_set_transfer_data_size(dma_channel_config *c, pio_dma_size_t size);
/** Ring wrap: `write` selects the address (false = read, true = write) and
 * `size_bits` the 2^size_bits-byte window (0 = none). The hardware RING_SIZE
 * field is 4 bits, so `size_bits` is clamped to 0..15. */
void channel_config_set_ring(dma_channel_config *c, bool write, uint8_t size_bits);
void channel_config_set_bswap(dma_channel_config *c, bool bswap);
void channel_config_set_irq_quiet(dma_channel_config *c, bool irq_quiet);
void channel_config_set_high_priority(dma_channel_config *c, bool high_priority);
void channel_config_set_sniff_enable(dma_channel_config *c, bool sniff_enable);

/* ── Controller ────────────────────────────────────────────────────────────── */

struct pio_dma;
/** Per-channel completion hook (sim extension; no SDK analogue). */
typedef void (*pio_dma_callback_t)(struct pio_dma *d, uint8_t ch, void *ctx);

typedef struct {
    dma_channel_config ctrl;
    pio_dma_addr_t read_addr;
    pio_dma_addr_t write_addr;
    uint32_t trans_count;        /* transfers remaining in this run       */
    uint32_t trans_count_reload; /* value a (re)trigger loads             */
    bool en;                     /* CTRL.EN: channel participates at all  */
    bool busy;
    pio_dma_callback_t on_complete;
    void *cb_ctx;
} pio_dma_channel_t;

/* Sniffer CALC values (SNIFF_CTRL). Seed the accumulator with
 * pio_dma_sniffer_set_data_accumulator (0xFFFFFFFF for the standard CRC
 * presets) and read the result with pio_dma_sniffer_get_data_accumulator,
 * which applies the invert/reverse output options. */
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

    /* Fractional pacing timers: fire at clk_sys × numerator / denominator. */
    struct {
        uint16_t num, den;
        uint32_t accum;
        bool credit; /* level-latched: at most one pending request */
    } timer[PIO_SIM_DMA_NUM_TIMERS];

    struct {
        bool en;
        uint8_t chan;
        uint8_t calc; /* pio_dma_sniff_calc_t */
        bool out_rev, out_inv;
        uint32_t data; /* SNIFF_DATA register */
    } sniff;

    uint32_t intr;                       /* raw completion flags (W1C)   */
    uint32_t inte[PIO_SIM_DMA_NUM_IRQS]; /* per-line enable              */
    uint32_t intf[PIO_SIM_DMA_NUM_IRQS]; /* per-line force               */
    uint8_t rr_next;                     /* round-robin arbitration seat */
} pio_dma_t;

/** Attach the controller to `pio_count` PIO blocks (slot n serves the DREQs
 * and FIFO endpoints with pio_index n). All channels reset disabled. */
void pio_dma_init(pio_dma_t *d, pio_sim_t *const *pios, uint8_t pio_count);

/** The pico-sdk-shaped default config (returned by value, like
 * dma_channel_get_default_config): 32-bit, increment read, don't increment
 * write, no ring, chain to `ch` itself (none), TREQ_FORCE, normal priority. */
dma_channel_config pio_dma_channel_get_default_config(uint8_t ch);

/** Program a channel and optionally trigger it (SDK dma_channel_configure,
 * with the controller passed explicitly). Enables the channel. */
void pio_dma_channel_configure(pio_dma_t *d, uint8_t ch, const dma_channel_config *c,
                               pio_dma_addr_t write_addr, pio_dma_addr_t read_addr,
                               uint32_t trans_count, bool trigger);

/** CTRL.EN: a disabled channel keeps its state but never transfers. */
void pio_dma_channel_set_enabled(pio_dma_t *d, uint8_t ch, bool en);

/** Trigger the channels in `mask`: each loads its reload count and goes busy.
 * A reload count of 0 is a NULL trigger (does not start; in IRQ_QUIET raises
 * the completion IRQ — the control-block chain termination idiom). Addresses
 * are NOT reset. */
void pio_dma_start_channel_mask(pio_dma_t *d, uint32_t mask);

/** CHAN_ABORT: stop the channels in `mask` dead. Immediate (transfers are
 * per-tick atomic); no completion IRQ is raised. */
void pio_dma_channel_abort_mask(pio_dma_t *d, uint32_t mask);

/** Abort a single channel `ch` (SDK dma_channel_abort). */
void pio_dma_channel_abort(pio_dma_t *d, uint8_t ch);

/** True while the channel is transferring (CTRL.BUSY). */
bool pio_dma_channel_is_busy(const pio_dma_t *d, uint8_t ch);
/** Live remaining transfer count for `ch` (not the reload value). */
uint32_t pio_dma_channel_get_trans_count(const pio_dma_t *d, uint8_t ch);

/** Per-channel completion hook (sim extension), called whenever the channel's
 * INTR bit is raised (respecting IRQ_QUIET). */
void pio_dma_channel_set_callback(pio_dma_t *d, uint8_t ch, pio_dma_callback_t fn, void *ctx);

/** One bus cycle: advance the pacing timers, pick the one ready channel
 * (high-priority round-robin first, then normal) and move one element.
 * Returns true if an element moved. Call once per system tick, after the
 * PIO tick. */
bool pio_dma_tick(pio_dma_t *d);

/* ── Pacing timers ─────────────────────────────────────────────────────────── */

/** Timer `timer` requests transfers at clk_sys × numerator / denominator
 * (denominator 0 disables). SDK dma_timer_set_fraction. */
void pio_dma_timer_set_fraction(pio_dma_t *d, uint8_t timer, uint16_t numerator,
                                uint16_t denominator);

/* ── Sniffer (SDK dma_sniffer_*) ───────────────────────────────────────────── */

/** Enable the sniffer on channel `ch` with CRC/checksum `mode`. */
void pio_dma_sniffer_enable(pio_dma_t *d, uint8_t ch, pio_dma_sniff_calc_t mode,
                            bool force_channel_enable);
/** Turn the sniffer off (SNIFF_CTRL EN=0); the accumulator is left untouched. */
void pio_dma_sniffer_disable(pio_dma_t *d);
/** Bit-invert the value returned by the accumulator getter (SNIFF_CTRL OUT_INV). */
void pio_dma_sniffer_set_output_invert_enabled(pio_dma_t *d, bool invert);
/** Bit-reverse the value returned by the accumulator getter (SNIFF_CTRL OUT_REV). */
void pio_dma_sniffer_set_output_reverse_enabled(pio_dma_t *d, bool reverse);
/** Seed the sniff accumulator (SNIFF_DATA), e.g. a CRC preload. */
void pio_dma_sniffer_set_data_accumulator(pio_dma_t *d, uint32_t seed_value);
/** SNIFF_DATA with the invert/reverse output options applied. */
uint32_t pio_dma_sniffer_get_data_accumulator(const pio_dma_t *d);

/* ── Interrupts (SDK dma_irqn_*) ───────────────────────────────────────────── */

/** Enable/disable channel `ch`'s completion IRQ on system line `irq_index`
 * (INTE). Toggle-with-bool, matching dma_irqn_set_channel_enabled. */
void pio_dma_irqn_set_channel_enabled(pio_dma_t *d, uint8_t irq_index, uint8_t ch, bool enabled);
/** Same for every channel in `channel_mask` (dma_irqn_set_channel_mask_enabled). */
void pio_dma_irqn_set_channel_mask_enabled(pio_dma_t *d, uint8_t irq_index, uint32_t channel_mask,
                                           bool enabled);
/** INTS bit for `ch` on line `irq_index` (dma_irqn_get_channel_status). */
bool pio_dma_irqn_get_channel_status(const pio_dma_t *d, uint8_t irq_index, uint8_t ch);
/** W1C the completion flag for `ch` (dma_irqn_acknowledge_channel). */
void pio_dma_irqn_acknowledge_channel(pio_dma_t *d, uint8_t irq_index, uint8_t ch);

/** Raw completion flags, INTR (sim extension read-back). */
uint32_t pio_dma_get_intr(const pio_dma_t *d);
/** Line status, INTS = (INTR & INTE) | INTF (sim extension read-back). */
uint32_t pio_dma_get_ints(const pio_dma_t *d, uint8_t irq_index);
/** INTE / INTF register read-back for line `line` (sim extensions). */
uint32_t pio_dma_get_inte(const pio_dma_t *d, uint8_t irq_index);
uint32_t pio_dma_get_intf(const pio_dma_t *d, uint8_t irq_index);
/** Force (or clear) channel `ch`'s IRQ on line `irq_index` — INTF. Sim
 * extension: the SDK exposes no DMA force helper. */
void pio_dma_irqn_force_channel(pio_dma_t *d, uint8_t irq_index, uint8_t ch, bool on);

#endif /* PIO_DMA_H */
