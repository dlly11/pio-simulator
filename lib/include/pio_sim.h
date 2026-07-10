/*
 * SPDX-License-Identifier: MIT
 * pio_sim — a pure-C functional simulator for the RP2040/RP2350 PIO block.
 *
 * Implements the complete PIO instruction set (JMP, WAIT, IN, OUT, PUSH,
 * PULL, MOV, IRQ, SET) with side-set, delay, the OSR/ISR shift registers
 * (autopush / autopull), joinable 4-deep FIFOs, scratch registers X/Y, per-SM
 * clock dividers, and a shared 32-pin pad model so multiple state machines —
 * and an external device model — can interact on the same wires.
 *
 * This is host-side test infrastructure: pure C with no Pico SDK, RTOS, or
 * platform headers, so it builds anywhere a host C toolchain does.
 *
 * Cycle model: pio_sim_tick() advances the PIO by one *system* clock. Each
 * enabled state machine maintains a fractional clock-divider accumulator and
 * runs one SM cycle when it elapses. An SM cycle either consumes a pending
 * delay, retries a stalled instruction, or fetches and executes one
 * instruction (committing side-set, then arming any post-instruction delay).
 *
 * API contract:
 *  - `sm` selects a state machine (0..3). Out-of-range values are masked into
 *    range (sm & 3), like the `irq & 7` and `line & 1` masks — not reported.
 *  - The library is not thread-safe: drive a pio_sim_t (and anything sharing
 *    its pads) from one thread.
 *  - Lifetimes: a pio_sim_group_t must outlive its member blocks' use — group
 *    init repoints each block's `pads` into the group (a later pio_sim_init on
 *    a member resets that member to its own pads). DMA endpoint memory
 *    (pio_dma.h) must outlive the transfers that reference it.
 *  - The structs are transparent for test convenience; prefer the accessors,
 *    and never write pin/pad fields directly (see the pio_sm_t note).
 */

#ifndef PIO_SIM_H
#define PIO_SIM_H

#include "pio_sim_config.h"

#include <stdbool.h>
#include <stdint.h>

#define PIO_SIM_NUM_SM 4U
#define PIO_SIM_INSN_COUNT 32U
#define PIO_SIM_FIFO_DEPTH 4U /* per-SM depth; doubles to 8 when joined */
#define PIO_SIM_FIFO_MAX 8U
/* A state machine sees a 32-pin window into the GPIOs; on RP2350 that window can
 * be offset (GPIOBASE) so the device may expose up to 48 physical GPIOs. */
#define PIO_SIM_PIN_WINDOW 32U
#if PIO_SIM_HAS_GPIO_BASE
#define PIO_SIM_NUM_PINS 48U
#else
#define PIO_SIM_NUM_PINS 32U
#endif
#define PIO_SIM_NUM_IRQ 8U

/* ── FIFO ──────────────────────────────────────────────────────────────────── */

typedef struct {
    uint32_t buf[PIO_SIM_FIFO_MAX];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint8_t cap; /* 0, PIO_SIM_FIFO_DEPTH, or PIO_SIM_FIFO_MAX (joined) */
} pio_fifo_t;

/* ── Shift direction ───────────────────────────────────────────────────────── */

typedef enum {
    PIO_SHIFT_LEFT = 0,  /* MSB first */
    PIO_SHIFT_RIGHT = 1, /* LSB first */
} pio_shift_dir_t;

/* ── State machine ─────────────────────────────────────────────────────────── */

typedef struct {
    bool enabled;

    /* Program counter / wrap */
    uint8_t pc;
    uint8_t wrap_bottom; /* pc wraps to here … */
    uint8_t wrap_top;    /* … after executing this address */

    /* Scratch + shift registers */
    uint32_t x;
    uint32_t y;
    uint32_t osr;
    uint32_t isr;
    uint8_t osr_count; /* bits consumed from the OSR: 0 = full (fresh word), 32 = exhausted */
    uint8_t isr_count; /* bits accumulated in the ISR: 0 = empty, 32 = full                 */

    /* Shift configuration */
    pio_shift_dir_t out_dir;
    pio_shift_dir_t in_dir;
    bool autopull;
    bool autopush;
    uint8_t pull_thresh; /* 1..32 */
    uint8_t push_thresh; /* 1..32 */

    /* Pin mapping */
    uint8_t out_base;
    uint8_t out_count;
    uint8_t set_base;
    uint8_t set_count;
    uint8_t in_base;
    /* Number of IN-mapped pins (RP2350 PINCTRL IN_COUNT): pins at or above this
     * index read as 0 for IN PINS / MOV x,PINS / WAIT PIN. 32 (the default) means
     * no masking, matching RP2040 which has no IN pin count. */
    uint8_t in_count;
    uint8_t sideset_base;
    uint8_t jmp_pin;

    /* Side-set: total_bits includes the enable bit when opt is set */
    uint8_t sideset_total_bits;
    bool sideset_opt;
    bool sideset_pindirs;

    /* Per-SM output registers. out_pin_val latches the value of pins written via
     * OUT/SET/MOV PINS or side-set; out_pin_oe marks the pins driven as outputs
     * (the pindir register); wrote_this_cycle records which pins were actually
     * written during the current SM cycle. The pad applies each cycle's writes
     * with highest-SM-wins priority (hardware collates only *simultaneous*
     * writes — see resolve_pads); unwritten pins hold their latched level.
     * Mutate these only through the API / instructions, never directly, or the
     * resolved pad state desynchronises. */
    uint64_t out_pin_val;
    uint64_t out_pin_oe;
    uint64_t wrote_this_cycle;

    /* EXECCTRL output controls (sm_config_set_out_special). out_inline_en uses bit
     * out_en_sel of the OUT data as an output enable; when it is 0 the OUT does not
     * write the pins, and under out_sticky it releases them (clears out_pin_oe for
     * the OUT span) so a lower-priority SM / external level / pull shows through. */
    bool out_sticky;
    bool out_inline_en;
    uint8_t out_en_sel;

    /* MOV STATUS source. By default STATUS is derived from the live FIFO level
     * (or, on RP2350, an IRQ flag) per status_sel/status_n, returning all-ones
     * when the condition holds and all-zeros otherwise. A test may instead pin
     * a fixed value via pio_sim_sm_set_status_value (status_override). */
    uint8_t status_sel;   /* pio_status_sel_t */
    uint8_t status_n;     /* compare level (TX/RX) or IRQ index */
    bool status_override; /* true: read_source returns status_value verbatim */
    uint32_t status_value;

    /* Clock divider (16.8 fixed point) */
    uint16_t clkdiv_int;
    uint8_t clkdiv_frac;
    uint32_t clk_accum; /* fractional accumulator, 8-bit frac scaled */

    /* Execution bookkeeping */
    uint8_t delay;      /* remaining delay cycles                    */
    bool stalled;       /* last cycle the instruction could not run  */
    bool exec_pending;  /* an instruction was injected via OUT/MOV EXEC or sm_exec */
    uint16_t exec_insn; /* the injected instruction                  */

    /* Sticky FIFO-debug flags (FDEBUG): set on the corresponding event, cleared
     * by the host. See PIO_FDEBUG_* and pio_sim_sm_get_fdebug/clear_fdebug. */
    uint8_t fdebug;

    pio_fifo_t tx;
    pio_fifo_t rx;
    uint8_t fifo_join; /* pio_fifo_join_t: gates the SM-side RX put/get MOVs */
} pio_sm_t;

/* ── Pad model ─────────────────────────────────────────────────────────────────
 * The shared electrical state of the GPIOs. One bit per physical pin (up to
 * PIO_SIM_NUM_PINS, so 64-bit to cover RP2350's 48). Normally each PIO block owns
 * its own pads; a multi-PIO group can point several blocks at one shared
 * pio_pads_t so they drive and sample the same wires. */
struct pio_sim; /* owners[] below points back at the blocks sharing these pads */
typedef struct {
    /* Resolved PIO drive: recomputed from the owning state machines' per-SM output
     * registers on every pin/pindir write (see resolve_pads). pin_dirs marks pins
     * the PIO drives as outputs; pin_levels is their value. */
    uint64_t pin_levels; /* PIO-driven line level per GPIO   */
    uint64_t pin_dirs;   /* 1 = driven as output by the PIO  */

    /* Input synchroniser (always on, to match silicon): GPIO inputs reach the
     * PIO logic through two flip-flops, so the level a state machine samples
     * (IN PINS, JMP PIN, WAIT GPIO/PIN) lags the live pad by two system clocks.
     * in_sync[1] is what the PIO sees; in_sync[0] is the intermediate stage.
     * pio_sim_set_input_sync_bypass can disable the delay per pin. */
    uint64_t in_sync[2];

    /* External drive: pins the host/device drives via pio_sim_set_pin (and clears
     * with pio_sim_release_pin). Models an *off-chip* driver on the wire — a
     * peripheral inside the chip drives through the function mux instead (see
     * periph_oe/periph_level and pio_gpio.h). */
    uint64_t ext_drive;
    uint64_t ext_levels;

    /* ── Pad registers (PADS_BANK0, digital-relevant fields; see pio_gpio.h) ──
     * pad_pue/pad_pde model the pull resistors (both set = bus keeper, which
     * holds the last driven level in keep_state). pad_od forces the pad off;
     * pad_ie=0 makes the PIO read the pin as 0 (host pio_sim_get_pin still
     * returns the wire). DRIVE/SLEWFAST/SCHMITT are stored in pad_cfg but do
     * not affect the digital simulation. */
    uint64_t pad_ie;     /* input enable (reset: all-ones)          */
    uint64_t pad_od;     /* output disable                          */
    uint64_t pad_pue;    /* pull-up enable                          */
    uint64_t pad_pde;    /* pull-down enable                        */
    uint64_t keep_state; /* bus-keeper latch: last driven level     */
#if PIO_SIM_HAS_PAD_ISO
    uint64_t pad_iso;    /* RP2350 isolation: freezes output, gates input */
    uint64_t iso_levels; /* output level latched when ISO was set   */
    uint64_t iso_oe;     /* output enable latched when ISO was set  */
#endif
    uint8_t pad_cfg[PIO_SIM_NUM_PINS]; /* stored-only: DRIVE[1:0]|SLEW<<2|SCHMITT<<3 */

    /* ── IO_BANK0 function mux (see pio_gpio.h) ──
     * funcsel routes each pad's output/OE: a PIO block only drives pins whose
     * FUNCSEL selects it (owner slot i = FUNC_PIOi); other functions drive via
     * periph_oe/periph_level. Inputs are always visible to the PIO regardless
     * of funcsel, matching silicon. The default 0xFF (LEGACY_ANY_PIO) is a
     * simulator-only value letting every owner drive, preserving the pre-mux
     * behaviour. The *over masks implement OUTOVER/OEOVER/INOVER. */
    uint8_t funcsel[PIO_SIM_NUM_PINS];
    uint64_t pio_func_mask[PIO_SIM_NUM_PIO]; /* pins routed to owner slot i (cache) */
    uint64_t periph_sel_mask;                /* pins routed to a non-PIO function   */
    uint64_t periph_oe;                      /* selected peripheral's output enable per pin */
    uint64_t periph_level;                   /* selected peripheral's output level per pin  */
    uint64_t outover_inv, outover_low, outover_high; /* OUTOVER per-pin masks */
    uint64_t oeover_inv, oeover_low, oeover_high;    /* OEOVER  per-pin masks */
    uint64_t inover_inv, inover_low, inover_high;    /* INOVER  per-pin masks */

    /* Per-pin INPUT_SYNC_BYPASS: bits set here skip the two-cycle synchroniser. */
    uint64_t sync_bypass;

    /* PIO blocks whose state machines drive this pad set (this block alone, or all
     * blocks of a shared group). Resolution scans their per-SM output registers. */
    struct pio_sim *owners[PIO_SIM_NUM_PIO];
    uint8_t owner_count;
} pio_pads_t;

/* ── PIO block ─────────────────────────────────────────────────────────────── */

typedef struct pio_sim {
    uint16_t insn[PIO_SIM_INSN_COUNT];
    bool insn_used[PIO_SIM_INSN_COUNT];

    pio_sm_t sm[PIO_SIM_NUM_SM];

    uint8_t irq; /* 8 IRQ flags packed into the low byte */

    /* System interrupt lines IRQ0/IRQ1: per-line enable (INTE) and force (INTF)
     * masks over the INTR source bits (see PIO_INTR_*). The line asserts when
     * (INTR & INTE) | INTF is non-zero. */
    uint32_t irq_inte[2];
    uint32_t irq_intf[2];

    /* Pad model. By default `pads` points at the embedded set, so each block has
     * its own wires; pio_sim_group_init_shared can repoint several blocks at one
     * shared pio_pads_t. Access pads only through `pads`. */
    pio_pads_t pads_embedded;
    pio_pads_t *pads;

#if PIO_SIM_HAS_GPIO_BASE
    /* RP2350 GPIOBASE: offset (0 or 16) applied to every pin the state machines
     * access (IN/OUT/SET/side-set/MOV pins, WAIT GPIO/PIN, JMP PIN), so the
     * 32-pin window maps to GPIO 0-31 or 16-47. Host pin access
     * (pio_sim_get_pin / set_pin) is always absolute and unaffected. */
    uint8_t gpio_base;
#endif

    /* External device hook, invoked once per system tick *after* the SMs run.
     * It may read pin_levels / pin_dirs and drive pins the PIO has released. */
    void (*on_tick)(struct pio_sim *pio, void *ctx);
    void *device_ctx;

    /* Per-instruction trace hook (see pio_sim_set_trace): fires when an
     * instruction *commits* — never on stall or delay cycles — including
     * instructions injected via OUT/MOV EXEC or pio_sim_sm_exec. `pc` is the
     * address the word was fetched from (the parked PC for injected words);
     * read pio->sm[sm] through the pointer for the post-execute registers. */
    void (*on_insn)(const struct pio_sim *pio, uint8_t sm, uint8_t pc, uint16_t insn, void *ctx);
    void *trace_ctx;

    uint64_t cycle; /* system clocks elapsed */

    /* Diagnostic: counts instruction fetches from addresses never written by
     * pio_sim_load (an unwritten word decodes as `jmp 0`). A non-zero value
     * after a run usually means a runaway PC / mis-set wrap. Behaviour is
     * otherwise unchanged. */
    uint64_t unwritten_fetches;

#if PIO_SIM_HAS_IRQ_PREVNEXT
    /* RP2350: neighbouring PIO blocks addressed by `irq/wait ... prev|next`.
     * NULL (the default) means unlinked — prev/next set & clear have no effect
     * and a prev/next wait is never satisfied. Wire two or more blocks together
     * with pio_sim_set_irq_neighbors to model cross-PIO IRQ signalling. */
    struct pio_sim *irq_prev;
    struct pio_sim *irq_next;
#endif
} pio_sim_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────────── */

/** Reset the whole block: clears instruction memory, all SMs, pins, and IRQs. */
void pio_sim_init(pio_sim_t *pio);

/** Load `count` instruction words at `offset` into shared instruction memory. */
void pio_sim_load(pio_sim_t *pio, uint8_t offset, const uint16_t *insns, uint8_t count);

/** Reset a single state machine's execution state: PC to wrap_bottom, X/Y/OSR/ISR
 * and their counts, delay/stall/pending-exec, the clock accumulator, and both
 * FIFOs (the join capacity is preserved). The sticky FDEBUG flags are preserved
 * (cleared only via pio_sim_sm_clear_fdebug), matching pio_sim_sm_init. Note this
 * is broader than the SDK's pio_sm_restart, which leaves the PC and scratch
 * registers untouched; reposition the PC afterwards with pio_sim_sm_set_pc if you
 * need a specific start address. */
void pio_sim_sm_restart(pio_sim_t *pio, uint8_t sm);

/** Enable or disable a state machine. */
void pio_sim_sm_set_enabled(pio_sim_t *pio, uint8_t sm, bool enabled);

/** Whether state machine `sm` is currently enabled. */
bool pio_sim_sm_is_enabled(const pio_sim_t *pio, uint8_t sm);

/** Whether state machine `sm` is currently stalled: its last cycle's instruction
 * could not make progress — a WAIT not yet satisfied, a blocking PULL/PUSH on an
 * empty/full FIFO, or a stalled instruction injected via pio_sim_sm_exec / OUT-EXEC. */
bool pio_sim_sm_is_stalled(const pio_sim_t *pio, uint8_t sm);

/** Set the enabled/disabled state of the state machines in `sm_mask` (SDK
 * pio_set_sm_mask_enabled). Does not touch the clock dividers — use
 * pio_sim_enable_sm_mask_in_sync when phase alignment is required. */
void pio_sim_set_sm_mask_enabled(pio_sim_t *pio, uint8_t sm_mask, bool enabled);

/** Enable the state machines in `sm_mask` and restart their clock dividers in
 * phase (SDK pio_enable_sm_mask_in_sync) so equal-divider SMs run in lockstep. */
void pio_sim_enable_sm_mask_in_sync(pio_sim_t *pio, uint8_t sm_mask);

/* ── Configuration ────────────────────────────────────────────────────────────
 * The SDK config-struct pattern: build a pio_sm_config with
 * pio_get_default_sm_config() and the sm_config_set_* mutators, then apply it
 * with pio_sim_sm_init() (resets + sets PC) or pio_sim_sm_set_config() (no
 * reset). The mutator names/signatures match the pico-sdk (they act on the
 * config value only). The pio_sm_config type is declared after the
 * pio_status_sel_t / pio_fifo_join_t enums it references, below. */

/**
 * Reset the fractional clock-divider accumulator of every SM whose bit is set
 * in `sm_mask`, re-aligning their divided clocks in phase (mirrors the SDK's
 * pio_clkdiv_restart_sm_mask). SMs enabled on the same tick with equal dividers
 * run in lockstep; use this to re-align SMs that were started at different
 * ticks (e.g. the etm_4bit_lo/hi pair or the cross-SM JTAG programs).
 */
void pio_sim_clkdiv_restart_sm_mask(pio_sim_t *pio, uint8_t sm_mask);
/** Per-SM clock-divider restart (SDK pio_sm_clkdiv_restart). */
void pio_sim_sm_clkdiv_restart(pio_sim_t *pio, uint8_t sm);
/** Set the program counter directly. Does not restart the SM or clear a
 * latched stall/pending exec — use pio_sim_sm_restart for a full reset. */
void pio_sim_sm_set_pc(pio_sim_t *pio, uint8_t sm, uint8_t pc);

/** Current program counter of state machine `sm`. Mirrors the SDK's
 * pio_sm_get_pc; useful to observe where an SM has parked (e.g. on a stalled
 * WAIT) without reaching into the struct. */
uint8_t pio_sim_sm_get_pc(const pio_sim_t *pio, uint8_t sm);

/** Scratch / shift register accessors: the state after the last tick. The
 * struct fields are public too; these mirror the get_pc/get_instr style so
 * callers need not depend on the struct layout. */
uint32_t pio_sim_sm_get_x(const pio_sim_t *pio, uint8_t sm);
uint32_t pio_sim_sm_get_y(const pio_sim_t *pio, uint8_t sm);
uint32_t pio_sim_sm_get_isr(const pio_sim_t *pio, uint8_t sm);
uint32_t pio_sim_sm_get_osr(const pio_sim_t *pio, uint8_t sm);
/** Shift counts: bits accumulated in the ISR / consumed from the OSR (see the
 * isr_count/osr_count fields for the exact convention). */
uint8_t pio_sim_sm_get_isr_count(const pio_sim_t *pio, uint8_t sm);
uint8_t pio_sim_sm_get_osr_count(const pio_sim_t *pio, uint8_t sm);

/** Instruction word state machine `sm` would execute next: a stalled instruction
 * injected via OUT/MOV EXEC or pio_sim_sm_exec that is pending retry, else the
 * word at its PC. (A non-stalling injected instruction is consumed immediately
 * and does not linger here.) Mirrors reading the SDK's SMx_INSTR register. */
uint16_t pio_sim_sm_get_instruction(const pio_sim_t *pio, uint8_t sm);

/* MOV STATUS source selector (mirrors EXECCTRL_STATUS_SEL). */
typedef enum {
    PIO_STATUS_TX_LEVEL = 0, /* all-ones while TX FIFO level < N            */
    PIO_STATUS_RX_LEVEL = 1, /* all-ones while RX FIFO level < N            */
    /* RP2350: all-ones while IRQ flag N is set. On RP2040 (no IRQ status
     * source) this selector falls through to PIO_STATUS_TX_LEVEL behaviour. */
    PIO_STATUS_IRQ_SET = 2,
#if PIO_SIM_HAS_IRQ_STATUS && PIO_SIM_HAS_IRQ_PREVNEXT
    /* RP2350 selects the IRQ flag's PIO block via EXECCTRL STATUS_N[4:3]
     * (0 = this PIO, 1 = prev, 2 = next); modelled as distinct selectors here.
     * The block links come from pio_sim_set_irq_neighbors / group init; an
     * unlinked neighbour reads the flag as clear. */
    PIO_STATUS_IRQ_SET_PREV = 3, /* all-ones while IRQ flag N of the prev PIO is set */
    PIO_STATUS_IRQ_SET_NEXT = 4, /* all-ones while IRQ flag N of the next PIO is set */
#endif
} pio_status_sel_t;

/** Pin MOV STATUS to a fixed value (sim extension, no SDK analogue: a test
 * override bypassing the FIFO/IRQ derivation). Runtime — not part of the
 * config struct. Clear the override with pio_sim_sm_clear_status_value. */
void pio_sim_sm_set_status_value(pio_sim_t *pio, uint8_t sm, uint32_t value);

/** Remove the MOV STATUS override set by pio_sim_sm_set_status_value, restoring
 * the normal FIFO/IRQ-derived status (sim extension). */
void pio_sim_sm_clear_status_value(pio_sim_t *pio, uint8_t sm);

/** Read back the MOV STATUS override (sim extension). Returns true if an
 * override is active, writing its value to *value when non-NULL; returns false
 * when the normal FIFO/IRQ-derived status is in effect. */
bool pio_sim_sm_get_status_value(const pio_sim_t *pio, uint8_t sm, uint32_t *value);

/* FIFO join: 0 = none (4+4), 1 = join TX (8 TX, 0 RX), 2 = join RX (8 RX, 0 TX).
 * RP2350 adds the random-access RX modes (4-entry register file addressed by
 * the MOV-RX-FIFO instructions): PUT (SM writes, system reads), GET (system
 * writes, SM reads), and PUTGET (both FJOIN_RX_PUT and FJOIN_RX_GET: the SM has
 * random-access put *and* get — a scratchpad). The direction restrictions are
 * enforced: an SM put is a no-op outside PUT/PUTGET, an SM get outside
 * GET/PUTGET. */
typedef enum {
    PIO_FIFO_JOIN_NONE = 0,
    PIO_FIFO_JOIN_TX = 1,
    PIO_FIFO_JOIN_RX = 2,
    PIO_FIFO_JOIN_RX_PUT = 3,    /* RP2350: SM writes RX via `mov rxfifo[], isr`  */
    PIO_FIFO_JOIN_RX_GET = 4,    /* RP2350: SM reads RX via `mov osr, rxfifo[]`   */
    PIO_FIFO_JOIN_RX_PUTGET = 5, /* RP2350: both put and get (SM scratch file)    */
} pio_fifo_join_t;

/** Runtime pindir write for a span of pins (SDK pio_sm_set_consecutive_pindirs):
 * marks `count` pins from `base` as output (`is_out`) or input. Not part of the
 * config struct — call it after pio_sim_sm_init as on hardware. */
void pio_sim_sm_set_consecutive_pindirs(pio_sim_t *pio, uint8_t sm, uint8_t base, uint8_t count,
                                        bool is_out);

/* ── SM configuration value (pio_sm_config) ────────────────────────────────────
 * Mirrors the pico-sdk pio_sm_config: an inert value you build with the
 * sm_config_set_* mutators, then apply with pio_sim_sm_init / _set_config.
 * The fields are internal — use the mutators. */
typedef struct {
    uint8_t out_base, out_count;
    uint8_t set_base, set_count;
    uint8_t in_base, in_count;
    uint8_t sideset_base;
    uint8_t sideset_total_bits; /* data bits + opt-enable bit (SDK convention) */
    bool sideset_opt, sideset_pindirs;
    uint8_t jmp_pin;
    pio_shift_dir_t out_dir, in_dir;
    bool autopull, autopush;
    uint8_t pull_thresh, push_thresh;
    uint16_t clkdiv_int;
    uint8_t clkdiv_frac;
    uint8_t wrap_bottom, wrap_top;
    uint8_t fifo_join;  /* pio_fifo_join_t */
    uint8_t status_sel; /* pio_status_sel_t */
    uint8_t status_n;
    bool out_sticky, out_inline_en;
    uint8_t out_en_sel;
} pio_sm_config;

/** The reset-default config (SDK pio_get_default_sm_config): wrap over the
 * whole instruction memory, shift left, thresholds 32, clkdiv 1.0, no
 * autopush/pull, IN pin count unmasked. */
pio_sm_config pio_get_default_sm_config(void);

/* Config mutators, acting on the config value only. Names match the pico-sdk
 * exactly except the split *_pin_base / *_pin_count variants, which are sim
 * extensions (the SDK sets base+count together via *_pins). */
void sm_config_set_out_pins(pio_sm_config *c, uint8_t out_base, uint8_t out_count);
void sm_config_set_out_pin_base(pio_sm_config *c, uint8_t out_base);   /* sim extension */
void sm_config_set_out_pin_count(pio_sm_config *c, uint8_t out_count); /* sim extension */
void sm_config_set_set_pins(pio_sm_config *c, uint8_t set_base, uint8_t set_count);
void sm_config_set_set_pin_base(pio_sm_config *c, uint8_t set_base);   /* sim extension */
void sm_config_set_set_pin_count(pio_sm_config *c, uint8_t set_count); /* sim extension */
void sm_config_set_in_pins(pio_sm_config *c, uint8_t in_base);
void sm_config_set_in_pin_base(pio_sm_config *c, uint8_t in_base); /* sim extension */
#if PIO_SIM_HAS_IN_PIN_COUNT
/** RP2350: IN PINS / MOV x,PINS / WAIT PIN see only this many low pins; higher
 * bits read 0. `count` 1..32 (32 = unmasked default). */
void sm_config_set_in_pin_count(pio_sm_config *c, uint8_t in_count);
#endif
void sm_config_set_sideset_pins(pio_sm_config *c, uint8_t sideset_base);
/** `bit_count` is the SDK convention: data bits + the enable bit when
 * `optional` (so `.side_set 2 opt` passes 3). */
void sm_config_set_sideset(pio_sm_config *c, uint8_t bit_count, bool optional, bool pindirs);
/** Inclusive wrap: `wrap_target` is the bottom, `wrap` the top. */
void sm_config_set_wrap(pio_sm_config *c, uint8_t wrap_target, uint8_t wrap);
/** Clock divider 16.8 fixed point; `div_int` 0 encodes 65536. */
void sm_config_set_clkdiv_int_frac(pio_sm_config *c, uint16_t div_int, uint8_t div_frac);
/** Float convenience (SDK sm_config_set_clkdiv). */
void sm_config_set_clkdiv(pio_sm_config *c, float div);
/** `shift_right` matches the SDK bool; threshold 0 (or >32) means 32. */
void sm_config_set_out_shift(pio_sm_config *c, bool shift_right, bool autopull,
                             uint8_t pull_threshold);
void sm_config_set_in_shift(pio_sm_config *c, bool shift_right, bool autopush,
                            uint8_t push_threshold);
void sm_config_set_fifo_join(pio_sm_config *c, pio_fifo_join_t join);
void sm_config_set_mov_status(pio_sm_config *c, pio_status_sel_t status_sel, uint8_t status_n);
void sm_config_set_jmp_pin(pio_sm_config *c, uint8_t pin);
/** EXECCTRL output specials (SDK sm_config_set_out_special): `sticky`
 * re-asserts driven pins every cycle; `has_enable_pin` uses OUT-data bit
 * `enable_bit_index` as an inline output-enable. */
void sm_config_set_out_special(pio_sm_config *c, bool sticky, bool has_enable_pin,
                               uint8_t enable_bit_index);

/** Apply `c` to `sm`, reset it, and set its PC to `initial_pc` (SDK
 * pio_sm_init). Clears the FIFOs but preserves the sticky FDEBUG flags, which
 * are cleared only via pio_sim_sm_clear_fdebug (as on hardware). */
void pio_sim_sm_init(pio_sim_t *pio, uint8_t sm, uint8_t initial_pc, const pio_sm_config *c);
/** Apply `c` to `sm` without resetting its execution state (SDK
 * pio_sm_set_config). Like the hardware, the FIFOs are cleared only when the
 * FIFO-join configuration actually changes; an unchanged join preserves any
 * queued words. (pio_sim_sm_init clears them unconditionally.) */
void pio_sim_sm_set_config(pio_sim_t *pio, uint8_t sm, const pio_sm_config *c);

/** Register the external device callback (NULL to clear). */
void pio_sim_set_device(pio_sim_t *pio, void (*on_tick)(pio_sim_t *, void *), void *ctx);

/** Per-instruction trace callback type. `pc` is the fetch address of the
 * committed word (parked PC for injected instructions); `insn` the executed
 * word. The SM's post-execute registers are readable via `pio->sm[sm]`. */
typedef void (*pio_sim_trace_fn)(const pio_sim_t *pio, uint8_t sm, uint8_t pc, uint16_t insn,
                                 void *ctx);

/** Install (or clear, with NULL) the per-instruction trace hook. It fires once
 * per committed instruction on any SM of this block — stall retries and delay
 * cycles do not fire it. */
void pio_sim_set_trace(pio_sim_t *pio, pio_sim_trace_fn fn, void *ctx);

#if PIO_SIM_HAS_IRQ_PREVNEXT
/** RP2350: link the neighbouring PIO blocks addressed by `irq/wait ... prev|next`.
 * Either pointer may be NULL to leave that direction unlinked (the default, in
 * which case prev/next set & clear have no effect and a prev/next wait never
 * completes). Pass `pio` itself to alias a direction back to this block. */
void pio_sim_set_irq_neighbors(pio_sim_t *pio, pio_sim_t *prev, pio_sim_t *next);
#endif

#if PIO_SIM_HAS_GPIO_BASE
/** RP2350: set the GPIOBASE window offset (0 or 16) for this PIO block. All pins
 * the state machines access are shifted by this amount, so a 32-pin window can
 * map to GPIO 0-31 (base 0, default) or 16-47 (base 16). Mirrors the SDK's
 * pio_set_gpio_base. Values other than 0 or 16 are clamped to those. */
void pio_sim_set_gpio_base(pio_sim_t *pio, uint8_t base);

/** Read back the GPIOBASE window offset (0 or 16). Mirrors the SDK's
 * pio_get_gpio_base; lets a consumer reproduce the SDK's absolute-to-window pin
 * mapping (e.g. pio_sm_set_pins_with_mask64 shifts its mask down by this). */
uint8_t pio_sim_get_gpio_base(const pio_sim_t *pio);
#endif

/** Fast-forward the two-cycle input synchroniser to the current pad levels, as
 * if the present inputs had been held stable for two clocks. Useful in tests
 * that set a static input level before running, so the held value is visible
 * without spending two settle ticks. Only inputs that change afterwards incur
 * the two-cycle delay. */
void pio_sim_sync_settle(pio_sim_t *pio);

/** Configure a pull (pull-up/down or open-drain default) on the pins in `mask`:
 * those pins read `level` whenever no block is driving them as outputs. Pins not
 * in `mask` keep plain level behaviour. Operates on this block's pad set (and any
 * blocks sharing it). */
void pio_sim_set_pull_level(pio_sim_t *pio, uint64_t mask, bool level);

/** Set the per-pin INPUT_SYNC_BYPASS mask: pins set here skip the two-cycle input
 * synchroniser, so a state machine samples their live level immediately. */
void pio_sim_set_input_sync_bypass(pio_sim_t *pio, uint64_t mask);

/* ── FIFO access (host side) — SDK pio_sm_* names ──────────────────────────── */

/** True if `sm`'s TX FIFO is full (mirrors pio_sm_is_tx_fifo_full). */
bool pio_sim_sm_is_tx_fifo_full(const pio_sim_t *pio, uint8_t sm);
/** True if `sm`'s TX FIFO is empty (mirrors pio_sm_is_tx_fifo_empty). */
bool pio_sim_sm_is_tx_fifo_empty(const pio_sim_t *pio, uint8_t sm);
/** True if `sm`'s RX FIFO is full (mirrors pio_sm_is_rx_fifo_full). */
bool pio_sim_sm_is_rx_fifo_full(const pio_sim_t *pio, uint8_t sm);
/** True if `sm`'s RX FIFO is empty (mirrors pio_sm_is_rx_fifo_empty). */
bool pio_sim_sm_is_rx_fifo_empty(const pio_sim_t *pio, uint8_t sm);

/** Put a word into the TX FIFO (SDK pio_sm_put). Sim extension: returns false
 * instead of blocking when full, and — like the hardware — sets the TXOVER
 * FDEBUG flag on a full-FIFO write (observable via pio_sim_sm_get_fdebug). */
bool pio_sim_sm_put(pio_sim_t *pio, uint8_t sm, uint32_t word);

/** Get a word from the RX FIFO (SDK pio_sm_get). On success writes the word
 * through `word` and returns true. Sim extension: returns false instead of
 * blocking when empty (leaving `word` untouched), and sets the RXUNDER FDEBUG
 * flag on an empty-FIFO read, matching the hardware. */
bool pio_sim_sm_get(pio_sim_t *pio, uint8_t sm, uint32_t *word);

/** Empty both FIFOs of `sm` (mirrors pio_sm_clear_fifos); keeps the join config. */
void pio_sim_sm_clear_fifos(pio_sim_t *pio, uint8_t sm);

#if PIO_SIM_HAS_RXFIFO_MOV
/* RP2350 direct-mapped RX register file (FJOIN_RX_PUT / RX_GET): the four RX
 * entries are addressed by index (0..3), not popped as a FIFO. In PUT mode the SM
 * writes via `mov rxfifo[], isr` and the host reads with pio_sim_sm_rxfifo_get; in
 * GET mode the host writes with pio_sim_sm_rxfifo_put and the SM reads via
 * `mov osr, rxfifo[]`. These — not pio_sim_sm_get/put — are the host access
 * path for those modes. `index` is masked to the 4-entry file (index & 3). */
uint32_t pio_sim_sm_rxfifo_get(const pio_sim_t *pio, uint8_t sm, uint8_t index);
void pio_sim_sm_rxfifo_put(pio_sim_t *pio, uint8_t sm, uint8_t index, uint32_t word);
#endif

/* Current FIFO occupancy (SDK pio_sm_get_*_fifo_level), 0..cap. */
uint8_t pio_sim_sm_get_tx_fifo_level(const pio_sim_t *pio, uint8_t sm);
uint8_t pio_sim_sm_get_rx_fifo_level(const pio_sim_t *pio, uint8_t sm);

/* Sticky FIFO-debug flags (FDEBUG), set on the event and cleared by the host. */
#define PIO_FDEBUG_TXSTALL 0x1U /* SM stalled pulling from an empty TX FIFO    */
#define PIO_FDEBUG_TXOVER 0x2U  /* host pushed to a full TX FIFO (lost)        */
#define PIO_FDEBUG_RXUNDER 0x4U /* host popped from an empty RX FIFO           */
#define PIO_FDEBUG_RXSTALL 0x8U /* SM stalled pushing to a full RX FIFO        */

/** Read the packed FDEBUG flags for `sm`. */
uint8_t pio_sim_sm_get_fdebug(const pio_sim_t *pio, uint8_t sm);
/** Clear the FDEBUG flags selected by `mask` for `sm`. */
void pio_sim_sm_clear_fdebug(pio_sim_t *pio, uint8_t sm, uint8_t mask);

/* ── Pin access (external device / test harness) ───────────────────────────── */

/** Resolved line level of `pin`: the PIO-driven value where the PIO drives it,
 * else the external (set_pin) level, else the pull level, else 0 (floating). */
bool pio_sim_get_pin(const pio_sim_t *pio, uint8_t pin);
/** Externally drive `pin` to `level` (host/device). The PIO wins on pins it also
 * drives as outputs; otherwise this level is what the pin reads. */
void pio_sim_set_pin(pio_sim_t *pio, uint8_t pin, bool level);
/** Stop externally driving `pin` (undo set_pin): the pin then reads its pull level,
 * or floats (reads 0) if no driver and no pull. */
void pio_sim_release_pin(pio_sim_t *pio, uint8_t pin);
/** True where the PIO is driving `pin` as an output (resolved across state machines). */
bool pio_sim_pin_is_pio_output(const pio_sim_t *pio, uint8_t pin);

/* ── IRQ access ────────────────────────────────────────────────────────────── */

/** Read SM IRQ flag `irq` (masked to the 8 flags: irq & 7). */
bool pio_sim_irq_get(const pio_sim_t *pio, uint8_t irq);
/** Clear SM IRQ flag `irq` (masked: irq & 7) — the host-side acknowledge. */
void pio_sim_irq_clear(pio_sim_t *pio, uint8_t irq);

/* ── System interrupt lines (IRQ0 / IRQ1) ──────────────────────────────────────
 * The PIO raises two system interrupt lines from a set of sources (the INTR
 * register): per-SM RX-FIFO-not-empty, per-SM TX-FIFO-not-full, and the SM IRQ
 * flags — the low four on RP2040, all eight on RP2350. Each line gates the
 * sources with its own enable (INTE) and OR-in force (INTF) mask. */
#define PIO_INTR_SM_RXNEMPTY(sm) ((uint32_t)1U << (sm))       /* bits 0..3  */
#define PIO_INTR_SM_TXNFULL(sm) ((uint32_t)1U << (4U + (sm))) /* bits 4..7  */
#define PIO_INTR_SM_IRQ(i) ((uint32_t)1U << (8U + (i)))       /* bits 8..11 (v0) / 8..15 (v1) */

/** Raw interrupt source word (INTR), independent of the enable/force masks. */
uint32_t pio_sim_get_intr(const pio_sim_t *pio);
/** Enable/disable the sources in `source_mask` on system line `line` (INTE),
 * toggling with `enabled` — matching the SDK's pio_set_irqn_source_mask_enabled
 * rather than replacing the whole register. */
void pio_sim_set_irqn_source_mask_enabled(pio_sim_t *pio, uint8_t line, uint32_t source_mask,
                                          bool enabled);
/** Enable/disable a single source bit on `line` (SDK pio_set_irqn_source_enabled). */
void pio_sim_set_irqn_source_enabled(pio_sim_t *pio, uint8_t line, uint32_t source, bool enabled);
/** Read back the enable mask (INTE) for system line `line` (0 or 1). */
uint32_t pio_sim_get_inte(const pio_sim_t *pio, uint8_t line);
/** Force the sources in `mask` on line `line` (INTF), toggling with `on`. Sim
 * extension: the SDK exposes no PIO IRQ-force helper. */
void pio_sim_set_intf(pio_sim_t *pio, uint8_t line, uint32_t mask, bool on);
/** Read back the force mask (INTF) for system line `line` (0 or 1). */
uint32_t pio_sim_get_intf(const pio_sim_t *pio, uint8_t line);
/** Masked interrupt status (INTS) for `line`: (INTR & INTE) | INTF. */
uint32_t pio_sim_get_ints(const pio_sim_t *pio, uint8_t line);
/** Whether system interrupt line `line` is currently asserted (INTS != 0). */
bool pio_sim_get_irqn_asserted(const pio_sim_t *pio, uint8_t line);

/* ── Stepping ──────────────────────────────────────────────────────────────── */

/** Advance the PIO by one system clock. */
void pio_sim_tick(pio_sim_t *pio);

/** Advance by `n` system clocks. */
void pio_sim_run(pio_sim_t *pio, uint64_t n);

/**
 * Run until the RX FIFO of `sm` is non-empty or `max_ticks` elapse.
 * Returns the number of ticks actually run; the caller checks rx_empty().
 */
uint64_t pio_sim_run_until_rx(pio_sim_t *pio, uint8_t sm, uint64_t max_ticks);

/**
 * Run until the TX FIFO of `sm` is empty or `max_ticks` elapse. Returns the
 * number of ticks run. FIFO-level only: the SM may still be shifting out a
 * word it already pulled into the OSR — use pio_sim_run_until_tx_drained to
 * wait for that word as well.
 */
uint64_t pio_sim_run_until_tx_empty(pio_sim_t *pio, uint8_t sm, uint64_t max_ticks);

/**
 * Run until state machine `sm` stalls on an empty TX FIFO or `max_ticks`
 * elapse — the pico-sdk idiom for "all data actually shifted out": it clears
 * the sticky FDEBUG TXSTALL flag, ticks until it sets again (TX empty *and*
 * the OSR's last word consumed), and returns the number of ticks run. The
 * program must keep OUTing/PULLing for the stall to occur, as streaming
 * programs do; otherwise the call runs the full max_ticks.
 */
uint64_t pio_sim_run_until_tx_drained(pio_sim_t *pio, uint8_t sm, uint64_t max_ticks);

/* ── Multi-PIO group ───────────────────────────────────────────────────────────
 * A device has more than one PIO block (RP2040: 2, RP2350: 3 — PIO_SIM_NUM_PIO).
 * A group ties several blocks to one system clock so they advance in lockstep,
 * and (on RP2350) wires their inter-PIO IRQ links into a ring so block i's
 * `next` is block i+1 and its `prev` is block i-1 (wrapping), matching the
 * PIO0→PIO1→PIO2→PIO0 ordering. By default each block keeps its own pad model;
 * pio_sim_group_init_shared points all blocks at one shared pad set so they
 * drive and sample the same GPIOs (or bridge pins with a device hook). */
typedef struct {
    pio_sim_t *blk[PIO_SIM_NUM_PIO];
    uint8_t count;
    bool shared;     /* blocks share `pads` (pio_sim_group_init_shared) */
    pio_pads_t pads; /* the shared pad set, when `shared` */
} pio_sim_group_t;

/** Initialise a group from `count` (1..PIO_SIM_NUM_PIO) already-initialised
 * blocks. On RP2350 this also wires the prev/next IRQ ring across them. */
void pio_sim_group_init(pio_sim_group_t *g, pio_sim_t *const *blocks, uint8_t count);

/** Like pio_sim_group_init, but also points every block at one shared pad set
 * (owned by the group) so the blocks drive and sample the same GPIOs. The shared
 * synchroniser is clocked once per group tick. */
void pio_sim_group_init_shared(pio_sim_group_t *g, pio_sim_t *const *blocks, uint8_t count);

/** Advance every block in the group by one system clock (lockstep). */
void pio_sim_group_tick(pio_sim_group_t *g);

/** Advance the whole group by `n` system clocks. */
void pio_sim_group_run(pio_sim_group_t *g, uint64_t n);

/** Enable state machines across every block in the group in one synchronised step:
 * `masks[i]` selects the SMs to enable on block `i`, and their clock dividers are
 * phase-aligned (as pio_sim_enable_sm_mask_in_sync does per block). Because the group
 * ticks all blocks in lockstep, SMs started this way run cycle-aligned across PIO
 * blocks — the multi-PIO analogue of an in-block synchronised start. */
void pio_sim_group_enable_sm_mask_in_sync(pio_sim_group_t *g, const uint8_t *masks);

/* ── DMA data requests ─────────────────────────────────────────────────────────
 * The PIO-side DREQ levels. The full DMA controller model (channels, chaining,
 * IRQs, sniffer, pacing timers) lives in pio_dma.h. */

/** TX DREQ: true when the SM's TX FIFO can accept another word. */
bool pio_sim_sm_is_dreq_tx(const pio_sim_t *pio, uint8_t sm);
/** RX DREQ: true when the SM's RX FIFO holds a word to read. */
bool pio_sim_sm_is_dreq_rx(const pio_sim_t *pio, uint8_t sm);

/* ── Instruction encoding helpers (match pico-sdk pio_encode_*) ────────────── */

uint16_t pio_sim_encode_jmp(uint8_t condition, uint8_t addr);
uint16_t pio_sim_encode_set(uint8_t dest, uint8_t value);
uint16_t pio_sim_encode_out(uint8_t dest, uint8_t count);
uint16_t pio_sim_encode_in(uint8_t src, uint8_t count);
uint16_t pio_sim_encode_push(bool if_full, bool block);
uint16_t pio_sim_encode_pull(bool if_empty, bool block);
uint16_t pio_sim_encode_mov(uint8_t dest, uint8_t op, uint8_t src);
#if PIO_SIM_HAS_RXFIFO_MOV
/* RP2350 indexed RX-FIFO moves. `_y` forms take the index from scratch register
 * Y (low 2 bits); the others use the literal `index` (0..3). */
uint16_t pio_sim_encode_mov_to_rxfifo(uint8_t index);   /* rxfifo[index] <- ISR */
uint16_t pio_sim_encode_mov_from_rxfifo(uint8_t index); /* OSR <- rxfifo[index] */
uint16_t pio_sim_encode_mov_to_rxfifo_y(void);          /* rxfifo[y] <- ISR     */
uint16_t pio_sim_encode_mov_from_rxfifo_y(void);        /* OSR <- rxfifo[y]     */
#endif
uint16_t pio_sim_encode_irq(bool clear, bool wait, uint8_t index);
/** IRQ with the index made SM-relative (the `rel` form): the low two bits of
 * `index` have the SM number added (mod 4) at execution time. */
uint16_t pio_sim_encode_irq_rel(bool clear, bool wait, uint8_t index);
uint16_t pio_sim_encode_wait(uint8_t polarity, uint8_t source, uint8_t index);
/** WAIT on an SM-relative IRQ index (`wait <pol> irq <index> rel`). */
uint16_t pio_sim_encode_wait_irq_rel(uint8_t polarity, uint8_t index);
#if PIO_SIM_HAS_WAIT_JMPPIN
/** RP2350: WAIT on the JMP pin (PINCTRL jmp_pin) plus `index` (`wait <pol>
 * jmppin` or `wait <pol> jmppin + <index>`). */
uint16_t pio_sim_encode_wait_jmppin(uint8_t polarity, uint8_t index);
#endif
uint16_t pio_sim_encode_nop(void);

/** Execute one instruction immediately on `sm` (like pio_sm_exec / an
 * SMx_INSTR write). Side-set applies, but the instruction's delay field is
 * ignored — as on silicon, where forced instructions skip their delay cycles
 * (RP2040 datasheet §3.4.5.2). Instructions injected via OUT/MOV EXEC *do*
 * execute their delay. PC only changes if the instruction writes it; a
 * stalling instruction latches and is retried on the SM's own cycles. */
void pio_sim_sm_exec(pio_sim_t *pio, uint8_t sm, uint16_t insn);

/* Side-set / delay field positions and operand encodings. */
#define PIO_OP_JMP 0U
#define PIO_OP_WAIT 1U
#define PIO_OP_IN 2U
#define PIO_OP_OUT 3U
#define PIO_OP_PUSHPULL 4U
#define PIO_OP_MOV 5U
#define PIO_OP_IRQ 6U
#define PIO_OP_SET 7U

/* JMP conditions */
#define PIO_COND_ALWAYS 0U
#define PIO_COND_NOTX 1U    /* !X      */
#define PIO_COND_XDEC 2U    /* X--     */
#define PIO_COND_NOTY 3U    /* !Y      */
#define PIO_COND_YDEC 4U    /* Y--     */
#define PIO_COND_XNEY 5U    /* X != Y  */
#define PIO_COND_PIN 6U     /* branch on input pin */
#define PIO_COND_NOTOSRE 7U /* !OSRE (output shift count < threshold) */

/* IN sources / OUT-MOV-SET destinations (subset shared encodings) */
#define PIO_SRC_PINS 0U
#define PIO_SRC_X 1U
#define PIO_SRC_Y 2U
#define PIO_SRC_NULL 3U
#define PIO_SRC_STATUS 5U
#define PIO_SRC_ISR 6U
#define PIO_SRC_OSR 7U

/* Generic destination aliases (OUT/SET numbering); kept for compatibility.
 * OUT and MOV number some destinations differently — use the per-instruction
 * families below where the instruction matters. */
#define PIO_DST_PINS 0U
#define PIO_DST_X 1U
#define PIO_DST_Y 2U
#define PIO_DST_NULL 3U
#define PIO_DST_PINDIRS 4U
#define PIO_DST_EXEC 4U /* MOV numbering — same value as PIO_DST_PINDIRS */
#define PIO_DST_PC 5U
#define PIO_DST_ISR 6U
#define PIO_DST_OSR 7U

/* OUT instruction destinations. */
#define PIO_OUT_DST_PINS 0U
#define PIO_OUT_DST_X 1U
#define PIO_OUT_DST_Y 2U
#define PIO_OUT_DST_NULL 3U
#define PIO_OUT_DST_PINDIRS 4U
#define PIO_OUT_DST_PC 5U
#define PIO_OUT_DST_ISR 6U
#define PIO_OUT_DST_EXEC 7U

/* MOV instruction destinations (PINDIRS is RP2350-only; EXEC differs from OUT). */
#define PIO_MOV_DST_PINS 0U
#define PIO_MOV_DST_X 1U
#define PIO_MOV_DST_Y 2U
#define PIO_MOV_DST_PINDIRS 3U
#define PIO_MOV_DST_EXEC 4U
#define PIO_MOV_DST_PC 5U
#define PIO_MOV_DST_ISR 6U
#define PIO_MOV_DST_OSR 7U

#if PIO_SIM_HAS_RXFIFO_MOV
/* Indexed RX-FIFO MOV operand bits (RP2350; shares the PUSH/PULL opcode).
 * Plain PUSH/PULL encode operand bit 4 as 0, so it discriminates the two. */
#define PIO_MOV_RXFIFO_BIT 0x10U /* 1 = indexed RX-FIFO MOV, 0 = PUSH/PULL     */
#define PIO_MOV_RXFIFO_GET 0x80U /* 1 = OSR <- rxfifo[idx], 0 = rxfifo[idx] <- ISR */
#define PIO_MOV_RXFIFO_IDX 0x08U /* 1 = literal index in bits [1:0], 0 = Y     */
#endif

/* WAIT sources */
#define PIO_WAIT_GPIO 0U
#define PIO_WAIT_PIN 1U
#define PIO_WAIT_IRQ 2U
#define PIO_WAIT_JMPPIN 3U /* RP2350: wait on jmp_pin + index */

/* IRQ / WAIT-IRQ index field, bits [4:3] select the addressing mode; bits [2:0]
 * are the IRQ flag index.
 *   0x00 = this PIO    0x10 = SM-relative    0x08 = previous PIO    0x18 = next PIO
 * prev/next address a neighbouring PIO block, linked via pio_sim_set_irq_neighbors
 * or pio_sim_group_init; an unlinked direction has no effect. */
#define PIO_IRQ_MODE_MASK 0x18U
#define PIO_IRQ_REL 0x10U
#define PIO_IRQ_PREV 0x08U
#define PIO_IRQ_NEXT 0x18U

/* MOV operations */
#define PIO_MOV_NONE 0U
#define PIO_MOV_INVERT 1U
#define PIO_MOV_REVERSE 2U

#endif /* PIO_SIM_H */
