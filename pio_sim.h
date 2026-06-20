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
 */

#ifndef PIO_SIM_H
#define PIO_SIM_H

#include <stdbool.h>
#include <stdint.h>

#define PIO_SIM_NUM_SM 4U
#define PIO_SIM_INSN_COUNT 32U
#define PIO_SIM_FIFO_DEPTH 4U /* per-SM depth; doubles to 8 when joined */
#define PIO_SIM_FIFO_MAX 8U
#define PIO_SIM_NUM_PINS 32U
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
    uint8_t osr_count; /* bits shifted out (0 = full, 32 = empty)  */
    uint8_t isr_count; /* bits shifted in  (0 = empty, 32 = full)  */

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
    uint8_t sideset_base;
    uint8_t jmp_pin;

    /* Side-set: total_bits includes the enable bit when opt is set */
    uint8_t sideset_total_bits;
    bool sideset_opt;
    bool sideset_pindirs;

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

    pio_fifo_t tx;
    pio_fifo_t rx;
} pio_sm_t;

/* ── PIO block ─────────────────────────────────────────────────────────────── */

typedef struct pio_sim {
    uint16_t insn[PIO_SIM_INSN_COUNT];
    bool insn_used[PIO_SIM_INSN_COUNT];

    pio_sm_t sm[PIO_SIM_NUM_SM];

    uint8_t irq; /* 8 IRQ flags packed into the low byte */

    /* Pad model: shared across all SMs and the external device. */
    uint32_t pin_levels; /* current line level per GPIO  */
    uint32_t pin_dirs;   /* 1 = driven as output by the PIO */

    /* Input synchroniser (always on, to match silicon): GPIO inputs reach the
     * PIO logic through two flip-flops, so the level a state machine samples
     * (IN PINS, JMP PIN, WAIT GPIO/PIN) lags the live pad by two system clocks.
     * in_sync[1] is what the PIO sees; in_sync[0] is the intermediate stage.
     * Pad writes (host or device) and reads of PIO *outputs* are unaffected.
     * Use pio_sim_sync_settle to fast-forward the pipeline to a held input. */
    uint32_t in_sync[2];

    /* External device hook, invoked once per system tick *after* the SMs run.
     * It may read pin_levels / pin_dirs and drive pins the PIO has released. */
    void (*on_tick)(struct pio_sim *pio, void *ctx);
    void *device_ctx;

    uint64_t cycle; /* system clocks elapsed */

    /* Diagnostic: counts instruction fetches from addresses never written by
     * pio_sim_load (an unwritten word decodes as `jmp 0`). A non-zero value
     * after a run usually means a runaway PC / mis-set wrap. Behaviour is
     * otherwise unchanged. */
    uint64_t unwritten_fetches;
} pio_sim_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────────── */

/** Reset the whole block: clears instruction memory, all SMs, pins, and IRQs. */
void pio_sim_init(pio_sim_t *pio);

/** Load `count` instruction words at `offset` into shared instruction memory. */
void pio_sim_load(pio_sim_t *pio, uint8_t offset, const uint16_t *insns, uint8_t count);

/** Reset a single state machine's execution state (PC, registers, FIFOs). */
void pio_sim_sm_restart(pio_sim_t *pio, uint8_t sm);

/** Enable or disable a state machine. */
void pio_sim_sm_set_enabled(pio_sim_t *pio, uint8_t sm, bool enabled);

/* ── Configuration (mirrors the pico-sdk sm_config_set_* surface) ──────────── */

void pio_sim_sm_set_wrap(pio_sim_t *pio, uint8_t sm, uint8_t bottom, uint8_t top);
void pio_sim_sm_set_sideset(pio_sim_t *pio, uint8_t sm, uint8_t bit_count, bool opt, bool pindirs);
void pio_sim_sm_set_sideset_base(pio_sim_t *pio, uint8_t sm, uint8_t base);
void pio_sim_sm_set_out_pins(pio_sim_t *pio, uint8_t sm, uint8_t base, uint8_t count);
void pio_sim_sm_set_set_pins(pio_sim_t *pio, uint8_t sm, uint8_t base, uint8_t count);
void pio_sim_sm_set_in_base(pio_sim_t *pio, uint8_t sm, uint8_t base);
void pio_sim_sm_set_jmp_pin(pio_sim_t *pio, uint8_t sm, uint8_t pin);
void pio_sim_sm_set_out_shift(pio_sim_t *pio, uint8_t sm, pio_shift_dir_t dir, bool autopull,
                              uint8_t threshold);
void pio_sim_sm_set_in_shift(pio_sim_t *pio, uint8_t sm, pio_shift_dir_t dir, bool autopush,
                             uint8_t threshold);
void pio_sim_sm_set_clkdiv(pio_sim_t *pio, uint8_t sm, uint16_t div_int, uint8_t div_frac);

/**
 * Reset the fractional clock-divider accumulator of every SM whose bit is set
 * in `sm_mask`, re-aligning their divided clocks in phase (mirrors the SDK's
 * pio_clkdiv_restart_sm_mask). SMs enabled on the same tick with equal dividers
 * run in lockstep; use this to re-align SMs that were started at different
 * ticks (e.g. the etm_4bit_lo/hi pair or the cross-SM JTAG programs).
 */
void pio_sim_clkdiv_restart(pio_sim_t *pio, uint8_t sm_mask);
void pio_sim_sm_set_pc(pio_sim_t *pio, uint8_t sm, uint8_t pc);

/* MOV STATUS source selector (mirrors EXECCTRL_STATUS_SEL). */
typedef enum {
    PIO_STATUS_TX_LEVEL = 0, /* all-ones while TX FIFO level < N            */
    PIO_STATUS_RX_LEVEL = 1, /* all-ones while RX FIFO level < N            */
    PIO_STATUS_IRQ_SET = 2,  /* RP2350: all-ones while IRQ flag N is set    */
} pio_status_sel_t;

/** Configure the MOV STATUS source: comparison against a FIFO level, or (on
 * RP2350) an IRQ flag. Clears any fixed-value override. */
void pio_sim_sm_set_status_sel(pio_sim_t *pio, uint8_t sm, pio_status_sel_t sel, uint8_t n);

/** Pin MOV STATUS to a fixed value (test override), bypassing the FIFO/IRQ
 * derivation. */
void pio_sim_sm_set_status_value(pio_sim_t *pio, uint8_t sm, uint32_t value);

/* FIFO join: 0 = none (4+4), 1 = join TX (8 TX, 0 RX), 2 = join RX (8 RX, 0 TX).
 * RP2350 adds two random-access RX modes (4-entry register file addressed by
 * the MOV-RX-FIFO instructions): PUT (SM writes, system reads) and GET (system
 * writes, SM reads). */
typedef enum {
    PIO_FIFO_JOIN_NONE = 0,
    PIO_FIFO_JOIN_TX = 1,
    PIO_FIFO_JOIN_RX = 2,
    PIO_FIFO_JOIN_RX_PUT = 3, /* RP2350: SM writes RX via `mov rxfifo[], isr`  */
    PIO_FIFO_JOIN_RX_GET = 4, /* RP2350: SM reads RX via `mov osr, rxfifo[]`   */
} pio_fifo_join_t;

void pio_sim_sm_set_fifo_join(pio_sim_t *pio, uint8_t sm, pio_fifo_join_t join);

/** Set the PIO output-enable (pindir) for a span of pins, as pindir config does. */
void pio_sim_sm_set_pindirs(pio_sim_t *pio, uint8_t sm, uint8_t base, uint8_t count, bool out);

/** Register the external device callback (NULL to clear). */
void pio_sim_set_device(pio_sim_t *pio, void (*on_tick)(pio_sim_t *, void *), void *ctx);

/** Fast-forward the two-cycle input synchroniser to the current pad levels, as
 * if the present inputs had been held stable for two clocks. Useful in tests
 * that set a static input level before running, so the held value is visible
 * without spending two settle ticks. Only inputs that change afterwards incur
 * the two-cycle delay. */
void pio_sim_sync_settle(pio_sim_t *pio);

/* ── FIFO access (host side) ───────────────────────────────────────────────── */

bool pio_sim_tx_full(const pio_sim_t *pio, uint8_t sm);
bool pio_sim_tx_empty(const pio_sim_t *pio, uint8_t sm);
bool pio_sim_rx_full(const pio_sim_t *pio, uint8_t sm);
bool pio_sim_rx_empty(const pio_sim_t *pio, uint8_t sm);

/** Push a word into the TX FIFO. Returns false if full. */
bool pio_sim_tx_push(pio_sim_t *pio, uint8_t sm, uint32_t word);

/** Pop a word from the RX FIFO. Returns false if empty. */
bool pio_sim_rx_pop(pio_sim_t *pio, uint8_t sm, uint32_t *word);

/* ── Pin access (external device / test harness) ───────────────────────────── */

bool pio_sim_get_pin(const pio_sim_t *pio, uint8_t pin);
void pio_sim_set_pin(pio_sim_t *pio, uint8_t pin, bool level);
bool pio_sim_pin_is_pio_output(const pio_sim_t *pio, uint8_t pin);

/* ── IRQ access ────────────────────────────────────────────────────────────── */

bool pio_sim_irq_get(const pio_sim_t *pio, uint8_t irq);
void pio_sim_irq_clear(pio_sim_t *pio, uint8_t irq);

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
 * Run until the TX FIFO of `sm` has drained (empty) or `max_ticks` elapse.
 * Returns the number of ticks run.
 */
uint64_t pio_sim_run_until_tx_empty(pio_sim_t *pio, uint8_t sm, uint64_t max_ticks);

/* ── Instruction encoding helpers (match pico-sdk pio_encode_*) ────────────── */

uint16_t pio_sim_encode_jmp(uint8_t condition, uint8_t addr);
uint16_t pio_sim_encode_set(uint8_t dest, uint8_t value);
uint16_t pio_sim_encode_out(uint8_t dest, uint8_t count);
uint16_t pio_sim_encode_in(uint8_t src, uint8_t count);
uint16_t pio_sim_encode_push(bool if_full, bool block);
uint16_t pio_sim_encode_pull(bool if_empty, bool block);
uint16_t pio_sim_encode_mov(uint8_t dest, uint8_t op, uint8_t src);
/* RP2350 indexed RX-FIFO moves. `_y` forms take the index from scratch register
 * Y (low 2 bits); the others use the literal `index` (0..3). */
uint16_t pio_sim_encode_mov_to_rxfifo(uint8_t index);   /* rxfifo[index] <- ISR */
uint16_t pio_sim_encode_mov_from_rxfifo(uint8_t index); /* OSR <- rxfifo[index] */
uint16_t pio_sim_encode_mov_to_rxfifo_y(void);          /* rxfifo[y] <- ISR     */
uint16_t pio_sim_encode_mov_from_rxfifo_y(void);        /* OSR <- rxfifo[y]     */
uint16_t pio_sim_encode_irq(bool clear, bool wait, uint8_t index);
/** IRQ with the index made SM-relative (the `rel` form): the low two bits of
 * `index` have the SM number added (mod 4) at execution time. */
uint16_t pio_sim_encode_irq_rel(bool clear, bool wait, uint8_t index);
uint16_t pio_sim_encode_wait(uint8_t polarity, uint8_t source, uint8_t index);
/** WAIT on an SM-relative IRQ index (`wait <pol> irq <index> rel`). */
uint16_t pio_sim_encode_wait_irq_rel(uint8_t polarity, uint8_t index);
uint16_t pio_sim_encode_nop(void);

/** Execute one instruction immediately on `sm` (like pio_sm_exec). */
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

#define PIO_DST_PINS 0U
#define PIO_DST_X 1U
#define PIO_DST_Y 2U
#define PIO_DST_NULL 3U
#define PIO_DST_PINDIRS 4U
#define PIO_DST_EXEC 4U /* MOV dest EXEC == 4; OUT dest EXEC == 7 (see encode) */
#define PIO_DST_PC 5U
#define PIO_DST_ISR 6U
#define PIO_DST_OSR 7U

/* WAIT sources */
#define PIO_WAIT_GPIO 0U
#define PIO_WAIT_PIN 1U
#define PIO_WAIT_IRQ 2U

/* IRQ / WAIT-IRQ index field, bits [4:3] select the addressing mode; bits [2:0]
 * are the IRQ flag index.
 *   0x00 = this PIO    0x10 = SM-relative    0x08 = previous PIO    0x18 = next PIO
 * prev/next address a neighbouring PIO block, which the single-block simulator
 * does not model (see pio_sim.c). */
#define PIO_IRQ_MODE_MASK 0x18U
#define PIO_IRQ_REL 0x10U
#define PIO_IRQ_PREV 0x08U
#define PIO_IRQ_NEXT 0x18U

/* MOV operations */
#define PIO_MOV_NONE 0U
#define PIO_MOV_INVERT 1U
#define PIO_MOV_REVERSE 2U

#endif /* PIO_SIM_H */
