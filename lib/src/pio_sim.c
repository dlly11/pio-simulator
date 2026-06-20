/*
 * SPDX-License-Identifier: MIT
 * pio_sim — RP2040/RP2350 PIO functional simulator. See pio_sim.h.
 */

#include "pio_sim.h"

#include <string.h>

/* ── FIFO ──────────────────────────────────────────────────────────────────── */

/* Drop the contents but keep the configured capacity (used on SM restart, which
 * must not silently undo a FIFO-join configuration). */
static void fifo_clear(pio_fifo_t *f)
{
    f->head = 0;
    f->tail = 0;
    f->count = 0;
}

static bool fifo_push(pio_fifo_t *f, uint32_t v)
{
    if (f->count >= f->cap) {
        return false;
    }
    f->buf[f->head] = v;
    f->head = (uint8_t)((f->head + 1U) % PIO_SIM_FIFO_MAX);
    f->count++;
    return true;
}

static bool fifo_pop(pio_fifo_t *f, uint32_t *v)
{
    if (f->count == 0U) {
        return false;
    }
    *v = f->buf[f->tail];
    f->tail = (uint8_t)((f->tail + 1U) % PIO_SIM_FIFO_MAX);
    f->count--;
    return true;
}

/* ── Bit helpers ───────────────────────────────────────────────────────────── */

static uint32_t mask_n(uint8_t n) { return (n >= 32U) ? 0xFFFFFFFFU : (((uint32_t)1U << n) - 1U); }

static uint32_t reverse32(uint32_t v_in)
{
    uint32_t v = v_in;
    v = ((v & 0x55555555U) << 1U) | ((v >> 1U) & 0x55555555U);
    v = ((v & 0x33333333U) << 2U) | ((v >> 2U) & 0x33333333U);
    v = ((v & 0x0F0F0F0FU) << 4U) | ((v >> 4U) & 0x0F0F0F0FU);
    v = ((v & 0x00FF00FFU) << 8U) | ((v >> 8U) & 0x00FF00FFU);
    v = (v << 16U) | (v >> 16U);
    return v;
}

/* ── Lifecycle ─────────────────────────────────────────────────────────────── */

static void sm_reset_exec(pio_sm_t *sm)
{
    sm->pc = sm->wrap_bottom;
    sm->x = 0;
    sm->y = 0;
    sm->osr = 0;
    sm->isr = 0;
    sm->osr_count = 32; /* empty */
    sm->isr_count = 0;  /* empty */
    sm->delay = 0;
    sm->stalled = false;
    sm->exec_pending = false;
    sm->exec_insn = 0;
    sm->clk_accum = 0;
    /* Preserve the configured FIFO-join capacity across a restart. */
    fifo_clear(&sm->tx);
    fifo_clear(&sm->rx);
}

void pio_sim_init(pio_sim_t *pio)
{
    (void)memset(pio, 0, sizeof(*pio));
    for (uint8_t i = 0; i < PIO_SIM_NUM_SM; i++) {
        pio_sm_t *sm = &pio->sm[i];
        sm->enabled = false;
        sm->wrap_bottom = 0;
        sm->wrap_top = PIO_SIM_INSN_COUNT - 1U;
        sm->out_dir = PIO_SHIFT_LEFT;
        sm->in_dir = PIO_SHIFT_LEFT;
        sm->pull_thresh = 32;
        sm->push_thresh = 32;
        sm->clkdiv_int = 1;
        sm->clkdiv_frac = 0;
        sm->sideset_total_bits = 0;
        sm->jmp_pin = 0;
        sm->tx.cap = PIO_SIM_FIFO_DEPTH;
        sm->rx.cap = PIO_SIM_FIFO_DEPTH;
        sm_reset_exec(sm);
    }
}

void pio_sim_load(pio_sim_t *pio, uint8_t offset, const uint16_t *insns, uint8_t count)
{
    for (uint8_t i = 0; (i < count) && ((offset + i) < PIO_SIM_INSN_COUNT); i++) {
        pio->insn[offset + i] = insns[i];
        pio->insn_used[offset + i] = true;
    }
}

void pio_sim_sm_restart(pio_sim_t *pio, uint8_t sm) { sm_reset_exec(&pio->sm[sm]); }

void pio_sim_sm_set_enabled(pio_sim_t *pio, uint8_t sm, bool enabled)
{
    pio->sm[sm].enabled = enabled;
}

/* ── Configuration ─────────────────────────────────────────────────────────── */

void pio_sim_sm_set_wrap(pio_sim_t *pio, uint8_t sm, uint8_t bottom, uint8_t top)
{
    pio->sm[sm].wrap_bottom = bottom;
    pio->sm[sm].wrap_top = top;
}

void pio_sim_sm_set_sideset(pio_sim_t *pio, uint8_t sm, uint8_t bit_count, bool opt, bool pindirs)
{
    /* bit_count is the pico-sdk "bit_count" = data bits + (opt ? 1 : 0). */
    pio->sm[sm].sideset_total_bits = bit_count;
    pio->sm[sm].sideset_opt = opt;
    pio->sm[sm].sideset_pindirs = pindirs;
}

void pio_sim_sm_set_sideset_base(pio_sim_t *pio, uint8_t sm, uint8_t base)
{
    pio->sm[sm].sideset_base = base;
}

void pio_sim_sm_set_out_pins(pio_sim_t *pio, uint8_t sm, uint8_t base, uint8_t count)
{
    pio->sm[sm].out_base = base;
    pio->sm[sm].out_count = count;
}

void pio_sim_sm_set_set_pins(pio_sim_t *pio, uint8_t sm, uint8_t base, uint8_t count)
{
    pio->sm[sm].set_base = base;
    pio->sm[sm].set_count = count;
}

void pio_sim_sm_set_in_base(pio_sim_t *pio, uint8_t sm, uint8_t base)
{
    pio->sm[sm].in_base = base;
}

void pio_sim_sm_set_jmp_pin(pio_sim_t *pio, uint8_t sm, uint8_t pin) { pio->sm[sm].jmp_pin = pin; }

void pio_sim_sm_set_out_shift(pio_sim_t *pio, uint8_t sm, pio_shift_dir_t dir, bool autopull,
                              uint8_t threshold)
{
    pio->sm[sm].out_dir = dir;
    pio->sm[sm].autopull = autopull;
    pio->sm[sm].pull_thresh = ((threshold == 0U) || (threshold > 32U)) ? 32U : threshold;
}

void pio_sim_sm_set_in_shift(pio_sim_t *pio, uint8_t sm, pio_shift_dir_t dir, bool autopush,
                             uint8_t threshold)
{
    pio->sm[sm].in_dir = dir;
    pio->sm[sm].autopush = autopush;
    pio->sm[sm].push_thresh = ((threshold == 0U) || (threshold > 32U)) ? 32U : threshold;
}

void pio_sim_sm_set_clkdiv(pio_sim_t *pio, uint8_t sm, uint16_t div_int, uint8_t div_frac)
{
    /* div_int == 0 means a divider of 65536 in hardware. */
    pio->sm[sm].clkdiv_int = div_int;
    pio->sm[sm].clkdiv_frac = div_frac;
}

void pio_sim_clkdiv_restart(pio_sim_t *pio, uint8_t sm_mask)
{
    for (uint8_t i = 0; i < PIO_SIM_NUM_SM; i++) {
        if ((sm_mask & (uint8_t)(1U << i)) != 0U) {
            pio->sm[i].clk_accum = 0;
        }
    }
}

void pio_sim_sm_set_pc(pio_sim_t *pio, uint8_t sm, uint8_t pc) { pio->sm[sm].pc = pc; }

void pio_sim_sm_set_status_sel(pio_sim_t *pio, uint8_t sm, pio_status_sel_t sel, uint8_t n)
{
    pio->sm[sm].status_sel = (uint8_t)sel;
    pio->sm[sm].status_n = n;
    pio->sm[sm].status_override = false;
}

void pio_sim_sm_set_status_value(pio_sim_t *pio, uint8_t sm, uint32_t value)
{
    pio->sm[sm].status_value = value;
    pio->sm[sm].status_override = true;
}

void pio_sim_sm_set_fifo_join(pio_sim_t *pio, uint8_t sm, pio_fifo_join_t join)
{
    pio_sm_t *s = &pio->sm[sm];
    switch (join) {
    case PIO_FIFO_JOIN_TX:
        s->tx.cap = PIO_SIM_FIFO_MAX;
        s->rx.cap = 0;
        break;
    case PIO_FIFO_JOIN_RX:
        s->tx.cap = 0;
        s->rx.cap = PIO_SIM_FIFO_MAX;
        break;
    case PIO_FIFO_JOIN_RX_PUT:
    case PIO_FIFO_JOIN_RX_GET:
        /* RP2350 random-access RX: a 4-entry register file addressed by the
         * MOV-RX-FIFO instructions; the TX FIFO is disabled. */
        s->tx.cap = 0;
        s->rx.cap = PIO_SIM_FIFO_DEPTH;
        break;
    case PIO_FIFO_JOIN_NONE:
    default:
        s->tx.cap = PIO_SIM_FIFO_DEPTH;
        s->rx.cap = PIO_SIM_FIFO_DEPTH;
        break;
    }
    s->tx.head = 0;
    s->tx.tail = 0;
    s->tx.count = 0;
    s->rx.head = 0;
    s->rx.tail = 0;
    s->rx.count = 0;
}

void pio_sim_sm_set_pindirs(pio_sim_t *pio, uint8_t sm, uint8_t base, uint8_t count, bool out)
{
    (void)sm;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t pin = (uint8_t)((base + i) % PIO_SIM_NUM_PINS);
        if (out) {
            pio->pin_dirs |= (1U << pin);
        } else {
            pio->pin_dirs &= ~(1U << pin);
        }
    }
}

void pio_sim_set_device(pio_sim_t *pio, void (*on_tick)(pio_sim_t *, void *), void *ctx)
{
    pio->on_tick = on_tick;
    pio->device_ctx = ctx;
}

void pio_sim_sync_settle(pio_sim_t *pio)
{
    pio->in_sync[0] = pio->pin_levels;
    pio->in_sync[1] = pio->pin_levels;
}

/* ── FIFO access ───────────────────────────────────────────────────────────── */

bool pio_sim_tx_full(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[sm].tx.count >= pio->sm[sm].tx.cap;
}
bool pio_sim_tx_empty(const pio_sim_t *pio, uint8_t sm) { return pio->sm[sm].tx.count == 0U; }
bool pio_sim_rx_full(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[sm].rx.count >= pio->sm[sm].rx.cap;
}
bool pio_sim_rx_empty(const pio_sim_t *pio, uint8_t sm) { return pio->sm[sm].rx.count == 0U; }

bool pio_sim_tx_push(pio_sim_t *pio, uint8_t sm, uint32_t word)
{
    return fifo_push(&pio->sm[sm].tx, word);
}

bool pio_sim_rx_pop(pio_sim_t *pio, uint8_t sm, uint32_t *word)
{
    return fifo_pop(&pio->sm[sm].rx, word);
}

/* ── Pin access ────────────────────────────────────────────────────────────── */

bool pio_sim_get_pin(const pio_sim_t *pio, uint8_t pin)
{
    return (pio->pin_levels & (1U << (pin % PIO_SIM_NUM_PINS))) != 0U;
}

void pio_sim_set_pin(pio_sim_t *pio, uint8_t pin, bool level)
{
    uint32_t bit = (uint32_t)1U << ((uint32_t)pin % PIO_SIM_NUM_PINS);
    if (level) {
        pio->pin_levels |= bit;
    } else {
        pio->pin_levels &= ~bit;
    }
}

bool pio_sim_pin_is_pio_output(const pio_sim_t *pio, uint8_t pin)
{
    return (pio->pin_dirs & (1U << (pin % PIO_SIM_NUM_PINS))) != 0U;
}

/* Level a state machine samples for an input pin: the second synchroniser
 * stage, two system clocks behind the live pad. */
static bool pio_input_level(const pio_sim_t *pio, uint8_t pin)
{
    return (pio->in_sync[1] & (1U << (pin % PIO_SIM_NUM_PINS))) != 0U;
}

static void drive_pins(pio_sim_t *pio, uint8_t base, uint8_t count, uint32_t value)
{
    for (uint8_t i = 0; i < count; i++) {
        uint8_t pin = (uint8_t)((base + i) % PIO_SIM_NUM_PINS);
        pio_sim_set_pin(pio, pin, ((value >> i) & 1U) != 0U);
    }
}

static void drive_pindirs(pio_sim_t *pio, uint8_t base, uint8_t count, uint32_t value)
{
    for (uint8_t i = 0; i < count; i++) {
        uint8_t pin = (uint8_t)((base + i) % PIO_SIM_NUM_PINS);
        uint32_t bit = (uint32_t)1U << pin;
        if (((value >> i) & 1U) != 0U) {
            pio->pin_dirs |= bit;
        } else {
            pio->pin_dirs &= ~bit;
        }
    }
}

static uint32_t read_in_pins(const pio_sim_t *pio, uint8_t base, uint8_t count)
{
    uint32_t v = 0;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t pin = (uint8_t)((base + i) % PIO_SIM_NUM_PINS);
        if (pio_input_level(pio, pin)) {
            v |= ((uint32_t)1U << i);
        }
    }
    return v;
}

/* ── IRQ access ────────────────────────────────────────────────────────────── */

bool pio_sim_irq_get(const pio_sim_t *pio, uint8_t irq)
{
    return (pio->irq & (1U << (irq & 7U))) != 0U;
}

void pio_sim_irq_clear(pio_sim_t *pio, uint8_t irq)
{
    uint32_t mask = ~((uint32_t)1U << (irq & 7U));
    pio->irq = (uint8_t)(pio->irq & mask);
}

/* ── Shift-register operations ─────────────────────────────────────────────── */

/* Pull a word from TX into OSR. Returns false if the FIFO was empty. */
static bool osr_refill(pio_sm_t *sm)
{
    uint32_t v;
    if (!fifo_pop(&sm->tx, &v)) {
        return false;
    }
    sm->osr = v;
    sm->osr_count = 0;
    return true;
}

/* Push ISR into RX. Returns false if the FIFO was full. */
static bool isr_drain(pio_sm_t *sm)
{
    if (!fifo_push(&sm->rx, sm->isr)) {
        return false;
    }
    sm->isr = 0;
    sm->isr_count = 0;
    return true;
}

/* ── Instruction execution ─────────────────────────────────────────────────── */

/* Decoded delay / side-set field of an instruction. */
typedef struct {
    uint8_t delay;        /* post-instruction idle cycles    */
    uint32_t side_val;    /* side-set value                  */
    bool side_en;         /* whether side-set applies         */
    uint8_t ss_data_bits; /* number of side-set pins driven */
} delay_sideset_t;

/* Side-set width is at most 5 bits; clamp so all shifts below are provably
 * in range for the static analyzer and for malformed configurations. */
static uint8_t sideset_total(const pio_sm_t *sm)
{
    return (sm->sideset_total_bits > 5U) ? 5U : sm->sideset_total_bits;
}

static delay_sideset_t decode_delay_sideset(const pio_sm_t *sm, uint8_t field)
{
    delay_sideset_t d = {0, 0, !sm->sideset_opt, 0};
    uint8_t ss_total = sideset_total(sm); /* clamped to ≤ 5 */
    /* delay_bits ∈ [0,5]; mask with 7 so the analyzer can prove the shifts
     * below stay within range. */
    uint8_t delay_bits = (uint8_t)((5U - ss_total) & 7U);
    d.delay = (uint8_t)(field & mask_n(delay_bits));
    if (ss_total > 0U) {
        if (sm->sideset_opt) {
            d.side_en = ((field >> 4U) & 1U) != 0U;
            d.ss_data_bits = (uint8_t)(ss_total - 1U);
        } else {
            d.ss_data_bits = ss_total;
        }
        d.side_val = ((uint32_t)field >> delay_bits) & mask_n(d.ss_data_bits);
    }
    return d;
}

/* Compute the MOV STATUS source: a fixed override, or all-ones/all-zeros from a
 * FIFO level comparison or (RP2350) an IRQ flag, per status_sel/status_n. */
static uint32_t read_status(const pio_sim_t *pio, const pio_sm_t *sm)
{
    if (sm->status_override) {
        return sm->status_value;
    }
    bool cond;
    switch (sm->status_sel) {
    case PIO_STATUS_RX_LEVEL:
        cond = (sm->rx.count < sm->status_n);
        break;
    case PIO_STATUS_IRQ_SET:
        cond = pio_sim_irq_get(pio, (uint8_t)(sm->status_n & 7U));
        break;
    case PIO_STATUS_TX_LEVEL:
    default:
        cond = (sm->tx.count < sm->status_n);
        break;
    }
    return cond ? 0xFFFFFFFFU : 0U;
}

/* Read a register / pin source used by IN and MOV (shared source encoding). */
static uint32_t read_source(const pio_sim_t *pio, const pio_sm_t *sm, uint8_t src)
{
    switch (src) {
    case PIO_SRC_PINS:
        return read_in_pins(pio, sm->in_base, 32);
    case PIO_SRC_X:
        return sm->x;
    case PIO_SRC_Y:
        return sm->y;
    case PIO_SRC_STATUS:
        return read_status(pio, sm);
    case PIO_SRC_ISR:
        return sm->isr;
    case PIO_SRC_OSR:
        return sm->osr;
    case PIO_SRC_NULL:
    default:
        return 0;
    }
}

/* ── Per-instruction handlers ──────────────────────────────────────────────── */
/* Each returns true if the instruction stalled (must be retried). */

static bool exec_jmp(const pio_sim_t *pio, pio_sm_t *sm, uint8_t operand, uint8_t *next_pc,
                     bool *set_pc)
{
    uint8_t cond = (uint8_t)((operand >> 5U) & 7U);
    uint8_t addr = (uint8_t)(operand & 0x1FU);
    bool take = false;
    switch (cond) {
    case PIO_COND_ALWAYS:
        take = true;
        break;
    case PIO_COND_NOTX:
        take = (sm->x == 0U);
        break;
    case PIO_COND_XDEC:
        take = (sm->x != 0U);
        sm->x--; /* post-decrement, always */
        break;
    case PIO_COND_NOTY:
        take = (sm->y == 0U);
        break;
    case PIO_COND_YDEC:
        take = (sm->y != 0U);
        sm->y--;
        break;
    case PIO_COND_XNEY:
        take = (sm->x != sm->y);
        break;
    case PIO_COND_PIN:
        take = pio_input_level(pio, sm->jmp_pin);
        break;
    case PIO_COND_NOTOSRE:
        take = (sm->osr_count < sm->pull_thresh);
        break;
    default:
        break;
    }
    if (take) {
        *set_pc = true;
        *next_pc = addr;
    }
    return false;
}

/* Effective IRQ flag for an IRQ / WAIT-IRQ index field (bits [4:0]). `local` is
 * false when the field addresses a neighbouring PIO block (prev/next), which
 * this single-block model does not implement. When SM-relative, the SM number
 * is added to the low two bits of the index (mod 4); bit 2 is unaffected. */
typedef struct {
    uint8_t index;
    bool local;
} irq_target_t;

static irq_target_t resolve_irq_target(uint8_t sm_idx, uint8_t field)
{
    uint8_t mode = (uint8_t)(field & PIO_IRQ_MODE_MASK);
    uint8_t base = (uint8_t)(field & 0x7U);
    irq_target_t t = {base, true};
    switch (mode) {
    case PIO_IRQ_REL:
        t.index = (uint8_t)((base & 0x4U) | (((base & 0x3U) + sm_idx) & 0x3U));
        break;
    case PIO_IRQ_PREV:
    case PIO_IRQ_NEXT:
        t.local = false; /* a neighbouring PIO block — not modelled here */
        break;
    default: /* this PIO, absolute index */
        break;
    }
    return t;
}

static bool exec_wait(pio_sim_t *pio, uint8_t sm_idx, uint8_t operand)
{
    const pio_sm_t *sm = &pio->sm[sm_idx];
    uint8_t pol = (uint8_t)((operand >> 7U) & 1U);
    uint8_t source = (uint8_t)((operand >> 5U) & 3U);
    uint8_t index = (uint8_t)(operand & 0x1FU);
    bool met = false;
    if (source == PIO_WAIT_GPIO) {
        met = (pio_input_level(pio, index) ? 1U : 0U) == pol;
    } else if (source == PIO_WAIT_PIN) {
        uint8_t pin = (uint8_t)((sm->in_base + index) % PIO_SIM_NUM_PINS);
        met = (pio_input_level(pio, pin) ? 1U : 0U) == pol;
    } else if (source == PIO_WAIT_IRQ) {
        irq_target_t t = resolve_irq_target(sm_idx, index);
        if (!t.local) {
            met = false; /* neighbouring-PIO IRQ not modelled: never satisfied */
        } else {
            met = (pio_sim_irq_get(pio, t.index) ? 1U : 0U) == pol;
            if (met && (pol == 1U)) {
                pio_sim_irq_clear(pio, t.index); /* WAIT 1 IRQ clears the flag */
            }
        }
    } else {
        /* Unknown wait source: leave met unchanged. */
    }
    return !met;
}

static bool exec_in(pio_sim_t *pio, uint8_t sm_idx, uint8_t operand)
{
    pio_sm_t *sm = &pio->sm[sm_idx];
    uint8_t src = (uint8_t)((operand >> 5U) & 7U);
    uint8_t count = (uint8_t)(operand & 0x1FU);
    if (count == 0U) {
        count = 32;
    }
    /* If this IN crosses the autopush threshold but the RX FIFO is full, stall
     * *before* shifting. Otherwise the instruction would be retried next cycle
     * and shift the same source bits into the ISR a second time. */
    bool will_autopush =
        sm->autopush && (((uint32_t)sm->isr_count + (uint32_t)count) >= sm->push_thresh);
    if (will_autopush && pio_sim_rx_full(pio, sm_idx)) {
        return true;
    }
    uint32_t data = read_source(pio, sm, src) & mask_n(count);
    if (sm->in_dir == PIO_SHIFT_RIGHT) {
        sm->isr = (count >= 32U) ? data : ((sm->isr >> count) | (data << (32U - (uint32_t)count)));
    } else {
        sm->isr = (count >= 32U) ? data : ((sm->isr << count) | data);
    }
    sm->isr_count = (uint8_t)(sm->isr_count + count);
    if (sm->isr_count > 32U) {
        sm->isr_count = 32;
    }
    if (sm->autopush && (sm->isr_count >= sm->push_thresh)) {
        (void)isr_drain(sm); /* RX has space (checked above): push cannot fail */
    }
    return false;
}

static void out_store(pio_sim_t *pio, pio_sm_t *sm, uint8_t dest, uint32_t val, uint8_t count,
                      uint8_t *next_pc, bool *set_pc)
{
    switch (dest) {
    case PIO_SRC_PINS:
        drive_pins(pio, sm->out_base, sm->out_count, val);
        break;
    case PIO_SRC_X:
        sm->x = val;
        break;
    case PIO_SRC_Y:
        sm->y = val;
        break;
    case PIO_DST_PINDIRS:
        drive_pindirs(pio, sm->out_base, sm->out_count, val);
        break;
    case PIO_DST_PC:
        *set_pc = true;
        *next_pc = (uint8_t)(val & 0x1FU);
        break;
    case PIO_SRC_ISR:
        sm->isr = val;
        sm->isr_count = count;
        break;
    case PIO_SRC_OSR: /* OUT EXEC */
        sm->exec_pending = true;
        sm->exec_insn = (uint16_t)val;
        break;
    case PIO_SRC_NULL:
    default:
        break;
    }
}

static bool exec_out(pio_sim_t *pio, pio_sm_t *sm, uint8_t operand, uint8_t *next_pc, bool *set_pc)
{
    uint8_t dest = (uint8_t)((operand >> 5U) & 7U);
    uint8_t count = (uint8_t)(operand & 0x1FU);
    if (count == 0U) {
        count = 32;
    }
    if (sm->autopull && (sm->osr_count >= sm->pull_thresh)) {
        if (!osr_refill(sm)) {
            return true; /* TX empty: stall */
        }
    }
    uint32_t out_val;
    if (sm->out_dir == PIO_SHIFT_RIGHT) {
        out_val = sm->osr & mask_n(count);
        sm->osr = (count >= 32U) ? 0U : (sm->osr >> count);
    } else {
        out_val = (count >= 32U) ? sm->osr : ((sm->osr >> (32U - count)) & mask_n(count));
        sm->osr = (count >= 32U) ? 0U : (sm->osr << count);
    }
    sm->osr_count = (uint8_t)(sm->osr_count + count);
    if (sm->osr_count > 32U) {
        sm->osr_count = 32;
    }
    out_store(pio, sm, dest, out_val, count, next_pc, set_pc);
    return false;
}

static bool exec_pushpull(pio_sim_t *pio, uint8_t sm_idx, uint8_t operand)
{
    pio_sm_t *sm = &pio->sm[sm_idx];
    bool is_pull = ((operand >> 7U) & 1U) != 0U;
    bool cond = ((operand >> 6U) & 1U) != 0U; /* iffull / ifempty */
    bool block = ((operand >> 5U) & 1U) != 0U;
    if (!is_pull) {
        if (cond && (sm->isr_count < sm->push_thresh)) {
            return false; /* iffull but not full enough → nop */
        }
        if (pio_sim_rx_full(pio, sm_idx)) {
            if (block) {
                return true;
            }
            /* non-blocking push on full FIFO: data lost, ISR cleared */
            sm->isr = 0;
            sm->isr_count = 0;
        } else {
            (void)isr_drain(sm);
        }
        return false;
    }
    if (cond && (sm->osr_count < sm->pull_thresh)) {
        return false; /* ifempty but not empty enough → nop */
    }
    if (pio_sim_tx_empty(pio, sm_idx)) {
        if (block) {
            return true;
        }
        /* non-blocking pull on empty FIFO: OSR <- X */
        sm->osr = sm->x;
        sm->osr_count = 0;
    } else {
        (void)osr_refill(sm);
    }
    return false;
}

/* RP2350 indexed RX-FIFO MOV (shares the PUSH/PULL opcode; selected when the
 * low five operand bits are non-zero with bit 4 set). bit 7 chooses direction
 * (0: rxfifo[idx] <- ISR, 1: OSR <- rxfifo[idx]); bit 3 selects the index
 * source (1: literal in bits [1:0], 0: scratch register Y). The RX FIFO is
 * treated as a 4-entry register file; see PIO_FIFO_JOIN_RX_PUT/GET. */
static void exec_mov_rxfifo(pio_sim_t *pio, uint8_t sm_idx, uint8_t operand)
{
    pio_sm_t *sm = &pio->sm[sm_idx];
    bool from_rx = (operand & 0x80U) != 0U;
    bool idx_literal = (operand & 0x08U) != 0U;
    uint8_t idx = idx_literal ? (uint8_t)(operand & 0x3U) : (uint8_t)(sm->y & 0x3U);
    if (from_rx) {
        sm->osr = sm->rx.buf[idx];
        sm->osr_count = 0;
    } else {
        sm->rx.buf[idx] = sm->isr;
        sm->isr = 0;
        sm->isr_count = 0;
    }
}

static void exec_mov(pio_sim_t *pio, pio_sm_t *sm, uint8_t operand, uint8_t *next_pc, bool *set_pc)
{
    uint8_t dest = (uint8_t)((operand >> 5U) & 7U);
    uint8_t mov_op = (uint8_t)((operand >> 3U) & 3U);
    uint8_t src = (uint8_t)(operand & 7U);
    uint32_t v = read_source(pio, sm, src);
    if (mov_op == PIO_MOV_INVERT) {
        v = ~v;
    } else if (mov_op == PIO_MOV_REVERSE) {
        v = reverse32(v);
    } else {
        /* PIO_MOV_NONE: value unchanged. */
    }
    switch (dest) {
    case PIO_DST_PINS:
        drive_pins(pio, sm->out_base, sm->out_count, v);
        break;
    case PIO_DST_X:
        sm->x = v;
        break;
    case PIO_DST_Y:
        sm->y = v;
        break;
    case PIO_DST_EXEC: /* MOV dest 4 == EXEC */
        sm->exec_pending = true;
        sm->exec_insn = (uint16_t)v;
        break;
    case PIO_DST_PC:
        *set_pc = true;
        *next_pc = (uint8_t)(v & 0x1FU);
        break;
    case PIO_DST_ISR:
        sm->isr = v;
        sm->isr_count = 0;
        break;
    case PIO_DST_OSR:
        sm->osr = v;
        sm->osr_count = 0;
        break;
    default:
        break;
    }
}

static bool exec_irq(pio_sim_t *pio, uint8_t sm_idx, uint8_t operand)
{
    const pio_sm_t *sm = &pio->sm[sm_idx];
    bool clr = ((operand >> 6U) & 1U) != 0U;
    bool wait = ((operand >> 5U) & 1U) != 0U;
    irq_target_t t = resolve_irq_target(sm_idx, (uint8_t)(operand & 0x1FU));
    if (!t.local) {
        /* prev/next target a neighbouring PIO block this model lacks: set and
         * clear have no local effect; a wait can never be satisfied here. */
        return wait && !clr;
    }
    uint8_t irq = t.index;
    if (clr) {
        pio_sim_irq_clear(pio, irq);
        return false;
    }
    if (wait) {
        /* IRQ ... wait: raise the flag on the cycle the instruction is first
         * presented, then park here until something clears it. sm->stalled is
         * false on the first execution and true on each retry, so we raise once
         * and merely poll thereafter (re-raising would defeat an external
         * clear). */
        if (!sm->stalled) {
            pio->irq |= (uint8_t)(1U << irq);
        }
        return pio_sim_irq_get(pio, irq);
    }
    pio->irq |= (uint8_t)(1U << irq);
    return false;
}

static void exec_set(pio_sim_t *pio, pio_sm_t *sm, uint8_t operand)
{
    uint8_t dest = (uint8_t)((operand >> 5U) & 7U);
    uint32_t value = ((uint32_t)operand & 0x1FU);
    switch (dest) {
    case PIO_DST_PINS:
        drive_pins(pio, sm->set_base, sm->set_count, value);
        break;
    case PIO_DST_X:
        sm->x = value;
        break;
    case PIO_DST_Y:
        sm->y = value;
        break;
    case PIO_DST_PINDIRS:
        drive_pindirs(pio, sm->set_base, sm->set_count, value);
        break;
    default:
        break;
    }
}

/*
 * Execute one instruction on `sm`. Returns true if it committed, false if it
 * stalled (in which case the caller leaves PC/delay untouched and retries).
 * `next_pc` receives the PC to load on commit (already wrap-resolved unless the
 * instruction set it explicitly, e.g. JMP / OUT PC). `*set_pc` signals that.
 */
static bool exec_one(pio_sim_t *pio, uint8_t sm_idx, uint16_t insn, uint8_t *next_pc, bool *set_pc,
                     uint8_t *delay_out)
{
    pio_sm_t *sm = &pio->sm[sm_idx];
    uint8_t op = (uint8_t)(insn >> 13U);
    uint8_t field = (uint8_t)((insn >> 8U) & 0x1FU);
    uint8_t operand = (uint8_t)(insn & 0xFFU);

    *set_pc = false;
    *next_pc = 0;

    delay_sideset_t ds = decode_delay_sideset(sm, field);

    bool stalled = false;
    switch (op) {
    case PIO_OP_JMP:
        stalled = exec_jmp(pio, sm, operand, next_pc, set_pc);
        break;
    case PIO_OP_WAIT:
        stalled = exec_wait(pio, sm_idx, operand);
        break;
    case PIO_OP_IN:
        stalled = exec_in(pio, sm_idx, operand);
        break;
    case PIO_OP_OUT:
        stalled = exec_out(pio, sm, operand, next_pc, set_pc);
        break;
    case PIO_OP_PUSHPULL:
        /* RP2350 indexed RX-FIFO MOV reuses this opcode (arg2 != 0); plain
         * PUSH/PULL have the low five operand bits clear. */
        if ((operand & 0x1FU) != 0U) {
            exec_mov_rxfifo(pio, sm_idx, operand);
        } else {
            stalled = exec_pushpull(pio, sm_idx, operand);
        }
        break;
    case PIO_OP_MOV:
        exec_mov(pio, sm, operand, next_pc, set_pc);
        break;
    case PIO_OP_IRQ:
        stalled = exec_irq(pio, sm_idx, operand);
        break;
    case PIO_OP_SET:
        exec_set(pio, sm, operand);
        break;
    default:
        break;
    }

    /* Side-set drives its pins every cycle the instruction is presented,
     * including stall cycles — hardware applies side-set regardless of whether
     * the instruction itself can make progress. */
    if (ds.side_en && (ds.ss_data_bits > 0U)) {
        if (sm->sideset_pindirs) {
            drive_pindirs(pio, sm->sideset_base, ds.ss_data_bits, ds.side_val);
        } else {
            drive_pins(pio, sm->sideset_base, ds.ss_data_bits, ds.side_val);
        }
    }

    if (stalled) {
        return false;
    }

    /* Delay applies only once the instruction commits. */
    *delay_out = ds.delay;
    return true;
}

/* Run a single SM cycle (called when the clock divider elapses). */
static void sm_cycle(pio_sim_t *pio, uint8_t sm_idx)
{
    pio_sm_t *sm = &pio->sm[sm_idx];

    if (sm->delay > 0U) {
        sm->delay--;
        return;
    }

    uint16_t insn;
    bool from_exec = sm->exec_pending;
    if (from_exec) {
        insn = sm->exec_insn;
    } else {
        if (!pio->insn_used[sm->pc]) {
            pio->unwritten_fetches++; /* diagnostic: runaway PC / bad wrap */
        }
        insn = pio->insn[sm->pc];
    }

    uint8_t next_pc = 0;
    bool set_pc = false;
    uint8_t delay = 0;
    bool committed = exec_one(pio, sm_idx, insn, &next_pc, &set_pc, &delay);

    if (!committed) {
        sm->stalled = true;
        return; /* retry same instruction next cycle */
    }
    sm->stalled = false;

    /* An injected instruction does not advance PC or wrap on its own (unless it
     * was a jmp/out-pc). It clears the pending flag once executed. */
    if (from_exec) {
        sm->exec_pending = false;
        if (set_pc) {
            sm->pc = next_pc;
        }
        sm->delay = delay;
        return;
    }

    if (set_pc) {
        sm->pc = next_pc;
    } else if (sm->pc == sm->wrap_top) {
        sm->pc = sm->wrap_bottom;
    } else {
        sm->pc = (uint8_t)((sm->pc + 1U) % PIO_SIM_INSN_COUNT);
    }
    sm->delay = delay;
}

/* ── Stepping ──────────────────────────────────────────────────────────────── */

void pio_sim_tick(pio_sim_t *pio)
{
    for (uint8_t i = 0; i < PIO_SIM_NUM_SM; i++) {
        pio_sm_t *sm = &pio->sm[i];
        if (!sm->enabled) {
            continue;
        }
        /* Clock divider: accumulate 1.0 (in 8-bit frac units = 256) each system
         * tick; the SM advances when the accumulator reaches the divider. */
        uint32_t div_units = ((uint32_t)sm->clkdiv_int << 8U) | sm->clkdiv_frac;
        if (div_units == 0U) {
            div_units = (65536U << 8U); /* div_int == 0 → 65536 */
        }
        sm->clk_accum += 256U;
        if (sm->clk_accum >= div_units) {
            sm->clk_accum -= div_units;
            sm_cycle(pio, i);
        }
    }
    if (pio->on_tick != NULL) {
        pio->on_tick(pio, pio->device_ctx);
    }
    /* Clock the two-stage input synchroniser from the (now updated) pads. */
    pio->in_sync[1] = pio->in_sync[0];
    pio->in_sync[0] = pio->pin_levels;
    pio->cycle++;
}

void pio_sim_run(pio_sim_t *pio, uint64_t n)
{
    for (uint64_t i = 0; i < n; i++) {
        pio_sim_tick(pio);
    }
}

uint64_t pio_sim_run_until_rx(pio_sim_t *pio, uint8_t sm, uint64_t max_ticks)
{
    uint64_t t = 0;
    while ((t < max_ticks) && pio_sim_rx_empty(pio, sm)) {
        pio_sim_tick(pio);
        t++;
    }
    return t;
}

uint64_t pio_sim_run_until_tx_empty(pio_sim_t *pio, uint8_t sm, uint64_t max_ticks)
{
    uint64_t t = 0;
    /* Drain when both the FIFO is empty and the SM is not mid-transfer holding
     * a pulled word (best-effort: callers pad with a margin). */
    while ((t < max_ticks) && !pio_sim_tx_empty(pio, sm)) {
        pio_sim_tick(pio);
        t++;
    }
    return t;
}

/* ── sm_exec ───────────────────────────────────────────────────────────────── */

void pio_sim_sm_exec(pio_sim_t *pio, uint8_t sm, uint16_t insn)
{
    /* Execute immediately, out of band, like pio_sm_exec(). Side-set and delay
     * still apply; PC is untouched unless the instruction writes it. */
    uint8_t next_pc = 0;
    bool set_pc = false;
    uint8_t delay = 0;
    bool committed = exec_one(pio, sm, insn, &next_pc, &set_pc, &delay);
    (void)delay; /* a forced instruction executes immediately; its delay field
                  * is ignored (side-set still applies). */
    if (committed) {
        if (set_pc) {
            pio->sm[sm].pc = next_pc;
        }
    } else {
        /* A stalling instruction injected via exec latches as pending so the
         * SM retries it on its next cycle. */
        pio->sm[sm].exec_pending = true;
        pio->sm[sm].exec_insn = insn;
    }
}

/* ── Encoders ──────────────────────────────────────────────────────────────── */

uint16_t pio_sim_encode_jmp(uint8_t condition, uint8_t addr)
{
    return (uint16_t)(((uint32_t)PIO_OP_JMP << 13U) | ((condition & 7U) << 5U) | (addr & 0x1FU));
}

uint16_t pio_sim_encode_set(uint8_t dest, uint8_t value)
{
    return (uint16_t)(((uint32_t)PIO_OP_SET << 13U) | ((dest & 7U) << 5U) | (value & 0x1FU));
}

uint16_t pio_sim_encode_out(uint8_t dest, uint8_t count)
{
    return (uint16_t)(((uint32_t)PIO_OP_OUT << 13U) | ((dest & 7U) << 5U) | (count & 0x1FU));
}

uint16_t pio_sim_encode_in(uint8_t src, uint8_t count)
{
    return (uint16_t)(((uint32_t)PIO_OP_IN << 13U) | ((src & 7U) << 5U) | (count & 0x1FU));
}

uint16_t pio_sim_encode_push(bool if_full, bool block)
{
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | (if_full ? (1U << 6U) : 0U) |
                      (block ? (1U << 5U) : 0U));
}

uint16_t pio_sim_encode_pull(bool if_empty, bool block)
{
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | (1U << 7U) |
                      (if_empty ? (1U << 6U) : 0U) | (block ? (1U << 5U) : 0U));
}

uint16_t pio_sim_encode_mov(uint8_t dest, uint8_t op, uint8_t src)
{
    return (uint16_t)(((uint32_t)PIO_OP_MOV << 13U) | ((dest & 7U) << 5U) | ((op & 3U) << 3U) |
                      (src & 7U));
}

uint16_t pio_sim_encode_irq(bool clear, bool wait, uint8_t index)
{
    return (uint16_t)(((uint32_t)PIO_OP_IRQ << 13U) | (clear ? (1U << 6U) : 0U) |
                      (wait ? (1U << 5U) : 0U) | (index & 0x1FU));
}

uint16_t pio_sim_encode_mov_to_rxfifo(uint8_t index)
{
    /* op=100, bit4 (mov-rx discriminator) + bit3 (literal index) + index[1:0] */
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | 0x10U | 0x08U | (index & 0x3U));
}

uint16_t pio_sim_encode_mov_from_rxfifo(uint8_t index)
{
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | 0x80U | 0x10U | 0x08U | (index & 0x3U));
}

uint16_t pio_sim_encode_mov_to_rxfifo_y(void)
{
    /* bit3 clear → index taken from scratch register Y */
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | 0x10U);
}

uint16_t pio_sim_encode_mov_from_rxfifo_y(void)
{
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | 0x80U | 0x10U);
}

uint16_t pio_sim_encode_irq_rel(bool clear, bool wait, uint8_t index)
{
    return pio_sim_encode_irq(clear, wait, (uint8_t)((index & 0x7U) | PIO_IRQ_REL));
}

uint16_t pio_sim_encode_wait(uint8_t polarity, uint8_t source, uint8_t index)
{
    return (uint16_t)(((uint32_t)PIO_OP_WAIT << 13U) | ((polarity & 1U) << 7U) |
                      ((source & 3U) << 5U) | (index & 0x1FU));
}

uint16_t pio_sim_encode_wait_irq_rel(uint8_t polarity, uint8_t index)
{
    return pio_sim_encode_wait(polarity, PIO_WAIT_IRQ, (uint8_t)((index & 0x7U) | PIO_IRQ_REL));
}

uint16_t pio_sim_encode_nop(void)
{
    /* nop == mov y, y */
    return pio_sim_encode_mov(PIO_DST_Y, PIO_MOV_NONE, PIO_SRC_Y);
}
