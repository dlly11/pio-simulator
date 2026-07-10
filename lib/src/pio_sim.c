/*
 * SPDX-License-Identifier: MIT
 * pio_sim — RP2040/RP2350 PIO functional simulator. See pio_sim.h.
 */

#include "pio_sim.h"

#include "pio_sim_internal.h"

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

/* Public-API state-machine index guard: out-of-range `sm` is masked into 0..3
 * rather than indexing out of bounds — the same defensive convention as the
 * `irq & 7U` and `line & 1U` masks elsewhere in the API. */
#define SM_IDX(sm) ((sm) & (PIO_SIM_NUM_SM - 1U))

/* ── Bit helpers ───────────────────────────────────────────────────────────── */

static uint32_t mask_n(uint8_t n) { return (n >= 32U) ? 0xFFFFFFFFU : (((uint32_t)1U << n) - 1U); }

/* Map a state-machine pin "view" index to a physical GPIO. The PIO pin mux is a
 * 32-pin window (indices wrap mod 32); on RP2350 GPIOBASE then offsets the whole
 * window so it can reach GPIO 16-47. Used by every SM-side pin access. */
static uint8_t phys_pin(const pio_sim_t *pio, uint8_t view)
{
    uint8_t w = (uint8_t)(view % PIO_SIM_PIN_WINDOW);
#if PIO_SIM_HAS_GPIO_BASE
    return (uint8_t)((pio->gpio_base + w) % PIO_SIM_NUM_PINS);
#else
    (void)pio;
    return w;
#endif
}

/* Chip-side drive after the function mux and IO_BANK0 overrides, before the
 * pad stage: PIO drive (already FUNCSEL-gated in resolve_pads) plus the
 * selected peripheral's output on non-PIO pins, with OUTOVER/OEOVER applied. */
void pio_pads_chip_drive(const pio_pads_t *p, uint64_t *oe_out, uint64_t *lvl_out)
{
    uint64_t pio_oe = p->pin_dirs;
    uint64_t per_oe = p->periph_oe & p->periph_sel_mask & ~pio_oe;
    uint64_t oe = pio_oe | per_oe;
    uint64_t lvl = (p->pin_levels & pio_oe) | (p->periph_level & per_oe);
    lvl = ((lvl ^ p->outover_inv) | p->outover_high) & ~p->outover_low;
    oe = ((oe ^ p->oeover_inv) | p->oeover_high) & ~p->oeover_low;
    /* Pad output-disable overrides everything the chip asks for. */
    oe &= ~p->pad_od;
#if PIO_SIM_HAS_PAD_ISO
    /* Isolation freezes the pad at the drive latched when ISO was raised. */
    oe = (oe & ~p->pad_iso) | (p->iso_oe & p->pad_iso);
    lvl = (lvl & ~p->pad_iso) | (p->iso_levels & p->pad_iso);
#endif
    *oe_out = oe;
    *lvl_out = lvl;
}

/* Resolved live pad level, by driver priority: the chip (PIO through the mux,
 * or the selected peripheral) wins on pins it drives; else an external
 * (off-chip) driver; else the pulls — up reads 1, down reads 0, both enabled
 * is the bus keeper holding the last driven level; else the pin floats and
 * reads 0 (true high-impedance). */
static uint64_t pads_live(const pio_pads_t *p)
{
    uint64_t oe;
    uint64_t lvl;
    pio_pads_chip_drive(p, &oe, &lvl);
    uint64_t ext = p->ext_drive & ~oe;
    uint64_t undriven = ~oe & ~ext;
    uint64_t keeper = p->pad_pue & p->pad_pde & undriven;
    uint64_t pull_up = p->pad_pue & ~p->pad_pde & undriven;
    return (lvl & oe) | (p->ext_levels & ext) | pull_up | (p->keep_state & keeper);
}

/* The input view the PIO logic sees, before the synchroniser: the wire level
 * gated by the pad's input enable (IE=0 reads 0; RP2350 isolation also gates
 * the input) and transformed by INOVER. INOVER is an IO_BANK0 override applied
 * upstream of the PIO's own two-stage synchroniser (matching silicon), which
 * is why it is folded in here — into what feeds in_sync[0] — rather than at the
 * SM read. */
static uint64_t pads_input_view(const pio_pads_t *p)
{
    uint64_t v = pads_live(p) & p->pad_ie;
#if PIO_SIM_HAS_PAD_ISO
    v &= ~p->pad_iso;
#endif
    return ((v ^ p->inover_inv) | p->inover_high) & ~p->inover_low;
}

/* Bus keeper: latch the level of every driven pad so pue&pde can hold it once
 * the driver lets go. Clocked once per system tick, with the synchroniser. */
static void pads_update_keeper(pio_pads_t *p)
{
    uint64_t oe;
    uint64_t lvl;
    pio_pads_chip_drive(p, &oe, &lvl);
    uint64_t driven = oe | p->ext_drive;
    uint64_t level = (lvl & oe) | (p->ext_levels & p->ext_drive & ~oe);
    p->keep_state = (p->keep_state & ~driven) | (level & driven);
}

/* Reset a pad set to the simulator's legacy-friendly defaults: every pin's
 * input enabled, no pulls, and FUNCSEL at the sim-only LEGACY_ANY_PIO value so
 * every owning block can drive every pin (the pre-mux behaviour). Datasheet
 * reset values are opt-in via pio_sim_pads_reset_hw (pio_gpio.h). */
void pio_pads_init_defaults(pio_pads_t *p)
{
    p->pad_ie = ~(uint64_t)0;
    for (uint8_t i = 0; i < PIO_SIM_NUM_PINS; i++) {
        p->funcsel[i] = 0xFFU; /* PIO_GPIO_FUNC_LEGACY_ANY_PIO */
    }
    for (uint8_t o = 0; o < PIO_SIM_NUM_PIO; o++) {
        p->pio_func_mask[o] = ~(uint64_t)0;
    }
    p->periph_sel_mask = 0;
}

/* Recompute the pad's PIO drive (pin_dirs / pin_levels) from the per-SM output
 * registers of every block that owns the pad. Hardware collates the pin *writes*
 * of each cycle: when multiple state machines write the same GPIO on the same
 * cycle, the highest-numbered one wins (RP2040 §3.5.6.1 / RP2350 PIO GPIO output
 * priority); a pin nobody writes this cycle holds its latched level, so on a
 * later cycle any SM's write lands regardless of number. OUT_STICKY makes an SM
 * re-assert its driven pins every cycle (see pio_sim_tick), which is what gives
 * it continuous priority. Called after every pin/pindir write and once per tick. */
static void resolve_pads(pio_pads_t *p)
{
    uint64_t oe = 0;
    uint64_t val = p->pin_levels; /* unwritten pins keep their latched level */
    for (uint8_t o = 0; o < p->owner_count; o++) {
        const struct pio_sim *b = p->owners[o];
        /* Function mux: owner slot o only reaches pins whose FUNCSEL selects
         * FUNC_PIO<o> (or the legacy any-PIO default). */
        uint64_t fmask = p->pio_func_mask[o];
        for (uint8_t s = 0; s < PIO_SIM_NUM_SM; s++) {
            /* Later iterations overwrite earlier ones, so on a same-cycle
             * conflict the highest-numbered SM (of the latest owner) wins. */
            uint64_t written = b->sm[s].wrote_this_cycle & fmask;
            val = (val & ~written) | (b->sm[s].out_pin_val & written);
            oe |= b->sm[s].out_pin_oe & fmask;
        }
    }
    p->pin_dirs = oe;
    p->pin_levels = val;
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
    pio->pads = &pio->pads_embedded;    /* own pads until joined to a shared group */
    pio->pads_embedded.owners[0] = pio; /* this block drives its own pads */
    pio->pads_embedded.owner_count = 1;
    pio_pads_init_defaults(&pio->pads_embedded);
    for (uint8_t i = 0; i < PIO_SIM_NUM_SM; i++) {
        pio_sm_t *sm = &pio->sm[i];
        sm->enabled = false;
        sm->wrap_bottom = 0;
        sm->wrap_top = PIO_SIM_INSN_COUNT - 1U;
        sm->out_dir = PIO_SHIFT_LEFT;
        sm->in_dir = PIO_SHIFT_LEFT;
        sm->pull_thresh = 32;
        sm->push_thresh = 32;
        sm->in_count = 32; /* unmasked IN pin group until set (matches RP2040) */
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

void pio_sim_sm_restart(pio_sim_t *pio, uint8_t sm) { sm_reset_exec(&pio->sm[SM_IDX(sm)]); }

void pio_sim_sm_set_enabled(pio_sim_t *pio, uint8_t sm, bool enabled)
{
    pio->sm[SM_IDX(sm)].enabled = enabled;
}

bool pio_sim_sm_is_enabled(const pio_sim_t *pio, uint8_t sm) { return pio->sm[SM_IDX(sm)].enabled; }

bool pio_sim_sm_is_stalled(const pio_sim_t *pio, uint8_t sm) { return pio->sm[SM_IDX(sm)].stalled; }

void pio_sim_set_sm_mask_enabled(pio_sim_t *pio, uint8_t sm_mask, bool enabled)
{
    for (uint8_t i = 0; i < PIO_SIM_NUM_SM; i++) {
        if ((sm_mask & (1U << i)) != 0U) {
            pio->sm[i].enabled = enabled;
        }
    }
}

void pio_sim_enable_sm_mask_in_sync(pio_sim_t *pio, uint8_t sm_mask)
{
    pio_sim_set_sm_mask_enabled(pio, sm_mask, true);
    pio_sim_clkdiv_restart_sm_mask(pio, sm_mask); /* phase-align the started SMs */
}

/* ── Configuration ─────────────────────────────────────────────────────────── */

/* Pin-span counts are at most 32 (the SM pin window): clamp so the pin-drive
 * loops never shift a 32-bit value by ≥ 32. */
static uint8_t clamp_pin_count(uint8_t count) { return (count > 32U) ? 32U : count; }

static uint8_t clamp_thresh(uint8_t threshold)
{
    return ((threshold == 0U) || (threshold > 32U)) ? 32U : threshold;
}

pio_sm_config pio_get_default_sm_config(void)
{
    pio_sm_config c;
    (void)memset(&c, 0, sizeof(c));
    c.wrap_bottom = 0;
    c.wrap_top = PIO_SIM_INSN_COUNT - 1U;
    c.out_dir = PIO_SHIFT_LEFT;
    c.in_dir = PIO_SHIFT_LEFT;
    c.pull_thresh = 32;
    c.push_thresh = 32;
    c.in_count = 32;  /* unmasked */
    c.clkdiv_int = 1; /* divide by 1 (a 0 here would encode 65536) */
    c.status_sel = (uint8_t)PIO_STATUS_TX_LEVEL;
    return c;
}

void sm_config_set_out_pins(pio_sm_config *c, uint8_t out_base, uint8_t out_count)
{
    c->out_base = out_base;
    c->out_count = clamp_pin_count(out_count);
}
void sm_config_set_out_pin_base(pio_sm_config *c, uint8_t out_base) { c->out_base = out_base; }
void sm_config_set_out_pin_count(pio_sm_config *c, uint8_t out_count)
{
    c->out_count = clamp_pin_count(out_count);
}
void sm_config_set_set_pins(pio_sm_config *c, uint8_t set_base, uint8_t set_count)
{
    c->set_base = set_base;
    c->set_count = clamp_pin_count(set_count);
}
void sm_config_set_set_pin_base(pio_sm_config *c, uint8_t set_base) { c->set_base = set_base; }
void sm_config_set_set_pin_count(pio_sm_config *c, uint8_t set_count)
{
    c->set_count = clamp_pin_count(set_count);
}
void sm_config_set_in_pins(pio_sm_config *c, uint8_t in_base) { c->in_base = in_base; }
void sm_config_set_in_pin_base(pio_sm_config *c, uint8_t in_base) { c->in_base = in_base; }
#if PIO_SIM_HAS_IN_PIN_COUNT
void sm_config_set_in_pin_count(pio_sm_config *c, uint8_t in_count)
{
    c->in_count = ((in_count == 0U) || (in_count > 32U)) ? 32U : in_count;
}
#endif
void sm_config_set_sideset_pins(pio_sm_config *c, uint8_t sideset_base)
{
    c->sideset_base = sideset_base;
}
void sm_config_set_sideset(pio_sm_config *c, uint8_t bit_count, bool optional, bool pindirs)
{
    c->sideset_total_bits = bit_count;
    c->sideset_opt = optional;
    c->sideset_pindirs = pindirs;
}
void sm_config_set_wrap(pio_sm_config *c, uint8_t wrap_target, uint8_t wrap)
{
    /* Both ends address the 32-word instruction memory. */
    c->wrap_bottom = (uint8_t)(wrap_target % PIO_SIM_INSN_COUNT);
    c->wrap_top = (uint8_t)(wrap % PIO_SIM_INSN_COUNT);
}
void sm_config_set_clkdiv_int_frac(pio_sm_config *c, uint16_t div_int, uint8_t div_frac)
{
    c->clkdiv_int = div_int;
    c->clkdiv_frac = div_frac;
}
void sm_config_set_clkdiv(pio_sm_config *c, float div)
{
    /* Guard the float→int conversions: a value that can't be represented in the
     * target integer type is UB. Clamp to the hardware range [1, 65536); the
     * SDK encodes a divisor of 65536 as clkdiv_int == 0, but that is only
     * reachable via set_clkdiv_int_frac, not this float helper. */
    if (!(div >= 1.0F)) { /* also catches NaN */
        div = 1.0F;
    } else if (div > 65535.99F) {
        div = 65535.99F;
    }
    uint16_t whole = (uint16_t)div;
    uint8_t frac = (uint8_t)((div - (float)whole) * 256.0F);
    c->clkdiv_int = whole;
    c->clkdiv_frac = frac;
}
void sm_config_set_out_shift(pio_sm_config *c, bool shift_right, bool autopull,
                             uint8_t pull_threshold)
{
    c->out_dir = shift_right ? PIO_SHIFT_RIGHT : PIO_SHIFT_LEFT;
    c->autopull = autopull;
    c->pull_thresh = clamp_thresh(pull_threshold);
}
void sm_config_set_in_shift(pio_sm_config *c, bool shift_right, bool autopush,
                            uint8_t push_threshold)
{
    c->in_dir = shift_right ? PIO_SHIFT_RIGHT : PIO_SHIFT_LEFT;
    c->autopush = autopush;
    c->push_thresh = clamp_thresh(push_threshold);
}
void sm_config_set_fifo_join(pio_sm_config *c, pio_fifo_join_t join)
{
    c->fifo_join = (uint8_t)join;
}
void sm_config_set_mov_status(pio_sm_config *c, pio_status_sel_t status_sel, uint8_t status_n)
{
    c->status_sel = (uint8_t)status_sel;
    c->status_n = status_n;
}
void sm_config_set_jmp_pin(pio_sm_config *c, uint8_t pin) { c->jmp_pin = pin; }
void sm_config_set_out_special(pio_sm_config *c, bool sticky, bool has_enable_pin,
                               uint8_t enable_bit_index)
{
    c->out_sticky = sticky;
    c->out_inline_en = has_enable_pin;
    c->out_en_sel = (uint8_t)(enable_bit_index & 0x1FU);
}

void pio_sim_clkdiv_restart_sm_mask(pio_sim_t *pio, uint8_t sm_mask)
{
    for (uint8_t i = 0; i < PIO_SIM_NUM_SM; i++) {
        if ((sm_mask & (uint8_t)(1U << i)) != 0U) {
            pio->sm[i].clk_accum = 0;
        }
    }
}

void pio_sim_sm_clkdiv_restart(pio_sim_t *pio, uint8_t sm) { pio->sm[SM_IDX(sm)].clk_accum = 0; }

void pio_sim_sm_set_pc(pio_sim_t *pio, uint8_t sm, uint8_t pc)
{
    /* PC indexes the 32-word instruction memory; keep it in range. */
    pio->sm[SM_IDX(sm)].pc = (uint8_t)(pc % PIO_SIM_INSN_COUNT);
}

uint8_t pio_sim_sm_get_pc(const pio_sim_t *pio, uint8_t sm) { return pio->sm[SM_IDX(sm)].pc; }

uint32_t pio_sim_sm_get_x(const pio_sim_t *pio, uint8_t sm) { return pio->sm[SM_IDX(sm)].x; }
uint32_t pio_sim_sm_get_y(const pio_sim_t *pio, uint8_t sm) { return pio->sm[SM_IDX(sm)].y; }
uint32_t pio_sim_sm_get_isr(const pio_sim_t *pio, uint8_t sm) { return pio->sm[SM_IDX(sm)].isr; }
uint32_t pio_sim_sm_get_osr(const pio_sim_t *pio, uint8_t sm) { return pio->sm[SM_IDX(sm)].osr; }

uint8_t pio_sim_sm_get_isr_count(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].isr_count;
}

uint8_t pio_sim_sm_get_osr_count(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].osr_count;
}

uint16_t pio_sim_sm_get_instruction(const pio_sim_t *pio, uint8_t sm)
{
    const pio_sm_t *s = &pio->sm[SM_IDX(sm)];
    return s->exec_pending ? s->exec_insn : pio->insn[s->pc % PIO_SIM_INSN_COUNT];
}

void pio_sim_sm_set_status_value(pio_sim_t *pio, uint8_t sm, uint32_t value)
{
    pio->sm[SM_IDX(sm)].status_value = value;
    pio->sm[SM_IDX(sm)].status_override = true;
}

void pio_sim_sm_clear_status_value(pio_sim_t *pio, uint8_t sm)
{
    pio->sm[SM_IDX(sm)].status_override = false;
}

bool pio_sim_sm_get_status_value(const pio_sim_t *pio, uint8_t sm, uint32_t *value)
{
    const pio_sm_t *s = &pio->sm[SM_IDX(sm)];
    if (s->status_override && (value != NULL)) {
        *value = s->status_value;
    }
    return s->status_override;
}

/* Apply a FIFO-join mode to an SM: reshape the TX/RX capacities and clear both
 * (changing the join reshapes the storage, as on hardware). */
static void apply_fifo_join(pio_sm_t *s, pio_fifo_join_t join)
{
    uint8_t old_join = s->fifo_join;
    s->fifo_join = (uint8_t)join;
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
    case PIO_FIFO_JOIN_RX_PUTGET:
#if PIO_SIM_HAS_RXFIFO_MOV
        /* RP2350 random-access RX: the RX FIFO becomes a 4-entry register file
         * addressed by the MOV-RX-FIFO instructions. Only the RX FIFO is
         * repurposed — the TX FIFO stays a normal 4-deep FIFO (RP2350 datasheet
         * §11.6: FJOIN_RX_PUT/GET affect the RX FIFO alone). */
        s->tx.cap = PIO_SIM_FIFO_DEPTH;
        s->rx.cap = PIO_SIM_FIFO_DEPTH;
#else
        /* RP2040: these modes do not exist; behave as the default 4+4 split. */
        s->tx.cap = PIO_SIM_FIFO_DEPTH;
        s->rx.cap = PIO_SIM_FIFO_DEPTH;
        s->fifo_join = (uint8_t)PIO_FIFO_JOIN_NONE;
#endif
        break;
    case PIO_FIFO_JOIN_NONE:
    default:
        s->tx.cap = PIO_SIM_FIFO_DEPTH;
        s->rx.cap = PIO_SIM_FIFO_DEPTH;
        break;
    }
    /* Silicon clears both FIFOs only when the FJOIN field actually changes (a
     * side effect of rewriting SHIFTCTRL); an unchanged join leaves FIFO
     * contents intact. This is what pio_sm_set_config does — the explicit
     * clear-on-init lives in pio_sim_sm_init, mirroring pio_sm_init's own
     * pio_sm_clear_fifos call. */
    if (s->fifo_join != old_join) {
        fifo_clear(&s->tx);
        fifo_clear(&s->rx);
    }
}

void pio_sim_sm_set_config(pio_sim_t *pio, uint8_t sm, const pio_sm_config *c)
{
    pio_sm_t *s = &pio->sm[SM_IDX(sm)];
    s->wrap_bottom = c->wrap_bottom;
    s->wrap_top = c->wrap_top;
    s->out_base = c->out_base;
    s->out_count = c->out_count;
    s->set_base = c->set_base;
    s->set_count = c->set_count;
    s->in_base = c->in_base;
    s->in_count = c->in_count;
    s->sideset_base = c->sideset_base;
    s->sideset_total_bits = c->sideset_total_bits;
    s->sideset_opt = c->sideset_opt;
    s->sideset_pindirs = c->sideset_pindirs;
    s->jmp_pin = c->jmp_pin;
    s->out_dir = c->out_dir;
    s->in_dir = c->in_dir;
    s->autopull = c->autopull;
    s->autopush = c->autopush;
    s->pull_thresh = c->pull_thresh;
    s->push_thresh = c->push_thresh;
    s->clkdiv_int = c->clkdiv_int;
    s->clkdiv_frac = c->clkdiv_frac;
    s->status_sel = c->status_sel;
    s->status_n = c->status_n;
    s->status_override = false;
    s->out_sticky = c->out_sticky;
    s->out_inline_en = c->out_inline_en;
    s->out_en_sel = c->out_en_sel;
    apply_fifo_join(s, (pio_fifo_join_t)c->fifo_join);
}

void pio_sim_sm_init(pio_sim_t *pio, uint8_t sm, uint8_t initial_pc, const pio_sm_config *c)
{
    pio_sim_sm_set_config(pio, sm, c);
    pio_sm_t *s = &pio->sm[SM_IDX(sm)];
    /* Reset the execution state (scratch, shift regs, delay, stall), as
     * pio_sm_init does, then point the PC at initial_pc. Clear the FIFOs
     * explicitly (pio_sm_init calls pio_sm_clear_fifos) — set_config only
     * clears them on a join change, so init must do it unconditionally. */
    fifo_clear(&s->tx);
    fifo_clear(&s->rx);
    s->x = 0;
    s->y = 0;
    s->osr = 0;
    s->isr = 0;
    s->osr_count = 32;
    s->isr_count = 0;
    s->delay = 0;
    s->stalled = false;
    s->exec_pending = false;
    s->exec_insn = 0;
    s->clk_accum = 0;
    s->fdebug = 0;
    s->enabled = false;
    /* Keep the PC in range like pio_sim_sm_set_pc does, so an out-of-range
     * initial_pc can't desync the post-commit PC increment. */
    s->pc = (uint8_t)(initial_pc % PIO_SIM_INSN_COUNT);
}

void pio_sim_sm_set_consecutive_pindirs(pio_sim_t *pio, uint8_t sm, uint8_t base, uint8_t count,
                                        bool is_out)
{
    pio_sm_t *s = &pio->sm[SM_IDX(sm)];
    count = clamp_pin_count(count); /* at most one full 32-pin window */
    for (uint8_t i = 0; i < count; i++) {
        uint64_t bit = (uint64_t)1U << phys_pin(pio, (uint8_t)(base + i));
        if (is_out) {
            s->out_pin_oe |= bit;
        } else {
            s->out_pin_oe &= ~bit;
        }
    }
    resolve_pads(pio->pads);
}

void pio_sim_set_device(pio_sim_t *pio, void (*on_tick)(pio_sim_t *, void *), void *ctx)
{
    pio->on_tick = on_tick;
    pio->device_ctx = ctx;
}

void pio_sim_set_trace(pio_sim_t *pio, pio_sim_trace_fn fn, void *ctx)
{
    pio->on_insn = fn;
    pio->trace_ctx = ctx;
}

#if PIO_SIM_HAS_IRQ_PREVNEXT
void pio_sim_set_irq_neighbors(pio_sim_t *pio, pio_sim_t *prev, pio_sim_t *next)
{
    pio->irq_prev = prev;
    pio->irq_next = next;
}
#endif

#if PIO_SIM_HAS_GPIO_BASE
void pio_sim_set_gpio_base(pio_sim_t *pio, uint8_t base)
{
    pio->gpio_base = (base >= 16U) ? 16U : 0U; /* hardware allows only 0 or 16 */
}

uint8_t pio_sim_get_gpio_base(const pio_sim_t *pio) { return pio->gpio_base; }
#endif

void pio_sim_sync_settle(pio_sim_t *pio)
{
    uint64_t view = pads_input_view(pio->pads);
    pio->pads->in_sync[0] = view;
    pio->pads->in_sync[1] = view;
}

void pio_sim_set_pull_level(pio_sim_t *pio, uint64_t mask, bool level)
{
    /* Thin alias over the PADS_BANK0 pull bits: up = PUE, down = PDE. */
    if (level) {
        pio->pads->pad_pue |= mask;
        pio->pads->pad_pde &= ~mask;
    } else {
        pio->pads->pad_pde |= mask;
        pio->pads->pad_pue &= ~mask;
    }
}

void pio_sim_set_input_sync_bypass(pio_sim_t *pio, uint64_t mask) { pio->pads->sync_bypass = mask; }

/* ── FIFO access ───────────────────────────────────────────────────────────── */

bool pio_sim_sm_is_tx_fifo_full(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].tx.count >= pio->sm[SM_IDX(sm)].tx.cap;
}
bool pio_sim_sm_is_tx_fifo_empty(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].tx.count == 0U;
}
bool pio_sim_sm_is_rx_fifo_full(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].rx.count >= pio->sm[SM_IDX(sm)].rx.cap;
}
bool pio_sim_sm_is_rx_fifo_empty(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].rx.count == 0U;
}

bool pio_sim_sm_put(pio_sim_t *pio, uint8_t sm, uint32_t word)
{
    if (!fifo_push(&pio->sm[SM_IDX(sm)].tx, word)) {
        pio->sm[SM_IDX(sm)].fdebug |= PIO_FDEBUG_TXOVER; /* host wrote a full TX FIFO */
        return false;
    }
    return true;
}

bool pio_sim_sm_get(pio_sim_t *pio, uint8_t sm, uint32_t *word)
{
    if (!fifo_pop(&pio->sm[SM_IDX(sm)].rx, word)) {
        pio->sm[SM_IDX(sm)].fdebug |= PIO_FDEBUG_RXUNDER; /* host read an empty RX FIFO */
        return false;
    }
    return true;
}

void pio_sim_sm_clear_fifos(pio_sim_t *pio, uint8_t sm)
{
    fifo_clear(&pio->sm[SM_IDX(sm)].tx);
    fifo_clear(&pio->sm[SM_IDX(sm)].rx);
}

#if PIO_SIM_HAS_RXFIFO_MOV
uint32_t pio_sim_sm_rxfifo_get(const pio_sim_t *pio, uint8_t sm, uint8_t index)
{
    return pio->sm[SM_IDX(sm)].rx.buf[index & 0x3U];
}

void pio_sim_sm_rxfifo_put(pio_sim_t *pio, uint8_t sm, uint8_t index, uint32_t word)
{
    pio->sm[SM_IDX(sm)].rx.buf[index & 0x3U] = word;
}
#endif

uint8_t pio_sim_sm_get_tx_fifo_level(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].tx.count;
}
uint8_t pio_sim_sm_get_rx_fifo_level(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].rx.count;
}

uint8_t pio_sim_sm_get_fdebug(const pio_sim_t *pio, uint8_t sm)
{
    return pio->sm[SM_IDX(sm)].fdebug;
}

void pio_sim_sm_clear_fdebug(pio_sim_t *pio, uint8_t sm, uint8_t mask)
{
    pio->sm[SM_IDX(sm)].fdebug &= (uint8_t)~mask;
}

/* ── Pin access ────────────────────────────────────────────────────────────── */

bool pio_sim_get_pin(const pio_sim_t *pio, uint8_t pin)
{
    return (pads_live(pio->pads) & ((uint64_t)1U << (pin % PIO_SIM_NUM_PINS))) != 0U;
}

void pio_sim_set_pin(pio_sim_t *pio, uint8_t pin, bool level)
{
    uint64_t bit = (uint64_t)1U << ((uint32_t)pin % PIO_SIM_NUM_PINS);
    pio->pads->ext_drive |= bit; /* host now drives this pin */
    if (level) {
        pio->pads->ext_levels |= bit;
    } else {
        pio->pads->ext_levels &= ~bit;
    }
}

void pio_sim_release_pin(pio_sim_t *pio, uint8_t pin)
{
    pio->pads->ext_drive &= ~((uint64_t)1U << ((uint32_t)pin % PIO_SIM_NUM_PINS));
}

bool pio_sim_pin_is_pio_output(const pio_sim_t *pio, uint8_t pin)
{
    /* The PIO's resolved OE as it reaches the pad: output-disable (and RP2350
     * isolation) block the drive even though the SM's OE register stays set. */
    uint64_t oe = pio->pads->pin_dirs & ~pio->pads->pad_od;
#if PIO_SIM_HAS_PAD_ISO
    oe &= ~pio->pads->pad_iso;
#endif
    return (oe & ((uint64_t)1U << (pin % PIO_SIM_NUM_PINS))) != 0U;
}

/* Level a state machine samples for an input pin (physical GPIO): the second
 * synchroniser stage (two clocks behind the live pad), unless the pin's
 * synchroniser is bypassed, in which case the live level is used directly. */
static bool pio_input_level(const pio_sim_t *pio, uint8_t pin)
{
    const pio_pads_t *p = pio->pads;
    uint64_t bit = (uint64_t)1U << (pin % PIO_SIM_NUM_PINS);
    uint64_t synced = (p->in_sync[1] & ~p->sync_bypass) | (pads_input_view(p) & p->sync_bypass);
    return (synced & bit) != 0U;
}

/* An OUT/SET/MOV PINS or side-set write updates this SM's pin output register,
 * records the pins as written this cycle (they compete for same-cycle priority),
 * then re-resolves the pad. The pin only drives the pad where this SM's pindir
 * (oe) is also set. */
static void drive_pins(pio_sim_t *pio, pio_sm_t *sm, uint8_t base, uint8_t count, uint32_t value)
{
    count = clamp_pin_count(count);
    for (uint8_t i = 0; i < count; i++) {
        uint64_t bit = (uint64_t)1U << phys_pin(pio, (uint8_t)(base + i));
        if (((value >> i) & 1U) != 0U) {
            sm->out_pin_val |= bit;
        } else {
            sm->out_pin_val &= ~bit;
        }
        sm->wrote_this_cycle |= bit;
    }
    resolve_pads(pio->pads);
}

/* An OUT/SET/MOV PINDIRS or side-set-pindirs write updates this SM's output-enable
 * (pindir) register, then re-resolves the pad. */
static void drive_pindirs(pio_sim_t *pio, pio_sm_t *sm, uint8_t base, uint8_t count, uint32_t value)
{
    count = clamp_pin_count(count);
    for (uint8_t i = 0; i < count; i++) {
        uint64_t bit = (uint64_t)1U << phys_pin(pio, (uint8_t)(base + i));
        if (((value >> i) & 1U) != 0U) {
            sm->out_pin_oe |= bit;
        } else {
            sm->out_pin_oe &= ~bit;
        }
    }
    resolve_pads(pio->pads);
}

/* Drop an OUT pin span from this SM's output enable (inline OUT enable = 0 under
 * OUT_STICKY): the SM stops driving those pins, so a lower-priority SM / external
 * level / pull shows through. */
static void release_out_pins(pio_sim_t *pio, pio_sm_t *sm, uint8_t base, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        uint64_t bit = (uint64_t)1U << phys_pin(pio, (uint8_t)(base + i));
        sm->out_pin_oe &= ~bit;
        sm->wrote_this_cycle &= ~bit; /* released pins no longer compete */
    }
    resolve_pads(pio->pads);
}

static uint32_t read_in_pins(const pio_sim_t *pio, uint8_t base, uint8_t count)
{
    uint32_t v = 0;
    count = clamp_pin_count(count);
    for (uint8_t i = 0; i < count; i++) {
        uint8_t pin = phys_pin(pio, (uint8_t)(base + i));
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

/* ── System interrupt lines ────────────────────────────────────────────────── */

uint32_t pio_sim_get_intr(const pio_sim_t *pio)
{
    uint32_t intr = 0;
    for (uint8_t sm = 0; sm < PIO_SIM_NUM_SM; sm++) {
        if (!pio_sim_sm_is_rx_fifo_empty(pio, sm)) {
            intr |= PIO_INTR_SM_RXNEMPTY(sm);
        }
        if (!pio_sim_sm_is_tx_fifo_full(pio, sm)) {
            intr |= PIO_INTR_SM_TXNFULL(sm);
        }
    }
#if PIO_SIM_HAS_INTR_IRQ8
    uint8_t irq_route_count = PIO_SIM_NUM_IRQ; /* RP2350: all 8 flags route */
#else
    uint8_t irq_route_count = 4U; /* RP2040: only the low four flags route */
#endif
    for (uint8_t i = 0; i < irq_route_count; i++) {
        if ((pio->irq & (1U << i)) != 0U) {
            intr |= PIO_INTR_SM_IRQ(i);
        }
    }
    return intr;
}

void pio_sim_set_irqn_source_mask_enabled(pio_sim_t *pio, uint8_t line, uint32_t source_mask,
                                          bool enabled)
{
    if (enabled) {
        pio->irq_inte[line & 1U] |= source_mask;
    } else {
        pio->irq_inte[line & 1U] &= ~source_mask;
    }
}

void pio_sim_set_irqn_source_enabled(pio_sim_t *pio, uint8_t line, uint32_t source, bool enabled)
{
    pio_sim_set_irqn_source_mask_enabled(pio, line, source, enabled);
}

uint32_t pio_sim_get_inte(const pio_sim_t *pio, uint8_t line) { return pio->irq_inte[line & 1U]; }

void pio_sim_set_intf(pio_sim_t *pio, uint8_t line, uint32_t mask, bool on)
{
    if (on) {
        pio->irq_intf[line & 1U] |= mask;
    } else {
        pio->irq_intf[line & 1U] &= ~mask;
    }
}

uint32_t pio_sim_get_intf(const pio_sim_t *pio, uint8_t line) { return pio->irq_intf[line & 1U]; }

uint32_t pio_sim_get_ints(const pio_sim_t *pio, uint8_t line)
{
    uint8_t l = (uint8_t)(line & 1U);
    return (pio_sim_get_intr(pio) & pio->irq_inte[l]) | pio->irq_intf[l];
}

bool pio_sim_get_irqn_asserted(const pio_sim_t *pio, uint8_t line)
{
    return pio_sim_get_ints(pio, line) != 0U;
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
#if !PIO_SIM_HAS_IRQ_STATUS
    (void)pio; /* RP2040: STATUS never reads an IRQ flag, so pio is unused. */
#endif
    if (sm->status_override) {
        return sm->status_value;
    }
    bool cond;
    switch (sm->status_sel) {
    case PIO_STATUS_RX_LEVEL:
        cond = (sm->rx.count < sm->status_n);
        break;
    case PIO_STATUS_IRQ_SET:
#if PIO_SIM_HAS_IRQ_STATUS
        cond = pio_sim_irq_get(pio, (uint8_t)(sm->status_n & 7U));
        break;
#if PIO_SIM_HAS_IRQ_PREVNEXT
    case PIO_STATUS_IRQ_SET_PREV:
        /* Unlinked neighbour reads clear, matching irq_target_block's NULL
         * convention for prev/next IRQ instructions. */
        cond =
            (pio->irq_prev != NULL) && pio_sim_irq_get(pio->irq_prev, (uint8_t)(sm->status_n & 7U));
        break;
    case PIO_STATUS_IRQ_SET_NEXT:
        cond =
            (pio->irq_next != NULL) && pio_sim_irq_get(pio->irq_next, (uint8_t)(sm->status_n & 7U));
        break;
#endif
#else
        /* RP2040: no IRQ status source; fall through to TX-level behaviour. */
        /* fallthrough */
#endif
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
        /* IN_COUNT masks the high pins of the IN group to 0 (RP2350); in_count is
         * 32 by default, so mask_n(32) leaves the value untouched on RP2040. */
        return read_in_pins(pio, sm->in_base, 32) & mask_n(sm->in_count);
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
        take = pio_input_level(pio, phys_pin(pio, sm->jmp_pin));
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

/* Which PIO block an IRQ / WAIT-IRQ index field addresses. PREV/NEXT (RP2350)
 * reach a neighbouring block; on RP2040, or when the neighbour is unlinked, they
 * resolve to no block. */
typedef enum {
    IRQ_BLOCK_SELF = 0,
    IRQ_BLOCK_PREV = 1,
    IRQ_BLOCK_NEXT = 2,
} irq_block_sel_t;

/* Effective IRQ flag for an IRQ / WAIT-IRQ index field (bits [4:0]) plus which
 * block it targets. When SM-relative, the SM number is added to the low two bits
 * of the index (mod 4); bit 2 is unaffected. */
typedef struct {
    uint8_t index;
    irq_block_sel_t block;
} irq_target_t;

static irq_target_t resolve_irq_target(uint8_t sm_idx, uint8_t field)
{
    uint8_t mode = (uint8_t)(field & PIO_IRQ_MODE_MASK);
    uint8_t base = (uint8_t)(field & 0x7U);
    irq_target_t t = {base, IRQ_BLOCK_SELF};
    switch (mode) {
    case PIO_IRQ_REL:
        t.index = (uint8_t)((base & 0x4U) | (((base & 0x3U) + sm_idx) & 0x3U));
        break;
    case PIO_IRQ_PREV:
        t.block = IRQ_BLOCK_PREV;
        break;
    case PIO_IRQ_NEXT:
        t.block = IRQ_BLOCK_NEXT;
        break;
    default: /* this PIO, absolute index */
        break;
    }
    return t;
}

/* Resolve an IRQ target to the PIO block to operate on. Returns NULL for a
 * prev/next target that is unlinked (always so on RP2040, which lacks the
 * inter-PIO IRQ feature). */
static pio_sim_t *irq_target_block(pio_sim_t *pio, irq_block_sel_t sel)
{
    switch (sel) {
    case IRQ_BLOCK_PREV:
#if PIO_SIM_HAS_IRQ_PREVNEXT
        return pio->irq_prev;
#else
        return NULL;
#endif
    case IRQ_BLOCK_NEXT:
#if PIO_SIM_HAS_IRQ_PREVNEXT
        return pio->irq_next;
#else
        return NULL;
#endif
    case IRQ_BLOCK_SELF:
    default:
        return pio;
    }
}

static bool exec_wait(pio_sim_t *pio, uint8_t sm_idx, uint8_t operand)
{
    const pio_sm_t *sm = &pio->sm[sm_idx];
    uint8_t pol = (uint8_t)((operand >> 7U) & 1U);
    uint8_t source = (uint8_t)((operand >> 5U) & 3U);
    uint8_t index = (uint8_t)(operand & 0x1FU);
    bool met = false;
    if (source == PIO_WAIT_GPIO) {
        met = (pio_input_level(pio, phys_pin(pio, index)) ? 1U : 0U) == pol;
    } else if (source == PIO_WAIT_PIN) {
        uint8_t pin = phys_pin(pio, (uint8_t)(sm->in_base + index));
        /* A pin at/above IN_COUNT is masked to 0 (RP2350); in_count defaults to 32
         * (index is ≤ 31), so this never masks on RP2040. */
        uint8_t lvl = (index < sm->in_count) && pio_input_level(pio, pin) ? 1U : 0U;
        met = lvl == pol;
    } else if (source == PIO_WAIT_IRQ) {
        irq_target_t t = resolve_irq_target(sm_idx, index);
        pio_sim_t *blk = irq_target_block(pio, t.block);
        if (blk == NULL) {
            met = false; /* unlinked neighbouring-PIO IRQ: never satisfied */
        } else {
            met = (pio_sim_irq_get(blk, t.index) ? 1U : 0U) == pol;
            if (met && (pol == 1U)) {
                pio_sim_irq_clear(blk, t.index); /* WAIT 1 IRQ clears the flag */
            }
        }
#if PIO_SIM_HAS_WAIT_JMPPIN
    } else if (source == PIO_WAIT_JMPPIN) {
        /* RP2350: wait on the JMP pin plus the index offset. */
        uint8_t pin = phys_pin(pio, (uint8_t)(sm->jmp_pin + index));
        met = (pio_input_level(pio, pin) ? 1U : 0U) == pol;
#endif
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
    if (will_autopush && pio_sim_sm_is_rx_fifo_full(pio, sm_idx)) {
        sm->fdebug |= PIO_FDEBUG_RXSTALL; /* autopush blocked by a full RX FIFO */
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
    case PIO_OUT_DST_PINS: {
        /* Inline OUT enable: bit out_en_sel of the OUT data gates the write. When
         * it is 0, the OUT does not write the pins (they hold); under OUT_STICKY it
         * also releases the OUT span (clears this SM's output enable). */
        bool driven = !sm->out_inline_en || (((val >> sm->out_en_sel) & 1U) != 0U);
        if (driven) {
            drive_pins(pio, sm, sm->out_base, sm->out_count, val);
        } else if (sm->out_sticky) {
            release_out_pins(pio, sm, sm->out_base, sm->out_count);
        }
        break;
    }
    case PIO_OUT_DST_X:
        sm->x = val;
        break;
    case PIO_OUT_DST_Y:
        sm->y = val;
        break;
    case PIO_OUT_DST_PINDIRS:
        drive_pindirs(pio, sm, sm->out_base, sm->out_count, val);
        break;
    case PIO_OUT_DST_PC:
        *set_pc = true;
        *next_pc = (uint8_t)(val & 0x1FU);
        break;
    case PIO_OUT_DST_ISR:
        sm->isr = val;
        sm->isr_count = count;
        break;
    case PIO_OUT_DST_EXEC:
        sm->exec_pending = true;
        sm->exec_insn = (uint16_t)val;
        break;
    case PIO_OUT_DST_NULL:
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
        /* The background refill at the top of this cycle already ran, so an
         * exhausted OSR here means TX was empty: stall. The OUT then completes
         * on a later cycle — at least one after a word arrives — matching the
         * §3.5.4.1 rule that an OUT cannot fill and shift the OSR same-cycle. */
        sm->fdebug |= PIO_FDEBUG_TXSTALL;
        return true;
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
    /* No in-instruction refill: the background autopull at the top of the next
     * SM cycle (sm_cycle) tops the OSR up before any following instruction
     * executes, so back-to-back OUTs still stream one word per cycle from a
     * fed FIFO, and JMP !OSRE / PULL-as-barrier observe the refreshed state.
     * Autopush is unchanged and deliberately eager: exec_in drains the ISR at
     * the end of the IN that crosses its threshold, matching the datasheet
     * pseudo-code (stall-before-shift when RX is full). */
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
        if (pio_sim_sm_is_rx_fifo_full(pio, sm_idx)) {
            sm->fdebug |= PIO_FDEBUG_RXSTALL; /* PUSH blocked/dropped on full RX */
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
    /* With autopull enabled, a PULL is a no-op while the OSR still holds data
     * (shift count below threshold): autopull already keeps the OSR topped up,
     * so PULL acts as a barrier rather than popping — and losing — another TX
     * word (RP2040 datasheet §3.5.4.2). */
    if (sm->autopull && (sm->osr_count < sm->pull_thresh)) {
        return false;
    }
    if (pio_sim_sm_is_tx_fifo_empty(pio, sm_idx)) {
        sm->fdebug |= PIO_FDEBUG_TXSTALL; /* PULL blocked/substituted on empty TX */
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

#if PIO_SIM_HAS_RXFIFO_MOV
/* RP2350 indexed RX-FIFO MOV (shares the PUSH/PULL opcode; selected when
 * operand bit 4 is set). bit 7 chooses direction
 * (0: rxfifo[idx] <- ISR, 1: OSR <- rxfifo[idx]); bit 3 selects the index
 * source (1: literal in bits [1:0], 0: scratch register Y). The RX FIFO is
 * treated as a 4-entry register file; see PIO_FIFO_JOIN_RX_PUT/GET. */
static void exec_mov_rxfifo(pio_sim_t *pio, uint8_t sm_idx, uint8_t operand)
{
    pio_sm_t *sm = &pio->sm[sm_idx];
    bool from_rx = (operand & PIO_MOV_RXFIFO_GET) != 0U;
    bool idx_literal = (operand & PIO_MOV_RXFIFO_IDX) != 0U;
    uint8_t idx = idx_literal ? (uint8_t)(operand & 0x3U) : (uint8_t)(sm->y & 0x3U);
    if (from_rx) {
        /* SM get is only valid with FJOIN_RX_GET set (GET or PUTGET mode). */
        if ((sm->fifo_join != (uint8_t)PIO_FIFO_JOIN_RX_GET) &&
            (sm->fifo_join != (uint8_t)PIO_FIFO_JOIN_RX_PUTGET)) {
            return;
        }
        sm->osr = sm->rx.buf[idx];
        sm->osr_count = 0;
    } else {
        /* SM put is only valid with FJOIN_RX_PUT set (PUT or PUTGET mode). */
        if ((sm->fifo_join != (uint8_t)PIO_FIFO_JOIN_RX_PUT) &&
            (sm->fifo_join != (uint8_t)PIO_FIFO_JOIN_RX_PUTGET)) {
            return;
        }
        sm->rx.buf[idx] = sm->isr;
        sm->isr = 0;
        sm->isr_count = 0;
    }
}
#endif /* PIO_SIM_HAS_RXFIFO_MOV */

static void exec_mov(pio_sim_t *pio, pio_sm_t *sm, uint8_t operand, uint8_t *next_pc, bool *set_pc)
{
    uint8_t dest = (uint8_t)((operand >> 5U) & 7U);
    uint8_t mov_op = (uint8_t)((operand >> 3U) & 3U);
    uint8_t src = (uint8_t)(operand & 7U);
    uint32_t v = read_source(pio, sm, src);
    if (mov_op == PIO_MOV_INVERT) {
        v = ~v;
    } else if (mov_op == PIO_MOV_REVERSE) {
        v = pio_reverse32(v);
    } else {
        /* PIO_MOV_NONE: value unchanged. */
    }
    switch (dest) {
    case PIO_MOV_DST_PINS:
        drive_pins(pio, sm, sm->out_base, sm->out_count, v);
        break;
    case PIO_MOV_DST_X:
        sm->x = v;
        break;
    case PIO_MOV_DST_Y:
        sm->y = v;
        break;
#if PIO_SIM_HAS_MOV_PINDIRS
    case PIO_MOV_DST_PINDIRS: /* RP2350 only: drives the OUT pin dirs */
        drive_pindirs(pio, sm, sm->out_base, sm->out_count, v);
        break;
#endif
    case PIO_MOV_DST_EXEC:
        sm->exec_pending = true;
        sm->exec_insn = (uint16_t)v;
        break;
    case PIO_MOV_DST_PC:
        *set_pc = true;
        *next_pc = (uint8_t)(v & 0x1FU);
        break;
    case PIO_MOV_DST_ISR:
        sm->isr = v;
        sm->isr_count = 0;
        break;
    case PIO_MOV_DST_OSR:
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
    pio_sim_t *blk = irq_target_block(pio, t.block);
    if (blk == NULL) {
        /* prev/next target an unlinked neighbouring PIO block: set and clear
         * have no effect; a wait can never be satisfied here. */
        return wait && !clr;
    }
    uint8_t irq = t.index;
    if (clr) {
        pio_sim_irq_clear(blk, irq);
        return false;
    }
    if (wait) {
        /* IRQ ... wait: raise the flag on the cycle the instruction is first
         * presented, then park here until something clears it. sm->stalled is
         * false on the first execution and true on each retry, so we raise once
         * and merely poll thereafter (re-raising would defeat an external
         * clear). */
        if (!sm->stalled) {
            blk->irq |= (uint8_t)(1U << irq);
        }
        return pio_sim_irq_get(blk, irq);
    }
    blk->irq |= (uint8_t)(1U << irq);
    return false;
}

static void exec_set(pio_sim_t *pio, pio_sm_t *sm, uint8_t operand)
{
    uint8_t dest = (uint8_t)((operand >> 5U) & 7U);
    uint32_t value = ((uint32_t)operand & 0x1FU);
    switch (dest) {
    case PIO_DST_PINS:
        drive_pins(pio, sm, sm->set_base, sm->set_count, value);
        break;
    case PIO_DST_X:
        sm->x = value;
        break;
    case PIO_DST_Y:
        sm->y = value;
        break;
    case PIO_DST_PINDIRS:
        drive_pindirs(pio, sm, sm->set_base, sm->set_count, value);
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
#if PIO_SIM_HAS_RXFIFO_MOV
        /* RP2350 indexed RX-FIFO MOV reuses this opcode; operand bit 4 is the
         * discriminator (plain PUSH/PULL encode it as 0). */
        if ((operand & PIO_MOV_RXFIFO_BIT) != 0U) {
            exec_mov_rxfifo(pio, sm_idx, operand);
        } else {
            stalled = exec_pushpull(pio, sm_idx, operand);
        }
#else
        /* RP2040: no indexed RX-FIFO MOV; the low operand bits are reserved. */
        stalled = exec_pushpull(pio, sm_idx, operand);
#endif
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
            drive_pindirs(pio, sm, sm->sideset_base, ds.ss_data_bits, ds.side_val);
        } else {
            drive_pins(pio, sm, sm->sideset_base, ds.ss_data_bits, ds.side_val);
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

    /* Background autopull: hardware tops the OSR up from TX on any cycle where
     * the shift count has reached the threshold — stall and delay cycles
     * included — independently of what instruction is executing (RP2040
     * datasheet §3.5.4.1). Running it before this cycle's instruction also
     * enforces the documented rule that an OUT cannot fill the OSR and shift
     * from it on the same cycle: an OUT that finds the OSR exhausted stalls at
     * least one cycle while the refill lands. (The datasheet notes the exact
     * refill point is pipeline-dependent and not to be relied upon; this
     * models the documented rules at one-tick granularity.) */
    if (sm->autopull && (sm->osr_count >= sm->pull_thresh)) {
        (void)osr_refill(sm);
    }

    if (sm->delay > 0U) {
        sm->delay--;
        return;
    }

    uint16_t insn;
    bool from_exec = sm->exec_pending;
    if (from_exec) {
        insn = sm->exec_insn;
    } else {
        /* Fetch address is masked to the 32-word memory: a caller-poked PC or a
         * bad wrap can never index out of bounds. */
        uint8_t fetch_pc = (uint8_t)(sm->pc % PIO_SIM_INSN_COUNT);
        if (!pio->insn_used[fetch_pc]) {
            pio->unwritten_fetches++; /* diagnostic: runaway PC / bad wrap */
        }
        insn = pio->insn[fetch_pc];
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
    if (pio->on_insn != NULL) {
        /* PC not yet advanced: report the committed word's fetch address. */
        pio->on_insn(pio, sm_idx, sm->pc, insn, pio->trace_ctx);
    }

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
    /* Open a fresh write window: a pin write is a one-shot event of the cycle
     * it happens in, so last tick's writes must not keep competing for the
     * same-cycle priority collation in resolve_pads. */
    for (uint8_t i = 0; i < PIO_SIM_NUM_SM; i++) {
        pio->sm[i].wrote_this_cycle = 0;
    }
    for (uint8_t i = 0; i < PIO_SIM_NUM_SM; i++) {
        pio_sm_t *sm = &pio->sm[i];
        /* Clock divider: accumulate 1.0 (in 8-bit frac units = 256) each system
         * tick; an SM cycle is due when the accumulator reaches the divider. The
         * divider free-runs even while the SM is disabled (only execution is gated
         * by enable) — matching hardware, which is why pio_enable_sm_mask_in_sync
         * must restart the dividers to align them rather than rely on enable timing. */
        /* div_int == 0 encodes 65536 regardless of the fractional part. */
        uint32_t div_int = (sm->clkdiv_int == 0U) ? 65536U : (uint32_t)sm->clkdiv_int;
        uint32_t div_units = (div_int << 8U) | sm->clkdiv_frac;
        sm->clk_accum += 256U;
        if (sm->clk_accum >= div_units) {
            sm->clk_accum -= div_units;
            if (sm->enabled) {
                sm_cycle(pio, i);
                if (sm->out_sticky) {
                    /* OUT_STICKY: re-assert every driven pin each cycle, keeping
                     * this SM in the priority contest even without a new write. */
                    sm->wrote_this_cycle |= sm->out_pin_oe;
                }
            }
        }
    }
    /* Re-resolve once with every SM's final write set for this tick, so sticky
     * re-assertions and the cross-SM priority collation land before the device
     * hook and the input synchroniser observe the pads. */
    resolve_pads(pio->pads);
    if (pio->on_tick != NULL) {
        pio->on_tick(pio, pio->device_ctx);
    }
    /* Clock the two-stage input synchroniser from the (now updated) pads. When
     * pads are shared across a group the group clocks them once instead, so a
     * block only clocks the synchroniser of pads it owns. */
    if (pio->pads == &pio->pads_embedded) {
        pads_update_keeper(pio->pads);
        pio->pads->in_sync[1] = pio->pads->in_sync[0];
        pio->pads->in_sync[0] = pads_input_view(pio->pads);
    }
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
    while ((t < max_ticks) && pio_sim_sm_is_rx_fifo_empty(pio, sm)) {
        pio_sim_tick(pio);
        t++;
    }
    return t;
}

uint64_t pio_sim_run_until_tx_empty(pio_sim_t *pio, uint8_t sm, uint64_t max_ticks)
{
    uint64_t t = 0;
    /* FIFO-empty only: the SM may still hold (and be shifting) a word it
     * already pulled into the OSR — use pio_sim_run_until_tx_drained to wait
     * for that word too. */
    while ((t < max_ticks) && !pio_sim_sm_is_tx_fifo_empty(pio, sm)) {
        pio_sim_tick(pio);
        t++;
    }
    return t;
}

uint64_t pio_sim_run_until_tx_drained(pio_sim_t *pio, uint8_t sm, uint64_t max_ticks)
{
    /* The pico-sdk idiom: clear the sticky TXSTALL flag, then run until it
     * sets again — the SM stalling on an empty TX means the FIFO is empty AND
     * the OSR's last word has been fully consumed. Requires the program to
     * keep OUTing/PULLing (as streaming programs do); bounded by max_ticks. */
    pio_sim_sm_clear_fdebug(pio, sm, PIO_FDEBUG_TXSTALL);
    uint64_t t = 0;
    while ((t < max_ticks) && ((pio_sim_sm_get_fdebug(pio, sm) & PIO_FDEBUG_TXSTALL) == 0U)) {
        pio_sim_tick(pio);
        t++;
    }
    return t;
}

/* ── Multi-PIO group ───────────────────────────────────────────────────────── */

void pio_sim_group_init(pio_sim_group_t *g, pio_sim_t *const *blocks, uint8_t count)
{
    if (count > PIO_SIM_NUM_PIO) {
        count = PIO_SIM_NUM_PIO;
    }
    g->count = count;
    g->shared = false; /* pio_sim_group_init_shared sets this true after init */
    for (uint8_t i = 0; i < count; i++) {
        g->blk[i] = blocks[i];
    }
#if PIO_SIM_HAS_IRQ_PREVNEXT
    /* Wire the inter-PIO IRQ links into a ring (PIO0->PIO1->...->PIO0). A single
     * block links to itself, which matches a `prev`/`next` that wraps around. */
    for (uint8_t i = 0; i < count; i++) {
        pio_sim_t *prev = g->blk[(i + count - 1U) % count];
        pio_sim_t *next = g->blk[(i + 1U) % count];
        pio_sim_set_irq_neighbors(g->blk[i], prev, next);
    }
#endif
}

void pio_sim_group_init_shared(pio_sim_group_t *g, pio_sim_t *const *blocks, uint8_t count)
{
    pio_sim_group_init(g, blocks, count);
    (void)memset(&g->pads, 0, sizeof(g->pads));
    pio_pads_init_defaults(&g->pads);
    g->shared = true;
    for (uint8_t i = 0; i < g->count; i++) {
        g->blk[i]->pads = &g->pads;    /* all blocks now drive/sample the same wires */
        g->pads.owners[i] = g->blk[i]; /* …and all contribute to its resolution */
    }
    g->pads.owner_count = g->count;
}

void pio_sim_group_tick(pio_sim_group_t *g)
{
    /* Open the write window for the whole group before any block runs: with
     * shared pads, resolve_pads at the end of block A's tick scans every
     * owner's SMs, so block B's flags from the previous tick must already be
     * cleared or B's stale writes would wrongly win same-cycle priority in the
     * pad state block A's on_tick hook observes. (Each block's own tick
     * re-clears its own flags — harmless.) Remaining sequential-model caveat:
     * block A's on_tick still runs before later blocks have executed this
     * tick, so it sees their *previous* outputs, one tick stale. */
    for (uint8_t i = 0; i < g->count; i++) {
        for (uint8_t s = 0; s < PIO_SIM_NUM_SM; s++) {
            g->blk[i]->sm[s].wrote_this_cycle = 0;
        }
    }
    for (uint8_t i = 0; i < g->count; i++) {
        pio_sim_tick(g->blk[i]);
    }
    if (g->shared) {
        /* Blocks skip the synchroniser of pads they don't own, so clock the one
         * shared pipeline here — exactly once per system tick. */
        pads_update_keeper(&g->pads);
        g->pads.in_sync[1] = g->pads.in_sync[0];
        g->pads.in_sync[0] = pads_input_view(&g->pads);
    }
}

void pio_sim_group_run(pio_sim_group_t *g, uint64_t n)
{
    for (uint64_t i = 0; i < n; i++) {
        pio_sim_group_tick(g);
    }
}

void pio_sim_group_enable_sm_mask_in_sync(pio_sim_group_t *g, const uint8_t *masks)
{
    for (uint8_t i = 0; i < g->count; i++) {
        pio_sim_enable_sm_mask_in_sync(g->blk[i], masks[i]);
    }
}

/* ── sm_exec ───────────────────────────────────────────────────────────────── */

void pio_sim_sm_exec(pio_sim_t *pio, uint8_t sm, uint16_t insn)
{
    /* Execute immediately, out of band, like pio_sm_exec(). Side-set still
     * applies but the delay field is IGNORED — matching silicon, where delay
     * cycles on a forced (SMx_INSTR-written) instruction do not occur (RP2040
     * datasheet §3.4.5.2; contrast OUT/MOV EXEC, whose injected instruction
     * does execute its delay — see sm_cycle's from_exec path). PC is untouched
     * unless the instruction writes it. Clear any stall latched by the SM's
     * own program first: the injected instruction is a fresh first
     * presentation, so first-cycle side effects (e.g. `irq n wait` raising its
     * flag) must fire — otherwise the wait could never be satisfied. */
    pio->sm[SM_IDX(sm)].stalled = false;
    uint8_t next_pc = 0;
    bool set_pc = false;
    uint8_t delay = 0;
    bool committed = exec_one(pio, SM_IDX(sm), insn, &next_pc, &set_pc, &delay);
    (void)delay; /* a forced instruction executes immediately; its delay field
                  * is ignored (side-set still applies). */
    if (committed) {
        if (pio->on_insn != NULL) {
            pio->on_insn(pio, SM_IDX(sm), pio->sm[SM_IDX(sm)].pc, insn, pio->trace_ctx);
        }
        if (set_pc) {
            pio->sm[SM_IDX(sm)].pc = next_pc;
        }
    } else {
        /* A stalling instruction injected via exec latches as pending so the
         * SM retries it on its next cycle. Mark it stalled too, so a parking
         * instruction (e.g. `irq ... wait`) treats the next cycle as a retry and
         * does not re-trigger its first-cycle side effect (re-raising the flag). */
        pio->sm[SM_IDX(sm)].exec_pending = true;
        pio->sm[SM_IDX(sm)].exec_insn = insn;
        pio->sm[SM_IDX(sm)].stalled = true;
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

#if PIO_SIM_HAS_RXFIFO_MOV
uint16_t pio_sim_encode_mov_to_rxfifo(uint8_t index)
{
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | PIO_MOV_RXFIFO_BIT | PIO_MOV_RXFIFO_IDX |
                      (index & 0x3U));
}

uint16_t pio_sim_encode_mov_from_rxfifo(uint8_t index)
{
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | PIO_MOV_RXFIFO_GET | PIO_MOV_RXFIFO_BIT |
                      PIO_MOV_RXFIFO_IDX | (index & 0x3U));
}

uint16_t pio_sim_encode_mov_to_rxfifo_y(void)
{
    /* PIO_MOV_RXFIFO_IDX clear → index taken from scratch register Y */
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | PIO_MOV_RXFIFO_BIT);
}

uint16_t pio_sim_encode_mov_from_rxfifo_y(void)
{
    return (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | PIO_MOV_RXFIFO_GET | PIO_MOV_RXFIFO_BIT);
}
#endif /* PIO_SIM_HAS_RXFIFO_MOV */

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

#if PIO_SIM_HAS_WAIT_JMPPIN
uint16_t pio_sim_encode_wait_jmppin(uint8_t polarity, uint8_t index)
{
    return pio_sim_encode_wait(polarity, PIO_WAIT_JMPPIN, index);
}
#endif

uint16_t pio_sim_encode_nop(void)
{
    /* nop == mov y, y */
    return pio_sim_encode_mov(PIO_DST_Y, PIO_MOV_NONE, PIO_SRC_Y);
}
