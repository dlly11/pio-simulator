/*
 * SPDX-License-Identifier: MIT
 * pio_gpio.c — PADS_BANK0 pad registers and IO_BANK0 function mux config.
 *
 * All state lives in the block's pio_pads_t (shared across a group); this
 * file is only the register-style setter/getter surface. Resolution happens
 * in pio_sim.c (pads_live / resolve_pads), which reads these fields.
 */

#include "pio_gpio.h"

#include "pio_sim_internal.h"

/* pad_cfg packing: DRIVE[1:0] | SLEWFAST<<2 | SCHMITT<<3. */
#define PAD_CFG_DRIVE_MASK 0x3U
#define PAD_CFG_SLEWFAST 0x4U
#define PAD_CFG_SCHMITT 0x8U

static uint64_t pin_bit(uint8_t pin) { return (uint64_t)1U << ((uint32_t)pin % PIO_SIM_NUM_PINS); }

static uint8_t pin_idx(uint8_t pin) { return (uint8_t)(pin % PIO_SIM_NUM_PINS); }

static void mask_set(uint64_t *m, uint64_t bit, bool on)
{
    if (on) {
        *m |= bit;
    } else {
        *m &= ~bit;
    }
}

/* ── FUNCSEL routing cache ─────────────────────────────────────────────────── */

void pio_pads_recompute_funcsel(pio_pads_t *p)
{
    for (uint8_t o = 0; o < PIO_SIM_NUM_PIO; o++) {
        p->pio_func_mask[o] = 0;
    }
    p->periph_sel_mask = 0;
    for (uint8_t i = 0; i < PIO_SIM_NUM_PINS; i++) {
        uint64_t bit = (uint64_t)1U << i;
        uint8_t f = p->funcsel[i];
        if (f == (uint8_t)PIO_GPIO_FUNC_LEGACY_ANY_PIO) {
            for (uint8_t o = 0; o < PIO_SIM_NUM_PIO; o++) {
                p->pio_func_mask[o] |= bit;
            }
        } else if ((f >= (uint8_t)PIO_GPIO_FUNC_PIO0) &&
                   (f < (uint8_t)PIO_GPIO_FUNC_PIO0 + PIO_SIM_NUM_PIO)) {
            p->pio_func_mask[f - (uint8_t)PIO_GPIO_FUNC_PIO0] |= bit;
        } else if (f != (uint8_t)PIO_GPIO_FUNC_NULL) {
            p->periph_sel_mask |= bit; /* some other (non-PIO) function */
        } else {
            /* FUNC_NULL: nothing drives the pad through the mux. */
        }
    }
}

void pio_sim_gpio_set_function(pio_sim_t *pio, uint8_t pin, pio_gpio_func_t f)
{
    pio->pads->funcsel[pin_idx(pin)] = (uint8_t)f;
    pio_pads_recompute_funcsel(pio->pads);
}

pio_gpio_func_t pio_sim_gpio_get_function(const pio_sim_t *pio, uint8_t pin)
{
    return (pio_gpio_func_t)pio->pads->funcsel[pin_idx(pin)];
}

static void set_override(uint64_t *inv, uint64_t *low, uint64_t *high, uint64_t bit,
                         pio_gpio_override_t o)
{
    mask_set(inv, bit, o == PIO_GPIO_OVERRIDE_INVERT);
    mask_set(low, bit, o == PIO_GPIO_OVERRIDE_LOW);
    mask_set(high, bit, o == PIO_GPIO_OVERRIDE_HIGH);
}

void pio_sim_gpio_set_outover(pio_sim_t *pio, uint8_t pin, pio_gpio_override_t o)
{
    pio_pads_t *p = pio->pads;
    set_override(&p->outover_inv, &p->outover_low, &p->outover_high, pin_bit(pin), o);
}

void pio_sim_gpio_set_oeover(pio_sim_t *pio, uint8_t pin, pio_gpio_override_t o)
{
    pio_pads_t *p = pio->pads;
    set_override(&p->oeover_inv, &p->oeover_low, &p->oeover_high, pin_bit(pin), o);
}

void pio_sim_gpio_set_inover(pio_sim_t *pio, uint8_t pin, pio_gpio_override_t o)
{
    pio_pads_t *p = pio->pads;
    set_override(&p->inover_inv, &p->inover_low, &p->inover_high, pin_bit(pin), o);
}

void pio_sim_gpio_set_periph_output(pio_sim_t *pio, uint8_t pin, bool oe, bool level)
{
    pio_pads_t *p = pio->pads;
    uint64_t bit = pin_bit(pin);
    mask_set(&p->periph_oe, bit, oe);
    mask_set(&p->periph_level, bit, level);
}

/* ── Pad registers ─────────────────────────────────────────────────────────── */

void pio_sim_pad_set_input_enable(pio_sim_t *pio, uint8_t pin, bool ie)
{
    mask_set(&pio->pads->pad_ie, pin_bit(pin), ie);
}

void pio_sim_pad_set_output_disable(pio_sim_t *pio, uint8_t pin, bool od)
{
    mask_set(&pio->pads->pad_od, pin_bit(pin), od);
}

void pio_sim_pad_set_pulls(pio_sim_t *pio, uint8_t pin, bool up, bool down)
{
    mask_set(&pio->pads->pad_pue, pin_bit(pin), up);
    mask_set(&pio->pads->pad_pde, pin_bit(pin), down);
}

void pio_sim_pad_set_drive(pio_sim_t *pio, uint8_t pin, uint8_t drive)
{
    uint8_t *cfg = &pio->pads->pad_cfg[pin_idx(pin)];
    *cfg = (uint8_t)((*cfg & ~PAD_CFG_DRIVE_MASK) | (drive & PAD_CFG_DRIVE_MASK));
}

void pio_sim_pad_set_slew_fast(pio_sim_t *pio, uint8_t pin, bool fast)
{
    uint8_t *cfg = &pio->pads->pad_cfg[pin_idx(pin)];
    *cfg = fast ? (uint8_t)(*cfg | PAD_CFG_SLEWFAST) : (uint8_t)(*cfg & ~PAD_CFG_SLEWFAST);
}

void pio_sim_pad_set_schmitt(pio_sim_t *pio, uint8_t pin, bool enable)
{
    uint8_t *cfg = &pio->pads->pad_cfg[pin_idx(pin)];
    *cfg = enable ? (uint8_t)(*cfg | PAD_CFG_SCHMITT) : (uint8_t)(*cfg & ~PAD_CFG_SCHMITT);
}

uint8_t pio_sim_pad_get_drive(const pio_sim_t *pio, uint8_t pin)
{
    return pio->pads->pad_cfg[pin_idx(pin)] & PAD_CFG_DRIVE_MASK;
}

bool pio_sim_pad_get_slew_fast(const pio_sim_t *pio, uint8_t pin)
{
    return (pio->pads->pad_cfg[pin_idx(pin)] & PAD_CFG_SLEWFAST) != 0U;
}

bool pio_sim_pad_get_schmitt(const pio_sim_t *pio, uint8_t pin)
{
    return (pio->pads->pad_cfg[pin_idx(pin)] & PAD_CFG_SCHMITT) != 0U;
}

#if PIO_SIM_HAS_PAD_ISO
void pio_sim_pad_set_iso(pio_sim_t *pio, uint8_t pin, bool iso)
{
    pio_pads_t *p = pio->pads;
    uint64_t bit = pin_bit(pin);
    if (iso && ((p->pad_iso & bit) == 0U)) {
        /* Raising ISO: latch the pad's current chip-side drive. Compute it
         * with ISO still clear so the live value is captured. */
        uint64_t oe;
        uint64_t lvl;
        pio_pads_chip_drive(p, &oe, &lvl);
        mask_set(&p->iso_oe, bit, (oe & bit) != 0U);
        mask_set(&p->iso_levels, bit, (lvl & bit) != 0U);
    }
    mask_set(&p->pad_iso, bit, iso);
}
#endif

void pio_sim_pads_reset_hw(pio_sim_t *pio)
{
    pio_pads_t *p = pio->pads;
    p->pad_ie = ~(uint64_t)0;
    p->pad_od = 0;
    p->pad_pue = 0;
    p->pad_pde = ~(uint64_t)0; /* datasheet reset: pull-down enabled */
    p->keep_state = 0;
    for (uint8_t i = 0; i < PIO_SIM_NUM_PINS; i++) {
        p->pad_cfg[i] = (uint8_t)(1U /* DRIVE=4mA */ | PAD_CFG_SCHMITT);
    }
#if PIO_SIM_HAS_PAD_ISO
    /* Datasheet reset holds pads isolated until software releases them; the
     * latched drive is hi-Z. */
    p->pad_iso = ~(uint64_t)0;
    p->iso_oe = 0;
    p->iso_levels = 0;
#endif
}
