/*
 * SPDX-License-Identifier: MIT
 * pio_clock.c — clk_sys clock-tree model (see pio_clock.h).
 */

#include "pio_clock.h"

#include <string.h>

/* clk_sys fractional divider width: RP2040 CLK_SYS_DIV is 24.8, RP2350 is
 * 16.16 (RP2040 datasheet §2.15.7 / RP2350 datasheet §8.1.6). */
#if PIO_SIM_PIO_VERSION >= 1
#define CLK_DIV_FRAC_BITS 16U
#define CLK_DIV_INT_MAX 0xFFFFU
#else
#define CLK_DIV_FRAC_BITS 8U
#define CLK_DIV_INT_MAX 0xFFFFFFU
#endif

void pio_clk_init_default(pio_clk_tree_t *t)
{
    (void)memset(t, 0, sizeof(*t));
    t->xosc_hz = 12000000U;
    t->pll_refdiv = 1;
    t->pll_fbdiv = 125; /* VCO = 1500 MHz on a 12 MHz crystal */
#if PIO_SIM_PIO_VERSION >= 1
    t->pll_postdiv1 = 5; /* 1500 / 5 / 2 = 150 MHz (RP2350 boot default) */
#else
    t->pll_postdiv1 = 6; /* 1500 / 6 / 2 = 125 MHz (RP2040 boot default) */
#endif
    t->pll_postdiv2 = 2;
    t->clksys_div_int = 1;
    t->clksys_div_frac = 0;
    t->bypass_pll = false;
}

uint64_t pio_clk_vco_hz(const pio_clk_tree_t *t)
{
    if (t->pll_refdiv == 0U) {
        return 0;
    }
    return (t->xosc_hz / t->pll_refdiv) * t->pll_fbdiv;
}

pio_clk_err_t pio_clk_validate(const pio_clk_tree_t *t)
{
    if ((t->xosc_hz < 1000000ULL) || (t->xosc_hz > 15000000ULL)) {
        return PIO_CLK_ERR_XOSC_RANGE;
    }
    if ((t->clksys_div_int == 0U) || (t->clksys_div_int > CLK_DIV_INT_MAX) ||
        (t->clksys_div_frac >> CLK_DIV_FRAC_BITS) != 0U) {
        return PIO_CLK_ERR_DIV_RANGE;
    }
    if (t->bypass_pll) {
        return PIO_CLK_OK; /* PLL parameters unused */
    }
    if ((t->pll_refdiv < 1U) || (t->pll_refdiv > 63U) ||
        ((t->xosc_hz / t->pll_refdiv) < 5000000ULL)) {
        return PIO_CLK_ERR_REFDIV_RANGE;
    }
    if ((t->pll_fbdiv < 16U) || (t->pll_fbdiv > 320U)) {
        return PIO_CLK_ERR_FBDIV_RANGE;
    }
    uint64_t vco = pio_clk_vco_hz(t);
    if ((vco < PIO_SIM_PLL_VCO_MIN_HZ) || (vco > PIO_SIM_PLL_VCO_MAX_HZ)) {
        return PIO_CLK_ERR_VCO_RANGE;
    }
    if ((t->pll_postdiv1 < 1U) || (t->pll_postdiv1 > 7U) || (t->pll_postdiv2 < 1U) ||
        (t->pll_postdiv2 > 7U)) {
        return PIO_CLK_ERR_POSTDIV_RANGE;
    }
    return PIO_CLK_OK;
}

pio_clk_err_t pio_clk_configure_pll(pio_clk_tree_t *t, uint16_t refdiv, uint16_t fbdiv,
                                    uint8_t postdiv1, uint8_t postdiv2)
{
    pio_clk_tree_t trial = *t;
    trial.pll_refdiv = refdiv;
    trial.pll_fbdiv = fbdiv;
    trial.pll_postdiv1 = postdiv1;
    trial.pll_postdiv2 = postdiv2;
    trial.bypass_pll = false;
    pio_clk_err_t err = pio_clk_validate(&trial);
    if (err == PIO_CLK_OK) {
        *t = trial;
    }
    return err;
}

uint64_t pio_clk_sys_hz(const pio_clk_tree_t *t)
{
    if (pio_clk_validate(t) != PIO_CLK_OK) {
        return 0;
    }
    uint64_t src = t->bypass_pll
                       ? t->xosc_hz
                       : (pio_clk_vco_hz(t) / t->pll_postdiv1 / t->pll_postdiv2);
    /* Fractional divide, rounded to nearest: src * 2^f / (int<<f | frac).
     * src <= 1.6 GHz and f <= 16, so the numerator fits comfortably in 64
     * bits (< 2^47). */
    uint64_t den = ((uint64_t)t->clksys_div_int << CLK_DIV_FRAC_BITS) | t->clksys_div_frac;
    uint64_t num = src << CLK_DIV_FRAC_BITS;
    return (num + (den / 2U)) / den;
}

/* ticks * scale / hz without overflow: hz < 2^32 and scale <= 1e9, so the
 * remainder product (ticks % hz) * scale stays below 2^63. */
static uint64_t muldiv_round(uint64_t ticks, uint64_t scale, uint64_t hz)
{
    if (hz == 0U) {
        return 0;
    }
    uint64_t whole = (ticks / hz) * scale;
    uint64_t rem = (ticks % hz) * scale;
    return whole + ((rem + (hz / 2U)) / hz);
}

/* ceil(t * hz / scale) without overflow: (t % scale) * hz needs t%scale < 1e9
 * and hz <= 1.6e9 — product < 1.6e18 < 2^63. */
static uint64_t muldiv_ceil(uint64_t time, uint64_t hz, uint64_t scale)
{
    uint64_t whole = (time / scale) * hz;
    uint64_t rem = (time % scale) * hz;
    return whole + ((rem + scale - 1U) / scale);
}

uint64_t pio_clk_ticks_to_ns(const pio_clk_tree_t *t, uint64_t ticks)
{
    return muldiv_round(ticks, 1000000000ULL, pio_clk_sys_hz(t));
}

uint64_t pio_clk_ticks_to_us(const pio_clk_tree_t *t, uint64_t ticks)
{
    return muldiv_round(ticks, 1000000ULL, pio_clk_sys_hz(t));
}

uint64_t pio_clk_ns_to_ticks(const pio_clk_tree_t *t, uint64_t ns)
{
    return muldiv_ceil(ns, pio_clk_sys_hz(t), 1000000000ULL);
}

uint64_t pio_clk_us_to_ticks(const pio_clk_tree_t *t, uint64_t us)
{
    return muldiv_ceil(us, pio_clk_sys_hz(t), 1000000ULL);
}
