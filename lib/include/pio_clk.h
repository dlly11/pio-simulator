/*
 * SPDX-License-Identifier: MIT
 * pio_clk.h — clk_sys clock-tree model: XOSC → PLL_SYS → clk_sys divider.
 *
 * Behavioural contract: this module does NOT change how the simulator steps.
 * One pio_sim_tick() remains exactly one clk_sys cycle; the tree only assigns
 * that tick a real-world duration, so tick counts convert to wall-clock time
 * (and back) with the same arithmetic real firmware implies. Configuration is
 * validated against the datasheet PLL limits (RP2040 §2.18 / RP2350 §8.6):
 * FBDIV 16–320, REFDIV 1–63 with FREF ≥ 5 MHz, VCO within the per-chip range
 * (750–1600 MHz on RP2040, 400–1600 MHz on RP2350 — PIO_SIM_PLL_VCO_MIN/MAX),
 * POSTDIV1/2 each 1–7. The clk_sys divider is 24.8 fractional on RP2040 and
 * 16.16 on RP2350. All arithmetic is 64-bit integer — no floating point.
 */

#ifndef PIO_CLK_H
#define PIO_CLK_H

#include "pio_sim_config.h"

#include <stdbool.h>
#include <stdint.h>

/** XOSC → PLL_SYS → clk_sys clock-tree configuration. */
typedef struct {
    uint64_t xosc_hz;         /**< crystal frequency (typically 12 MHz) */
    uint16_t pll_refdiv;      /**< reference divider, 1..63 */
    uint16_t pll_fbdiv;       /**< feedback divider, 16..320 */
    uint8_t pll_postdiv1;     /**< post divider 1, 1..7 */
    uint8_t pll_postdiv2;     /**< post divider 2, 1..7 */
    uint32_t clksys_div_int;  /**< clk_sys integer divider (>= 1) */
    uint32_t clksys_div_frac; /**< fractional part: /256ths (RP2040, 8-bit)
                               * or /65536ths (RP2350, 16-bit) */
    bool bypass_pll;          /**< clk_sys sourced from XOSC directly */
} pio_clk_tree_t;

/** Validation result for a pio_clk_tree_t. */
typedef enum {
    PIO_CLK_OK = 0,            /**< configuration is legal */
    PIO_CLK_ERR_XOSC_RANGE,    /**< crystal outside 1..15 MHz */
    PIO_CLK_ERR_REFDIV_RANGE,  /**< refdiv outside 1..63 or FREF < 5 MHz */
    PIO_CLK_ERR_FBDIV_RANGE,   /**< fbdiv outside 16..320 */
    PIO_CLK_ERR_VCO_RANGE,     /**< VCO outside the per-chip min..1600 MHz */
    PIO_CLK_ERR_POSTDIV_RANGE, /**< postdiv1/2 outside 1..7 */
    PIO_CLK_ERR_DIV_RANGE,     /**< clk_sys divider int 0 or frac too wide */
} pio_clk_err_t;

/** The chip's boot-firmware default tree: 12 MHz crystal, PLL_SYS at
 * 125 MHz (RP2040: 12×125/6/2) or 150 MHz (RP2350: 12×125/5/2), divider 1. */
void pio_clk_init_default(pio_clk_tree_t *t);

/** Validate every field against the datasheet ranges. */
pio_clk_err_t pio_clk_validate(const pio_clk_tree_t *t);

/** Set the PLL parameters, validating the result (the tree is only updated when
 * the whole configuration is legal). `refdiv`/`fbdiv` are `uint16_t` — they hold
 * a hardware register field (refdiv 1..63, fbdiv 16..320) — so a caller value
 * wider than 16 bits is taken modulo 2^16 before validation; an out-of-range
 * value is rejected only once truncated (e.g. 0x10001 aliases to refdiv 1). */
pio_clk_err_t pio_clk_configure_pll(pio_clk_tree_t *t, uint16_t refdiv, uint16_t fbdiv,
                                    uint8_t postdiv1, uint8_t postdiv2);

/** VCO frequency: xosc / refdiv × fbdiv. */
uint64_t pio_clk_vco_hz(const pio_clk_tree_t *t);

/** Resulting clk_sys frequency in integer Hz (fractional divider rounded to
 * nearest). Returns 0 if the tree fails validation. */
uint64_t pio_clk_sys_hz(const pio_clk_tree_t *t);

/** Convert a tick count (clk_sys cycles) to nanoseconds / microseconds,
 * rounded to nearest. Exact 64-bit integer arithmetic. Returns 0 if the clock
 * tree fails validation (pio_clk_sys_hz == 0). */
uint64_t pio_clk_ticks_to_ns(const pio_clk_tree_t *t, uint64_t ticks);
uint64_t pio_clk_ticks_to_us(const pio_clk_tree_t *t, uint64_t ticks);

/** Convert a duration to the number of ticks that covers it (rounded UP, so
 * the returned tick count is never shorter than the requested time — the
 * right direction for timeouts). Returns 0 for an invalid clock tree
 * (indistinguishable from a 0-duration input — validate the tree first). */
uint64_t pio_clk_ns_to_ticks(const pio_clk_tree_t *t, uint64_t ns);
uint64_t pio_clk_us_to_ticks(const pio_clk_tree_t *t, uint64_t us);

#endif /* PIO_CLK_H */
