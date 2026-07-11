/* SPDX-License-Identifier: MIT */
/**
 * @file
 * pio_gpio.h — GPIO pad registers (PADS_BANK0) and per-pin function mux
 * (IO_BANK0) for the PIO simulator.
 *
 * Pads: the digital-relevant fields are simulated exactly — IE (input enable:
 * 0 makes the PIO read the pin as 0), OD (output disable: the pad is never
 * driven by the chip), PUE/PDE pulls (up reads 1 / down reads 0 when undriven),
 * and on RP2350 ISO (isolation: output frozen at the level latched when ISO was
 * raised, input gated). DRIVE strength, SLEWFAST and SCHMITT are analog: they
 * are accepted and stored for read-back but do not affect the digital
 * simulation.
 *
 * Deviation: both pulls enabled is modelled as a bus keeper (holds the last
 * driven level) for a deterministic read. Real RP2040/RP2350 pads have no
 * keeper — both weak resistors on at once form an indeterminate mid-rail
 * divider — so this is a sim-only convenience, not silicon-exact.
 *
 * Mux: each pin's FUNCSEL routes the pad's output/OE. A PIO block only drives
 * pins whose FUNCSEL selects it — owner slot i of a pad set corresponds to
 * FUNC_PIOi (a standalone block is slot 0, i.e. PIO0). Other functions (SIO,
 * UART, PWM…) drive through the explicit peripheral output register
 * (pio_sim_gpio_set_periph_output), which models an on-chip driver, distinct
 * from the off-chip pio_sim_set_pin external drive. Inputs are always visible
 * to the PIO regardless of FUNCSEL, matching silicon. OUTOVER/OEOVER/INOVER
 * apply at the mux boundary exactly as the IO_BANK0 GPIOx_CTRL fields do.
 *
 * Simulator-only deviation: pins reset to PIO_GPIO_FUNC_LEGACY_ANY_PIO, which
 * lets every owning block drive every pin (the historical behaviour, so
 * existing tests and users are unaffected). Hardware-accurate setups call
 * pio_sim_gpio_set_function per pin; pio_sim_pads_reset_hw applies the
 * datasheet pad reset values.
 *
 * All functions route through pio->pads, so they apply to the shared pad set
 * when the block is part of a pio_sim_group.
 */

#ifndef PIO_GPIO_H
#define PIO_GPIO_H

#include "pio_sim.h"

/* Included directly (IWYU): this header uses bool/uint*_t itself. */
#include <stdbool.h>
#include <stdint.h>

/** Per-pin function select (IO_BANK0 GPIOx_CTRL FUNCSEL).
 * Numeric values match the chip's FUNCSEL encoding (RP2040 datasheet
 * Table 289; RP2350 datasheet Table 9-1). Only the PIO functions change the
 * routing behaviour; every other value is "a non-PIO function" whose output
 * comes from the peripheral output register. */
typedef enum {
#if PIO_SIM_HAS_FUNCSEL_PIO2
    PIO_GPIO_FUNC_HSTX = 0, /* RP2350 */
#endif
    PIO_GPIO_FUNC_SPI = 1,
    PIO_GPIO_FUNC_UART = 2,
    PIO_GPIO_FUNC_I2C = 3,
    PIO_GPIO_FUNC_PWM = 4,
    PIO_GPIO_FUNC_SIO = 5,
    PIO_GPIO_FUNC_PIO0 = 6,
    PIO_GPIO_FUNC_PIO1 = 7,
#if PIO_SIM_HAS_FUNCSEL_PIO2
    PIO_GPIO_FUNC_PIO2 = 8, /* RP2350 */
    PIO_GPIO_FUNC_GPCK = 9,
    PIO_GPIO_FUNC_USB = 10,
    PIO_GPIO_FUNC_UART_AUX = 11,
#else
    PIO_GPIO_FUNC_GPCK = 8,
    PIO_GPIO_FUNC_USB = 9,
#endif
    PIO_GPIO_FUNC_NULL = 0x1F,
    /* Simulator-only reset value: every owning PIO block may drive the pin. */
    PIO_GPIO_FUNC_LEGACY_ANY_PIO = 0xFF,
} pio_gpio_func_t;

/** IO_BANK0 GPIOx_CTRL OUTOVER/OEOVER/INOVER encoding. */
typedef enum {
    PIO_GPIO_OVERRIDE_NORMAL = 0,
    PIO_GPIO_OVERRIDE_INVERT = 1,
    PIO_GPIO_OVERRIDE_LOW = 2,
    PIO_GPIO_OVERRIDE_HIGH = 3,
} pio_gpio_override_t;

/** Route pin `pin` (physical GPIO number) to function `f`. A PIO block only
 * drives pins routed to its own FUNC_PIOn; inputs stay visible regardless. */
void pio_sim_gpio_set_function(pio_sim_t *pio, uint8_t pin, pio_gpio_func_t f);
/** Read back the FUNCSEL currently routed to `pin`. */
pio_gpio_func_t pio_sim_gpio_get_function(const pio_sim_t *pio, uint8_t pin);

/** Output/OE/input overrides at the mux boundary (GPIOx_CTRL OUTOVER/OEOVER/
 * INOVER). OEOVER_HIGH can drive a pad even with no function output; INOVER
 * applies before the input synchroniser. */
void pio_sim_gpio_set_outover(pio_sim_t *pio, uint8_t pin, pio_gpio_override_t o);
/** As pio_sim_gpio_set_outover, but for the OEOVER field. */
void pio_sim_gpio_set_oeover(pio_sim_t *pio, uint8_t pin, pio_gpio_override_t o);
/** As pio_sim_gpio_set_outover, but for the INOVER field. */
void pio_sim_gpio_set_inover(pio_sim_t *pio, uint8_t pin, pio_gpio_override_t o);

/** Read back the override configured above for `pin`. */
pio_gpio_override_t pio_sim_gpio_get_outover(const pio_sim_t *pio, uint8_t pin);
/** As pio_sim_gpio_get_outover, but for the OEOVER field. */
pio_gpio_override_t pio_sim_gpio_get_oeover(const pio_sim_t *pio, uint8_t pin);
/** As pio_sim_gpio_get_outover, but for the INOVER field. */
pio_gpio_override_t pio_sim_gpio_get_inover(const pio_sim_t *pio, uint8_t pin);

/** Output of the currently selected non-PIO peripheral (SIO, UART, PWM…) on
 * `pin`: reaches the pad only while the pin's FUNCSEL selects a non-PIO
 * function. Models an on-chip driver — for an off-chip device on the wire use
 * pio_sim_set_pin / pio_sim_release_pin. */
void pio_sim_gpio_set_periph_output(pio_sim_t *pio, uint8_t pin, bool oe, bool level);
/** Read back the peripheral output set above (either pointer may be NULL). */
void pio_sim_gpio_get_periph_output(const pio_sim_t *pio, uint8_t pin, bool *oe, bool *level);

/* ── Pad registers (PADS_BANK0) ────────────────────────────────────────────── */

/** Input enable: while false the PIO logic reads the pin as 0 (the host-side
 * pio_sim_get_pin still returns the true wire level). Reset: enabled. */
void pio_sim_pad_set_input_enable(pio_sim_t *pio, uint8_t pin, bool ie);
/** Read back the input-enable (IE) bit set above for `pin`. */
bool pio_sim_pad_get_input_enable(const pio_sim_t *pio, uint8_t pin);

/** Output disable: while true the chip never drives the pad, whatever the mux
 * or the SMs ask for; external drive and pulls still apply. Reset: off. */
void pio_sim_pad_set_output_disable(pio_sim_t *pio, uint8_t pin, bool od);
/** Read back the output-disable (OD) bit set above for `pin`. */
bool pio_sim_pad_get_output_disable(const pio_sim_t *pio, uint8_t pin);

/** Pull resistors: up only reads 1 when undriven, down only reads 0, neither
 * floats (reads 0). Both enabled is modelled as a bus keeper (holds the last
 * driven level) for determinism — a sim-only convenience, since silicon has no
 * keeper (both resistors on is an indeterminate mid-rail divider). Subsumes
 * pio_sim_set_pull_level. Reset: none. */
void pio_sim_pad_set_pulls(pio_sim_t *pio, uint8_t pin, bool up, bool down);
/** Read back the pull configuration set above (either pointer may be NULL). */
void pio_sim_pad_get_pulls(const pio_sim_t *pio, uint8_t pin, bool *up, bool *down);

/** Analog pad config, stored for read-back only — no digital effect.
 * `drive` is the DRIVE field code (0=2mA, 1=4mA, 2=8mA, 3=12mA). */
void pio_sim_pad_set_drive(pio_sim_t *pio, uint8_t pin, uint8_t drive);
/** Store the SLEWFAST bit for `pin` (analog, read-back only, no digital effect). */
void pio_sim_pad_set_slew_fast(pio_sim_t *pio, uint8_t pin, bool fast);
/** Store the SCHMITT bit for `pin` (analog, read-back only, no digital effect). */
void pio_sim_pad_set_schmitt(pio_sim_t *pio, uint8_t pin, bool enable);
/** Read back the DRIVE field code set above for `pin` (0=2mA…3=12mA). */
uint8_t pio_sim_pad_get_drive(const pio_sim_t *pio, uint8_t pin);
/** Read back the SLEWFAST bit set above for `pin`. */
bool pio_sim_pad_get_slew_fast(const pio_sim_t *pio, uint8_t pin);
/** Read back the SCHMITT bit set above for `pin`. */
bool pio_sim_pad_get_schmitt(const pio_sim_t *pio, uint8_t pin);

#if PIO_SIM_HAS_PAD_ISO
/** RP2350 pad isolation: raising ISO latches the pad's current output drive
 * and freezes it there; the input reads 0 while isolated. Clearing ISO
 * reconnects the live chip drive. Reset (sim): off — pio_sim_pads_reset_hw
 * applies the datasheet's ISO=1. */
void pio_sim_pad_set_iso(pio_sim_t *pio, uint8_t pin, bool iso);
/** Read back the RP2350 pad-isolation latch (see pio_sim_pad_set_iso). */
bool pio_sim_pad_get_iso(const pio_sim_t *pio, uint8_t pin);
#endif

/** Apply the datasheet PADS_BANK0 reset values to every pin: IE=1, OD=0,
 * PUE=0, PDE=1, DRIVE=4mA, SCHMITT=1, SLEWFAST=0 (RP2350: ISO=1). The
 * simulator's own init uses legacy-friendly defaults instead (no pulls,
 * LEGACY_ANY_PIO routing); call this for register-accurate reset state. */
void pio_sim_pads_reset_hw(pio_sim_t *pio);

#endif /* PIO_GPIO_H */
