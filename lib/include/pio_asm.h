/*
 * SPDX-License-Identifier: MIT
 * pio_asm — a minimal PIO assembler for pio_sim.
 *
 * Where no pioasm toolchain is available, this assembles the common subset of
 * PIO assembly directly into instruction words plus the program metadata pio_sim
 * needs (side-set config, wrap points).
 *
 * Comments: ';' and '//' run to end of line; C-style block comments may span lines.
 *
 * Supported syntax (one statement per line):
 *   .program <name>
 *   .define [PUBLIC] <SYMBOL> <int-expr>
 *   .side_set <n> [opt] [pindirs]
 *   .wrap_target
 *   .wrap
 *   .origin <addr>            .pio_version <0|1|rp2040|rp2350>
 *   .clock_div <float>        .fifo <txrx|tx|rx|txput|txget|putget>
 *   .mov_status <txfifo|rxfifo|irq ...> < <n>
 *   .out <count> [left|right] [auto|manual] [<thresh>]   ; pin count + shift cfg
 *   .in  <count> [left|right] [auto|manual] [<thresh>]
 *   .set <count>
 *   .word <int-expr>          ; emit a raw instruction word
 *   .lang_opt <...>           ; parsed and ignored
 *   <label>:                  ; or "public <label>:"
 *   <mnemonic> <operands> [side <n>] [<delay>]   ; delay is "[n]"
 *
 * Mnemonics: jmp, wait, in, out, push, pull, mov, irq, set, nop.
 * Only the common operand forms are accepted; unknown forms return an error with
 * the offending line number.
 *
 * SM-relative IRQ indices are supported: `irq <set|wait|clear> <n> rel` and
 * `wait <pol> irq <n> rel`. On RP2350 (PIO version 1) the indexed RX-FIFO moves
 * `mov rxfifo[<n|y>], isr` / `mov osr, rxfifo[<n|y>]` and `wait <pol> jmppin <n>`
 * are also accepted; on RP2040 (version 0) those forms are rejected. See
 * pio_sim_config.h for the PIO_SIM_PIO_VERSION selector.
 *
 * The assembler is line-based and forgiving about whitespace and case. It handles
 * full integer expressions (pioasm's operators and precedence) over defines and
 * labels, the program-config directives above, .word, and .lang_opt (ignored). It
 * is still not a full pioasm: no code-generation back ends, no C/Python output,
 * and no macro/`%}` verbatim passthrough beyond skipping it.
 */

#ifndef PIO_ASM_H
#define PIO_ASM_H

#include "pio_sim.h"

/* Included directly (IWYU): this header uses bool/uint*_t itself. */
#include <stdbool.h>
#include <stdint.h>

#define PIO_ASM_MAX_INSNS 32U
#define PIO_ASM_ERR_LEN 128U
#define PIO_ASM_MAX_PUBLIC 8U

/* A `public <name>:` label and its instruction index (relative to program
 * start) — mirrors pioasm's `<prog>_offset_<name>` defines. */
typedef struct {
    char name[32];
    uint8_t index;
} pio_public_label_t;

typedef struct {
    uint16_t insns[PIO_ASM_MAX_INSNS];
    uint8_t count;

    /* Side-set config parsed from .side_set */
    uint8_t sideset_bits; /* data bits + (opt ? 1 : 0); 0 if no .side_set */
    bool sideset_opt;
    bool sideset_pindirs;

    /* Wrap targets (instruction indices, relative to program start). */
    uint8_t wrap_bottom;
    uint8_t wrap_top;

    /* Public labels (`public <name>:`), exposed as offsets. */
    pio_public_label_t public_labels[PIO_ASM_MAX_PUBLIC];
    uint8_t public_count;

    /* Program-config metadata (from .program / .origin / .pio_version /
     * .clock_div / .fifo / .mov_status / .in / .out / .set). The `has_*` flags
     * say whether the directive appeared; pio_asm_apply_program_config applies
     * the set ones to a state machine. Storage is version-neutral. */
    char name[32];
    bool has_origin;
    uint8_t origin;     /* .origin: absolute load address */
    int pio_version;    /* .pio_version: 0 or 1; -1 if unset */
    bool has_clock_div; /* .clock_div */
    double clock_div;
    bool has_fifo_join; /* .fifo */
    pio_fifo_join_t fifo_join;
    bool has_mov_status; /* .mov_status */
    pio_status_sel_t mov_status_sel;
    uint8_t mov_status_n;
    /* .in / .out / .set pin/shift defaults. */
    struct {
        bool set;
        uint8_t count;
        pio_shift_dir_t dir;
        bool autoshift;
        uint8_t threshold;
    } in_cfg, out_cfg, set_cfg;

    bool ok;
    int error_line;              /* 1-based; 0 if none */
    char error[PIO_ASM_ERR_LEN]; /* human-readable message */
} pio_program_t;

/**
 * Assemble a single .program from `src` (the whole .pio file is fine; the
 * first .program is used, or `name` selects one when non-NULL).
 *
 * Returns true on success; on failure `out->ok` is false and error/error_line
 * are populated.
 */
bool pio_asm_assemble(const char *src, const char *name, pio_program_t *out);

/**
 * Convenience: load an already-assembled `prog` into `pio` at `offset`
 * (relocating JMP targets), apply its side-set and wrap config to state
 * machine `sm`, and set the SM's PC to `offset` — the program's first
 * instruction, as the SDK's pio_sm_init does, so a preamble before
 * .wrap_target runs once. Returns false and does nothing if prog->ok is
 * false.
 */
bool pio_asm_load_program(pio_sim_t *pio, uint8_t sm, uint8_t offset, const pio_program_t *prog);

/**
 * Like pio_asm_load_program, but load at the program's `.origin` when one was
 * given (`prog->has_origin`), else at offset 0 — mirroring how a fixed-origin
 * program is placed. Equivalent to pio_asm_load_program with the resolved offset.
 */
bool pio_asm_load_program_at_origin(pio_sim_t *pio, uint8_t sm, const pio_program_t *prog);

/**
 * Apply the program-config metadata captured from directives (.clock_div,
 * .fifo, .mov_status, .in/.out/.set) to state machine `sm`, mirroring the
 * sm_config_set_* calls the C-SDK output would emit. Only directives that were
 * present take effect. `pio_asm_load_program` does not call this; invoke it
 * separately when you want the directive-driven configuration applied.
 */
void pio_asm_apply_program_config(pio_sim_t *pio, uint8_t sm, const pio_program_t *prog);

#endif /* PIO_ASM_H */
