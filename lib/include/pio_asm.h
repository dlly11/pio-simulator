/*
 * SPDX-License-Identifier: MIT
 * pio_asm — a minimal PIO assembler for pio_sim.
 *
 * Where no pioasm toolchain is available, this assembles the common subset of
 * PIO assembly directly into instruction words plus the program metadata pio_sim
 * needs (side-set config, wrap points).
 *
 * Supported syntax (one statement per line; ';' starts a comment):
 *   .program <name>
 *   .define [PUBLIC] <SYMBOL> <int-expr>
 *   .side_set <n> [opt] [pindirs]
 *   .wrap_target
 *   .wrap
 *   <label>:
 *   <mnemonic> <operands> [side <n>] [<delay>]   ; delay is "[n]"
 *
 * Mnemonics: jmp, wait, in, out, push, pull, mov, irq, set, nop.
 * Only the common operand forms are accepted; unknown forms return an error with
 * the offending line number.
 *
 * SM-relative IRQ indices are supported: `irq <set|wait|clear> <n> rel` and
 * `wait <pol> irq <n> rel`. The RP2350 indexed RX-FIFO moves are supported:
 * `mov rxfifo[<n|y>], isr` and `mov osr, rxfifo[<n|y>]`.
 *
 * The assembler is line-based and forgiving about whitespace and case, but it
 * is not a full pioasm: it does not implement expressions beyond integer and
 * defined-symbol substitution, .lang_opt, or .word.
 */

#ifndef PIO_ASM_H
#define PIO_ASM_H

#include "pio_sim.h"

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
 * Convenience: assemble `prog` into `pio` at `offset`, applying side-set and
 * wrap config to state machine `sm`. Returns false (and leaves pio untouched
 * beyond the load) if prog->ok is false.
 */
bool pio_asm_load_program(pio_sim_t *pio, uint8_t sm, uint8_t offset, const pio_program_t *prog);

#endif /* PIO_ASM_H */
