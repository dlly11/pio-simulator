/* 03_uart_tx — drive a real UART transmitter and decode the wire.
 *
 * This embeds the canonical pico-examples `uart_tx` program (8n1, one bit every
 * 8 state-machine cycles) and shows a full transmit-and-verify loop:
 *
 *   - side-set (optional) drives the idle/start levels,
 *   - the clock divider sets the baud rate (bit period = 8 * clkdiv cycles); it
 *     is declared in the program with `.clock_div` and applied by
 *     pio_asm_apply_program_config, mirroring how a pioasm C-SDK output would,
 *   - the host queues a byte with pio_sim_sm_put,
 *   - we sample GPIO once per system tick and decode the 8n1 frame by reading
 *     each bit window at its centre,
 *   - a pio_sim_set_trace hook counts the instructions actually committed.
 *
 * The recovered byte is checked against what was sent, so running the example
 * proves the timing end to end.
 */
#include "pio_asm.h"
#include "pio_sim.h"

#include <stdio.h>

#define TX_PIN 0
#define CLKDIV 2 /* integer divider: bit period = 8 * CLKDIV ticks (keep == .clock_div) */
#define BIT_TICKS (8 * CLKDIV)

/* The canonical 8n1 UART TX program. OUT pin and side-set pin are both TX. The
 * clock divider is declared here (.clock_div) rather than in C, so
 * pio_asm_apply_program_config below actually applies it. */
static const char *const UART_SRC =
    ".program uart_tx\n"
    ".side_set 1 opt\n"
    ".clock_div 2\n"              /* == CLKDIV; applied by apply_program_config */
    "    pull       side 1 [7]\n" /* idle/stop high, wait byte */
    "    set x, 7   side 0 [7]\n" /* start bit low, load count  */
    "bitloop:\n"
    "    out pins, 1\n"            /* shift one data bit to TX   */
    "    jmp x-- bitloop   [6]\n"; /* 8 cycles per data bit */

/* Trace hook context: count committed instructions. */
static void count_insns(const pio_sim_t *pio, uint8_t sm, uint8_t pc, uint16_t insn, void *ctx)
{
    (void)pio;
    (void)sm;
    (void)pc;
    (void)insn;
    (*(unsigned long *)ctx)++;
}

int main(void)
{
    pio_program_t prog;
    if (!pio_asm_assemble(UART_SRC, NULL, &prog)) {
        (void)fprintf(stderr, "assemble failed (line %d): %s\n", prog.error_line, prog.error);
        return 1;
    }

    pio_sim_t pio;
    pio_sim_init(&pio);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, TX_PIN, 1);
    sm_config_set_sideset_pins(&c, TX_PIN);
    sm_config_set_out_shift(&c, true, false, 32); /* LSB first, explicit pull */
    /* NB: the clock divider is intentionally NOT set here — it comes from the
     * program's `.clock_div` directive via pio_asm_apply_program_config below. */
    pio_sim_sm_init(&pio, 0, 0, &c);
    if (!pio_asm_load_program(&pio, 0, 0, &prog)) {
        (void)fprintf(stderr, "load failed\n");
        return 1;
    }
    /* Applies the program's directive-derived config — here the .clock_div that
     * sets the baud rate. Without this call the divider would stay at 1. */
    pio_asm_apply_program_config(&pio, 0, &prog);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, TX_PIN, 1, true);

    unsigned long committed = 0;
    pio_sim_set_trace(&pio, count_insns, &committed);
    pio_sim_sm_set_enabled(&pio, 0, true);

    const uint8_t byte = 0x4B; /* 'K' */
    if (!pio_sim_sm_put(&pio, 0, byte)) {
        (void)fprintf(stderr, "TX FIFO unexpectedly full\n");
        return 1;
    }
    printf("sending 0x%02X ('%c') at bit period %d ticks (clkdiv %d)\n", byte, byte, BIT_TICKS,
           CLKDIV);

    /* Sample the line once per tick across 12 bit windows (idle, start, 8 data,
     * and a couple of trailing windows for margin). */
    uint8_t w[BIT_TICKS * 12];
    for (unsigned i = 0; i < sizeof(w); i++) {
        pio_sim_tick(&pio);
        w[i] = pio_sim_get_pin(&pio, TX_PIN) ? 1U : 0U;
    }

    /* Window w's centre is at BIT_TICKS*w + BIT_TICKS/2. Window 0 is idle-high,
     * window 1 is the start bit (low), windows 2..9 are data LSB-first. */
    const unsigned centre = BIT_TICKS / 2U;
    printf("  idle window  = %u (expect 1)\n", w[centre]);
    printf("  start window = %u (expect 0)\n", w[BIT_TICKS + centre]);
    uint8_t got = 0;
    for (unsigned k = 0; k < 8; k++) {
        uint8_t bit = w[(BIT_TICKS * (2U + k)) + centre];
        got |= (uint8_t)(bit << k);
    }
    printf("recovered 0x%02X after %lu instructions\n", got, committed);

    if (got != byte) {
        (void)fprintf(stderr, "decode mismatch: sent 0x%02X, got 0x%02X\n", byte, got);
        return 1;
    }
    /* One frame commits exactly: pull + set + 8 * (out + jmp) = 18 instructions
     * (the trailing pull then stalls on the empty FIFO, committing nothing). */
    const unsigned long expect_insns = 18;
    if (committed != expect_insns) {
        (void)fprintf(stderr, "trace mismatch: expected %lu instructions, counted %lu\n",
                      expect_insns, committed);
        return 1;
    }
    printf("OK: UART frame transmitted and decoded\n");
    return 0;
}
