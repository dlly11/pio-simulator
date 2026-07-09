/* 02_fifo_echo — move data through a state machine with no pins involved.
 *
 * The program loops: pull a word from the TX FIFO, copy it through the shift
 * registers, and push it to the RX FIFO. The host writes words in and reads
 * them back out — a software round-trip that shows the FIFO API without any
 * GPIO or timing concerns:
 *
 *   pio_sim_sm_put            -> host writes the TX FIFO
 *   pio_sim_run_until_rx      -> run until the SM produces a result
 *   pio_sim_sm_get            -> host reads the RX FIFO
 *   pio_sim_sm_get_*_fifo_level / is_*_fifo_empty -> inspect occupancy
 *
 * `pull` blocks until the host supplies a word and `push` blocks until there is
 * room in RX, so the state machine paces itself to the host automatically.
 */
#include "pio_asm.h"
#include "pio_sim.h"

#include <stdio.h>

static const char *const ECHO_SRC = ".program echo\n"
                                    ".wrap_target\n"
                                    "    pull            \n" /* wait for a TX word -> OSR */
                                    "    mov isr, osr    \n" /* copy it into the ISR       */
                                    "    push            \n" /* publish it to the RX FIFO  */
                                    ".wrap\n";

int main(void)
{
    pio_program_t prog;
    if (!pio_asm_assemble(ECHO_SRC, NULL, &prog)) {
        (void)fprintf(stderr, "assemble failed (line %d): %s\n", prog.error_line, prog.error);
        return 1;
    }

    pio_sim_t pio;
    pio_sim_init(&pio);

    /* Defaults are all this program needs: manual pull/push, 32-bit words. */
    pio_sm_config c = pio_get_default_sm_config();
    pio_sim_sm_init(&pio, 0, 0, &c);
    if (!pio_asm_load_program(&pio, 0, 0, &prog)) {
        (void)fprintf(stderr, "load failed\n");
        return 1;
    }
    pio_sim_sm_set_enabled(&pio, 0, true);

    const uint32_t words[] = {0xDEADBEEFU, 0x00C0FFEEU, 0x12345678U};
    const unsigned n = (unsigned)(sizeof(words) / sizeof(words[0]));

    printf("     sent    ->    received\n");
    for (unsigned i = 0; i < n; i++) {
        /* Hand a word to the state machine. */
        if (!pio_sim_sm_put(&pio, 0, words[i])) {
            (void)fprintf(stderr, "TX FIFO unexpectedly full\n");
            return 1;
        }
        printf("  0x%08X (tx level %u)\n", words[i], pio_sim_sm_get_tx_fifo_level(&pio, 0));

        /* Let the SM run until it publishes a result (bounded so a stuck
         * program can't loop forever). */
        (void)pio_sim_run_until_rx(&pio, 0, 1000);
        if (pio_sim_sm_is_rx_fifo_empty(&pio, 0)) {
            (void)fprintf(stderr, "no result produced for word %u\n", i);
            return 1;
        }

        uint32_t got = 0;
        if (!pio_sim_sm_get(&pio, 0, &got)) {
            (void)fprintf(stderr, "RX FIFO unexpectedly empty\n");
            return 1;
        }
        printf("            -> 0x%08X (rx level %u)\n", got, pio_sim_sm_get_rx_fifo_level(&pio, 0));

        if (got != words[i]) {
            (void)fprintf(stderr, "mismatch: sent 0x%08X, got 0x%08X\n", words[i], got);
            return 1;
        }
    }

    printf("OK: %u words echoed through the state machine\n", n);
    return 0;
}
