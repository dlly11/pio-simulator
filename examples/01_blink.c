/* 01_blink — the "hello world" of the PIO simulator.
 *
 * A two-instruction program toggles one pin, so GPIO 0 emits a square wave.
 * This is the smallest complete use of the library and shows the canonical
 * setup path end to end:
 *
 *   assemble  -> pio_asm_assemble
 *   init      -> pio_sim_init
 *   configure -> pio_get_default_sm_config + sm_config_set_* + pio_sim_sm_init
 *   load      -> pio_asm_load_program (overlays .wrap onto the state machine)
 *   drive pin -> pio_sim_sm_set_consecutive_pindirs (mark the pin an output)
 *   run       -> pio_sim_sm_set_enabled + pio_sim_tick
 *   observe   -> pio_sim_get_pin
 *
 * Each `set` carries a one-cycle delay ([1]), so the pin is high for two
 * system cycles and low for two: a period-4 square wave.
 */
#include "pio_asm.h"
#include "pio_sim.h"

#include <stdio.h>

/* The pin the program drives. The state machine's SET group is based here. */
#define BLINK_PIN 0

static const char *const BLINK_SRC = ".program blink\n"
                                     ".wrap_target\n"
                                     "    set pins, 1 [1]\n" /* pin high, 2 cycles */
                                     "    set pins, 0 [1]\n" /* pin low, 2 cycles  */
                                     ".wrap\n";

int main(void)
{
    /* 1. Assemble the program text into instruction words + metadata. */
    pio_program_t prog;
    if (!pio_asm_assemble(BLINK_SRC, NULL, &prog)) {
        (void)fprintf(stderr, "assemble failed (line %d): %s\n", prog.error_line, prog.error);
        return 1;
    }

    /* 2. Reset a PIO block. */
    pio_sim_t pio;
    pio_sim_init(&pio);

    /* 3. Build the state-machine config the pico-sdk way: start from the reset
     *    defaults and set only what this program needs — the SET pin group. */
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_set_pins(&c, BLINK_PIN, 1);
    pio_sim_sm_init(&pio, 0, 0, &c);

    /* 4. Load the words at offset 0; this overlays the program's .wrap onto
     *    the state machine so it loops the two instructions cleanly. */
    if (!pio_asm_load_program(&pio, 0, 0, &prog)) {
        (void)fprintf(stderr, "load failed\n");
        return 1;
    }

    /* 5. Mark the pin an output (the SM can only drive pins whose direction is
     *    output) and start the state machine. */
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, BLINK_PIN, 1, true);
    pio_sim_sm_set_enabled(&pio, 0, true);

    /* 6. Tick the clock and watch the pin. Print two full periods. */
    printf("cycle : GPIO%d\n", BLINK_PIN);
    uint8_t level[8];
    for (int cycle = 0; cycle < 8; cycle++) {
        pio_sim_tick(&pio);
        level[cycle] = pio_sim_get_pin(&pio, BLINK_PIN) ? 1U : 0U;
        printf("  %2d   :  %d\n", cycle, level[cycle]);
    }

    /* Sanity check so CI's "run the example" step catches a regression. Each set
     * carries a [1] delay, so the pin holds each level for two cycles: the exact
     * waveform is 1,1,0,0 repeated — a total count alone would pass many wrong
     * shapes (e.g. 1,0,1,0 or an inverted phase). */
    static const uint8_t expect[8] = {1, 1, 0, 0, 1, 1, 0, 0};
    for (int cycle = 0; cycle < 8; cycle++) {
        if (level[cycle] != expect[cycle]) {
            (void)fprintf(stderr, "unexpected waveform at cycle %d: got %u, expected %u\n", cycle,
                          level[cycle], expect[cycle]);
            return 1;
        }
    }
    printf("OK: period-4 square wave (1,1,0,0 repeated) over 8 cycles\n");
    return 0;
}
