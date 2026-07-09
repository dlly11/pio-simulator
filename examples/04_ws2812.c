/* 04_ws2812 — generate a WS2812 (NeoPixel) waveform and decode it back.
 *
 * WS2812 LEDs are driven by a single-wire, self-clocked protocol: every bit is
 * a fixed-period pulse whose HIGH time encodes 0 or 1. This embeds the
 * canonical pico-examples `ws2812` program and shows:
 *
 *   - side-set plus per-instruction delays building precise pulse timing,
 *   - autopull (configured on the state machine, not in the program text) so
 *     the 24-bit GRB words stream out without explicit `pull`s,
 *   - decoding the captured waveform by measuring each HIGH run: a long run
 *     (T1+T2) is a 1, a short run (T1) is a 0.
 *
 * Two pixels are sent and the recovered colours are compared to the originals.
 */
#include "pio_asm.h"
#include "pio_sim.h"

#include <stdio.h>

#define WS_PIN 0
#define BITS_PER_PIXEL 24
#define HIGH_ONE_THRESHOLD 4 /* '1' high run is 7, '0' is 2: split at 4 */

/* Canonical WS2812 program. T1/T2/T3 set the pulse shape; a '1' holds high for
 * T1+T2 cycles, a '0' for only T1. */
static const char *const WS_SRC = ".program ws2812\n"
                                  ".side_set 1\n"
                                  ".define T1 2\n"
                                  ".define T2 5\n"
                                  ".define T3 3\n"
                                  ".wrap_target\n"
                                  "bitloop:\n"
                                  "    out x, 1       side 0 [T3 - 1]\n"
                                  "    jmp !x do_zero side 1 [T1 - 1]\n"
                                  "do_one:\n"
                                  "    jmp  bitloop   side 1 [T2 - 1]\n"
                                  "do_zero:\n"
                                  "    nop            side 0 [T2 - 1]\n"
                                  ".wrap\n";

int main(void)
{
    pio_program_t prog;
    if (!pio_asm_assemble(WS_SRC, NULL, &prog)) {
        (void)fprintf(stderr, "assemble failed (line %d): %s\n", prog.error_line, prog.error);
        return 1;
    }

    pio_sim_t pio;
    pio_sim_init(&pio);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_sideset_pins(&c, WS_PIN);
    /* Shift left (MSB first), autopull every 24 bits: one GRB pixel per word. */
    sm_config_set_out_shift(&c, false, true, BITS_PER_PIXEL);
    pio_sim_sm_init(&pio, 0, 0, &c);
    if (!pio_asm_load_program(&pio, 0, 0, &prog)) {
        (void)fprintf(stderr, "load failed\n");
        return 1;
    }
    pio_asm_apply_program_config(&pio, 0, &prog);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, WS_PIN, 1, true);
    pio_sim_sm_set_enabled(&pio, 0, true);

    /* Two 24-bit GRB pixels. The SM shifts MSB-first, so left-align each colour
     * into the top 24 bits of the FIFO word. */
    const uint32_t pixels[] = {0x00FF00U /* green */, 0x1234ABU /* arbitrary */};
    const unsigned npix = (unsigned)(sizeof(pixels) / sizeof(pixels[0]));
    for (unsigned i = 0; i < npix; i++) {
        if (!pio_sim_sm_put(&pio, 0, pixels[i] << 8)) {
            (void)fprintf(stderr, "TX FIFO unexpectedly full\n");
            return 1;
        }
    }
    printf("sending %u pixels: 0x%06X 0x%06X\n", npix, pixels[0], pixels[1]);

    /* Each bit is 10 cycles; capture enough ticks for both pixels plus slack. */
    enum { NSAMP = (10 * BITS_PER_PIXEL * 2) + 40 };
    static uint8_t w[NSAMP];
    for (unsigned i = 0; i < NSAMP; i++) {
        pio_sim_tick(&pio);
        w[i] = pio_sim_get_pin(&pio, WS_PIN) ? 1U : 0U;
    }

    /* One HIGH run per bit; long run => 1, short run => 0. Reconstruct MSB-first. */
    uint32_t recovered[2] = {0, 0};
    unsigned bit = 0;
    unsigned i = 0;
    const unsigned total_bits = BITS_PER_PIXEL * npix;
    while (i < NSAMP && bit < total_bits) {
        if (w[i] == 1U) {
            unsigned len = 0;
            while (i < NSAMP && w[i] == 1U) {
                len++;
                i++;
            }
            unsigned value = (len >= HIGH_ONE_THRESHOLD) ? 1U : 0U;
            unsigned p = bit / BITS_PER_PIXEL;
            recovered[p] = (recovered[p] << 1) | value;
            bit++;
        } else {
            i++;
        }
    }

    printf("recovered %u bits: 0x%06X 0x%06X\n", bit, recovered[0], recovered[1]);
    if (bit != total_bits || recovered[0] != pixels[0] || recovered[1] != pixels[1]) {
        (void)fprintf(stderr, "decode mismatch\n");
        return 1;
    }
    printf("OK: WS2812 waveform generated and decoded\n");
    return 0;
}
