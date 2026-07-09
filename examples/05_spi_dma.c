/* 05_spi_dma — stream data out of an SPI-style PIO program using DMA.
 *
 * A memory buffer is pushed through a DMA channel into a state machine's TX
 * FIFO, paced by the PIO's TX data request (DREQ), and the SM clocks each byte
 * out MSB-first on a data pin alongside a side-set clock. This is the first
 * "whole system" example: the assembler, instruction core, pad/side-set model
 * and the DMA engine all run together.
 *
 * API surface shown:
 *   - pio_dma_init / pio_dma_channel_get_default_config
 *   - channel_config_set_dreq (PIO_DMA_DREQ_PIO_TX) to pace on the TX FIFO
 *   - pio_dma_channel_configure (memory -> TX FIFO)
 *   - the documented drive order: pio_sim_tick THEN pio_dma_tick each cycle
 *   - pio_dma_channel_is_busy / pio_dma_channel_get_trans_count progress
 *
 * We sample the data pin on each rising clock edge, reconstruct the bytes, and
 * check them against the buffer DMA streamed out.
 */
#include "pio_asm.h"
#include "pio_dma.h"
#include "pio_sim.h"

#include <stdio.h>

#define DATA_PIN 0
#define CLK_PIN 1

/* Minimal MSB-first shifter: present a data bit with the clock low, then raise
 * the clock while the bit is stable so a receiver can sample it. */
static const char *const SPI_SRC = ".program spi_tx\n"
                                   ".side_set 1\n"
                                   ".wrap_target\n"
                                   "    out pins, 1 side 0 [1]\n" /* data out, clock low  */
                                   "    nop         side 1 [1]\n" /* clock high, data held */
                                   ".wrap\n";

int main(void)
{
    pio_program_t prog;
    if (!pio_asm_assemble(SPI_SRC, NULL, &prog)) {
        (void)fprintf(stderr, "assemble failed (line %d): %s\n", prog.error_line, prog.error);
        return 1;
    }

    pio_sim_t pio;
    pio_sim_init(&pio);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, DATA_PIN, 1);
    sm_config_set_sideset_pins(&c, CLK_PIN);
    sm_config_set_out_shift(&c, false, true, 8); /* MSB first, autopull each byte */
    pio_sim_sm_init(&pio, 0, 0, &c);
    if (!pio_asm_load_program(&pio, 0, 0, &prog)) {
        (void)fprintf(stderr, "load failed\n");
        return 1;
    }
    pio_asm_apply_program_config(&pio, 0, &prog);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, DATA_PIN, 1, true);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, CLK_PIN, 1, true);
    pio_sim_sm_set_enabled(&pio, 0, true);

    /* Attach a DMA controller to this one PIO block. */
    pio_dma_t dma;
    pio_sim_t *blocks[] = {&pio};
    pio_dma_init(&dma, blocks, 1);

    /* Each 32-bit FIFO word feeds one byte out the top (MSB-first, 8-bit
     * autopull), so left-align every source byte into bits 31..24. */
    const uint8_t message[] = {0xB3U, 0x5AU, 0xC7U, 0x01U};
    const unsigned n = (unsigned)(sizeof(message) / sizeof(message[0]));
    uint32_t txbuf[8];
    for (unsigned i = 0; i < n; i++) {
        txbuf[i] = (uint32_t)message[i] << 24;
    }

    dma_channel_config dc = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&dc, PIO_DMA_DREQ_PIO_TX(0, 0)); /* pace on TX FIFO space */
    /* write end = TX FIFO, read end = memory; transfer n words, trigger now. */
    pio_dma_channel_configure(&dma, 0, &dc, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(txbuf), n,
                              true);

    printf("DMA streaming %u bytes:", n);
    for (unsigned i = 0; i < n; i++) {
        printf(" 0x%02X", message[i]);
    }
    printf("\n");

    /* Drive PIO then DMA each system cycle. Stop once DMA has delivered every
     * word and the SM has clocked the last byte out. */
    enum { MAX_TICKS = 400 };
    uint8_t data[MAX_TICKS];
    uint8_t clk[MAX_TICKS];
    unsigned ticks = 0;
    unsigned recovered_bits = 0;
    const unsigned want_bits = n * 8U;
    uint8_t got = 0;
    uint8_t recovered[8];
    unsigned nbytes = 0;

    for (; ticks < MAX_TICKS; ticks++) {
        pio_sim_tick(&pio);
        (void)pio_dma_tick(&dma);
        data[ticks] = pio_sim_get_pin(&pio, DATA_PIN) ? 1U : 0U;
        clk[ticks] = pio_sim_get_pin(&pio, CLK_PIN) ? 1U : 0U;

        /* Sample the data pin on each rising clock edge, MSB first. */
        if (ticks > 0 && clk[ticks] == 1U && clk[ticks - 1U] == 0U && recovered_bits < want_bits) {
            got = (uint8_t)((got << 1) | data[ticks]);
            if (++recovered_bits % 8U == 0U) {
                recovered[nbytes++] = got;
                got = 0;
            }
        }
        if (recovered_bits == want_bits && !pio_dma_channel_is_busy(&dma, 0)) {
            ticks++;
            break;
        }
    }

    printf("recovered %u bytes in %u cycles:", nbytes, ticks);
    for (unsigned i = 0; i < nbytes; i++) {
        printf(" 0x%02X", recovered[i]);
    }
    printf("\n");
    printf("DMA channel busy=%d, remaining=%u\n", (int)pio_dma_channel_is_busy(&dma, 0),
           pio_dma_channel_get_trans_count(&dma, 0));

    if (nbytes != n) {
        (void)fprintf(stderr, "expected %u bytes, got %u\n", n, nbytes);
        return 1;
    }
    for (unsigned i = 0; i < n; i++) {
        if (recovered[i] != message[i]) {
            (void)fprintf(stderr, "byte %u mismatch: sent 0x%02X, got 0x%02X\n", i, message[i],
                          recovered[i]);
            return 1;
        }
    }
    printf("OK: %u bytes streamed by DMA and clocked out over SPI\n", n);
    return 0;
}
