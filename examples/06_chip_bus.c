/* 06_chip_bus — the capstone: two PIO blocks talk over shared GPIOs, with DMA
 * feeding one side and draining the other, all driven by the chip model.
 *
 * pio_chip_t composes every PIO block around one shared pad set, a DMA
 * controller and a clock tree. Here:
 *
 *   - PIO0 runs an SPI-style PRODUCER: it clocks bytes out on a data pin with a
 *     side-set clock. A DMA channel feeds its TX FIFO from a memory buffer.
 *   - PIO1 runs a CONSUMER on the SAME shared pins: it waits on the clock edge,
 *     samples the data pin into its ISR, and autopushes recovered bytes to its
 *     RX FIFO. A second DMA channel drains that RX FIFO into memory.
 *   - pio_chip_tick advances both blocks in lockstep plus one DMA bus cycle.
 *   - the clock tree reports how long the exchange took in real nanoseconds.
 *   - a trace hook counts the producer's committed instructions.
 *
 * The received buffer is compared byte-for-byte against the transmitted one, so
 * running this proves the whole stack — assembler, two state machines, shared
 * pad model, DMA (both directions) and clock tree — works together.
 */
#include "pio_asm.h"
#include "pio_chip.h"
#include "pio_dma.h"
#include "pio_sim.h"

#include <stdio.h>

#define DATA_PIN 0
#define CLK_PIN 1
#define PRODUCER_CLKDIV 4 /* slow the bus so the consumer samples with margin */

/* PIO0: present a data bit with the clock low, then raise the clock. */
static const char *const PRODUCER_SRC = ".program spi_tx\n"
                                        ".side_set 1\n"
                                        ".wrap_target\n"
                                        "    out pins, 1 side 0 [1]\n"
                                        "    nop         side 1 [1]\n"
                                        ".wrap\n";

/* PIO1: sample the data pin on each rising clock edge, MSB first. */
static const char *const CONSUMER_SRC = ".program spi_rx\n"
                                        ".define CLK 1\n"
                                        ".wrap_target\n"
                                        "    wait 1 gpio CLK\n" /* rising edge: bit is stable */
                                        "    in pins, 1\n"      /* sample the data pin         */
                                        "    wait 0 gpio CLK\n" /* fall before the next edge   */
                                        ".wrap\n";

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
    pio_program_t prod;
    pio_program_t cons;
    if (!pio_asm_assemble(PRODUCER_SRC, NULL, &prod)) {
        (void)fprintf(stderr, "producer assemble failed (line %d): %s\n", prod.error_line,
                      prod.error);
        return 1;
    }
    if (!pio_asm_assemble(CONSUMER_SRC, NULL, &cons)) {
        (void)fprintf(stderr, "consumer assemble failed (line %d): %s\n", cons.error_line,
                      cons.error);
        return 1;
    }

    /* One chip: shared pads across blocks, DMA attached to all, default clocks. */
    pio_chip_t chip;
    pio_chip_init(&chip);
    pio_sim_t *p0 = pio_chip_pio(&chip, 0); /* producer */
    pio_sim_t *p1 = pio_chip_pio(&chip, 1); /* consumer */

    /* Producer: OUT the data pin, side-set the clock, MSB-first autopull. */
    pio_sm_config pc = pio_get_default_sm_config();
    sm_config_set_out_pins(&pc, DATA_PIN, 1);
    sm_config_set_sideset_pins(&pc, CLK_PIN);
    sm_config_set_out_shift(&pc, false, true, 8);
    sm_config_set_clkdiv(&pc, (float)PRODUCER_CLKDIV);
    pio_sim_sm_init(p0, 0, 0, &pc);
    if (!pio_asm_load_program(p0, 0, 0, &prod)) {
        (void)fprintf(stderr, "producer load failed\n");
        return 1;
    }
    pio_asm_apply_program_config(p0, 0, &prod);
    pio_sim_sm_set_consecutive_pindirs(p0, 0, DATA_PIN, 1, true);
    pio_sim_sm_set_consecutive_pindirs(p0, 0, CLK_PIN, 1, true);

    /* Consumer: IN from the data pin, MSB-first autopush a byte at a time. It
     * only reads the shared pins, so it drives no pindirs. */
    pio_sm_config cc = pio_get_default_sm_config();
    sm_config_set_in_pins(&cc, DATA_PIN);
    sm_config_set_in_shift(&cc, false, true, 8);
    pio_sim_sm_init(p1, 0, 0, &cc);
    if (!pio_asm_load_program(p1, 0, 0, &cons)) {
        (void)fprintf(stderr, "consumer load failed\n");
        return 1;
    }
    pio_asm_apply_program_config(p1, 0, &cons);

    unsigned long prod_insns = 0;
    pio_sim_set_trace(p0, count_insns, &prod_insns);

    pio_sim_sm_set_enabled(p0, 0, true);
    pio_sim_sm_set_enabled(p1, 0, true);

    /* DMA 0: memory -> producer TX FIFO. DMA 1: consumer RX FIFO -> memory. */
    const uint8_t message[] = {0xB3U, 0x5AU, 0xC7U, 0x2DU};
    const unsigned n = (unsigned)(sizeof(message) / sizeof(message[0]));
    uint32_t txbuf[8];
    uint32_t rxbuf[8] = {0};
    for (unsigned i = 0; i < n; i++) {
        txbuf[i] = (uint32_t)message[i] << 24; /* MSB-first, left-aligned */
    }

    dma_channel_config tx = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&tx, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&chip.dma, 0, &tx, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(txbuf), n,
                              true);

    dma_channel_config rx = pio_dma_channel_get_default_config(1);
    channel_config_set_dreq(&rx, PIO_DMA_DREQ_PIO_RX(1, 0));
    /* FIFO -> memory is the mirror of the default: the RX FIFO is a fixed
     * register (no read increment), while the destination buffer advances. */
    channel_config_set_read_increment(&rx, false);
    channel_config_set_write_increment(&rx, true);
    /* The RX words land right-justified (8 bits in the low byte). */
    pio_dma_channel_configure(&chip.dma, 1, &rx, pio_dma_addr_mem(rxbuf), pio_dma_addr_rxf(1, 0), n,
                              true);

    printf("PIO0 -> shared GPIO -> PIO1, %u bytes, clk_sys = %llu Hz\n", n,
           (unsigned long long)pio_clk_sys_hz(&chip.clk));

    /* Run the whole chip until DMA 1 has captured every byte. */
    enum { MAX_TICKS = 4000 };
    unsigned ticks = 0;
    for (; ticks < MAX_TICKS; ticks++) {
        pio_chip_tick(&chip);
        if (!pio_dma_channel_is_busy(&chip.dma, 1)) {
            ticks++;
            break;
        }
    }

    printf("exchange took %u cycles (~%llu ns), producer ran %lu instructions\n", ticks,
           (unsigned long long)pio_chip_ticks_to_ns(&chip, ticks), prod_insns);
    printf("     sent  ->  received\n");
    int ok = 1;
    for (unsigned i = 0; i < n; i++) {
        uint8_t got = (uint8_t)(rxbuf[i] & 0xFFU);
        printf("     0x%02X  ->  0x%02X\n", message[i], got);
        if (got != message[i]) {
            ok = 0;
        }
    }

    if (pio_dma_channel_is_busy(&chip.dma, 1)) {
        (void)fprintf(stderr, "consumer DMA never completed\n");
        return 1;
    }
    if (!ok) {
        (void)fprintf(stderr, "byte mismatch across the bus\n");
        return 1;
    }
    /* The producer commits exactly 2 instructions (out + nop) per bit for every
     * bit of the message, then stalls on autopull once the TX FIFO drains. */
    const unsigned long expect_insns = 2UL * 8UL * n;
    if (prod_insns != expect_insns) {
        (void)fprintf(stderr, "trace mismatch: expected %lu producer instructions, counted %lu\n",
                      expect_insns, prod_insns);
        return 1;
    }
    printf("OK: two PIO blocks exchanged %u bytes over shared GPIO via DMA\n", n);
    return 0;
}
