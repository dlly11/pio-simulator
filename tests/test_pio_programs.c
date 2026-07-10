/* test_pio_programs.c — end-to-end validation with real PIO programs.
 *
 * Assembles the canonical pico-examples programs in tests/pio/ (the same
 * sources the encoding differential checks against real pioasm) and runs them
 * through the simulator, asserting observable behaviour: pin waveforms, bit
 * timing, and — for the headline test — a DMA-fed frame through pio_chip. This
 * exercises the assembler, instruction core, pad/side-set model, clock divider
 * and DMA engine together, the way a user actually drives the block. */

#include "unity.h"

#include "pio_asm.h"
#include "pio_chip.h"
#include "pio_dma.h"
#include "pio_sim.h"

#include <stdio.h>
#include <stdlib.h>

#ifndef TESTS_PIO_DIR
#error "TESTS_PIO_DIR must be defined (path to the .pio corpus)"
#endif

static pio_sim_t pio;

void setUp(void) { pio_sim_init(&pio); }
void tearDown(void) { (void)0; }

/* Read tests/pio/<base>.pio into a malloc'd, NUL-terminated buffer. */
static char *read_program(const char *base)
{
    char path[512];
    (void)snprintf(path, sizeof(path), "%s/%s.pio", TESTS_PIO_DIR, base);
    FILE *f = fopen(path, "rb");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, path);
    (void)fseek(f, 0, SEEK_END);
    long n = ftell(f);
    TEST_ASSERT_TRUE(n > 0);
    (void)fseek(f, 0, SEEK_SET);
    size_t len = (size_t)n;
    char *buf = malloc(len + 1U);
    TEST_ASSERT_NOT_NULL(buf);
    size_t got = fread(buf, 1, len, f);
    if (got > len) { /* fread never over-reads, but bound it so the '\0' index is provably in range */
        got = len;
    }
    /* `got` is bounded by `len` above and `buf` holds `len + 1` bytes, so this is
     * in range; the analyzer flags the ftell-derived length as tainted, but the
     * corpus path is a trusted build-time constant, not external input. */
    /* NOLINTNEXTLINE(clang-analyzer-security.ArrayBound) */
    buf[got] = '\0';
    (void)fclose(f);
    return buf;
}

/* Assemble a named program from the corpus, asserting success. */
static void assemble(const char *base, const char *name, pio_program_t *p)
{
    char *src = read_program(base);
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, name, p), p->error);
    free(src);
}

/* Sample GPIO `pin` once per system tick into `out[0..n)`. */
static void sample_pin(pio_sim_t *p, uint8_t pin, uint8_t *out, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        pio_sim_tick(p);
        out[i] = pio_sim_get_pin(p, pin) ? 1U : 0U;
    }
}

/* ── WS2812: bit period encodes the value in the high-time ─────────────────── */

static void test_ws2812_bit_timing_waveform(void)
{
    /* T1=2, T2=5, T3=3: a '1' bit holds the line high for T1+T2 = 7 cycles, a
     * '0' bit for only T1 = 2. Drive a known MSB-first pattern and decode the
     * high-run lengths off the side-set (data) pin. */
    pio_program_t p;
    assemble("ws2812", "ws2812", &p);
    const uint8_t pin = 0;
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 8); /* autopull, MSB first, 8-bit */
    pio_sim_sm_init(&pio, 0, 0, &c);
    TEST_ASSERT_TRUE(pio_asm_load_program(&pio, 0, 0, &p)); /* overlays wrap/side-set/pc */
    pio_asm_apply_program_config(&pio, 0, &p);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, pin, 1, true);
    pio_sim_sm_set_enabled(&pio, 0, true);

    pio_sim_sm_put(&pio, 0, 0xA0U << 24); /* MSB-aligned 0b10100000 → bits 1,0,1,0,0,0,0,0 */

    uint8_t w[120];
    sample_pin(&pio, pin, w, 120);

    /* Measure the length of each contiguous high run. */
    uint8_t runs[16];
    uint8_t nr = 0;
    uint32_t i = 0;
    while ((i < 120U) && (nr < 16U)) {
        if (w[i] == 1U) {
            uint8_t len = 0;
            while ((i < 120U) && (w[i] == 1U)) {
                len++;
                i++;
            }
            runs[nr++] = len;
        } else {
            i++;
        }
    }
    TEST_ASSERT_TRUE(nr >= 3U);
    TEST_ASSERT_EQUAL_UINT8(7U, runs[0]); /* bit 1 → high 7 */
    TEST_ASSERT_EQUAL_UINT8(2U, runs[1]); /* bit 0 → high 2 */
    TEST_ASSERT_EQUAL_UINT8(7U, runs[2]); /* bit 1 → high 7 */
}

/* ── UART TX: sample the line at bit centres and decode the 8n1 frame ───────── */

static void test_uart_tx_frame_decodes(void)
{
    pio_program_t p;
    assemble("uart_tx", "uart_tx", &p);
    const uint8_t pin = 0; /* OUT base and side-set base both = TX pin */
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, true, false, 32); /* LSB first, explicit pull */
    pio_sim_sm_init(&pio, 0, 0, &c);
    TEST_ASSERT_TRUE(pio_asm_load_program(&pio, 0, 0, &p)); /* overlays wrap/side-set/pc */
    pio_asm_apply_program_config(&pio, 0, &p);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, pin, 1, true);
    pio_sim_sm_set_enabled(&pio, 0, true);

    const uint8_t byte = 0x4B; /* 'K' */
    pio_sim_sm_put(&pio, 0, byte);

    /* Frame layout, 8 cycles per bit: [pull idle-high][set start-low][8 data
     * bits LSB-first]. Sample each bit window at its centre (cycle 8*k+4). */
    uint8_t w[8U * 12U];
    sample_pin(&pio, pin, w, sizeof(w));
    TEST_ASSERT_EQUAL_UINT8(1U, w[4]);  /* window 0: idle/stop high */
    TEST_ASSERT_EQUAL_UINT8(0U, w[12]); /* window 1: start bit low  */
    uint8_t got = 0;
    for (uint8_t k = 0; k < 8U; k++) {
        uint8_t bit = w[(8U * (2U + k)) + 4U]; /* windows 2..9 = data LSB-first */
        got |= (uint8_t)(bit << k);
    }
    TEST_ASSERT_EQUAL_HEX8(byte, got);
}

/* ── SPI TX: sample the data pin on the clock-high half, MSB first ─────────── */

static void test_spi_tx_shifts_msb_first(void)
{
    pio_program_t p;
    assemble("spi_tx", "spi_tx", &p);
    const uint8_t data_pin = 0;
    const uint8_t clk_pin = 1;
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, data_pin, 1);
    sm_config_set_sideset_pins(&c, clk_pin);
    sm_config_set_out_shift(&c, false, true, 8); /* autopull, MSB first */
    pio_sim_sm_init(&pio, 0, 0, &c);
    TEST_ASSERT_TRUE(pio_asm_load_program(&pio, 0, 0, &p)); /* overlays wrap/side-set/pc */
    pio_asm_apply_program_config(&pio, 0, &p);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, data_pin, 1, true);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, clk_pin, 1, true);
    pio_sim_sm_set_enabled(&pio, 0, true);

    const uint8_t byte = 0xB3; /* 1011 0011 */
    pio_sim_sm_put(&pio, 0, (uint32_t)byte << 24);

    uint8_t data[64];
    uint8_t clk[64];
    for (uint32_t i = 0; i < 64U; i++) {
        pio_sim_tick(&pio);
        data[i] = pio_sim_get_pin(&pio, data_pin) ? 1U : 0U;
        clk[i] = pio_sim_get_pin(&pio, clk_pin) ? 1U : 0U;
    }
    /* Sample the data pin on each rising clock edge, reconstruct MSB-first. */
    uint8_t got = 0;
    uint8_t bits = 0;
    for (uint32_t i = 1; (i < 64U) && (bits < 8U); i++) {
        if ((clk[i] == 1U) && (clk[i - 1U] == 0U)) {
            got = (uint8_t)((got << 1) | data[i]);
            bits++;
        }
    }
    TEST_ASSERT_EQUAL_UINT8(8U, bits);
    TEST_ASSERT_EQUAL_HEX8(byte, got);
}

/* ── Headline: DMA streams a WS2812 pixel through pio_chip, end to end ──────── */

static pio_chip_t chip;

static void test_chip_dma_feeds_ws2812(void)
{
    /* DMA → PIO0 TX FIFO → WS2812 waveform on a pin, driven by pio_chip_tick,
     * with the elapsed frame time read back from the clock tree. Ties the
     * assembler, DMA engine, pad/side-set model and clock together. */
    pio_chip_init(&chip);
    pio_sim_t *p0 = pio_chip_pio(&chip, 0);

    pio_program_t prog;
    assemble("ws2812", "ws2812", &prog);
    const uint8_t pin = 0;
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 8);
    pio_sim_sm_init(p0, 0, 0, &c);
    TEST_ASSERT_TRUE(pio_asm_load_program(p0, 0, 0, &prog)); /* overlays wrap/side-set/pc */
    pio_asm_apply_program_config(p0, 0, &prog);
    pio_sim_sm_set_consecutive_pindirs(p0, 0, pin, 1, true);
    pio_sim_sm_set_enabled(p0, 0, true);

    static uint32_t pixel[1] = {0x80U << 24}; /* MSB-first 0b10000000 → 1,0,0,0,0,0,0,0 */
    dma_channel_config dc = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&dc, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&chip.dma, 0, &dc, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(pixel), 1,
                              true);

    uint8_t w[120];
    for (uint32_t i = 0; i < 120U; i++) {
        pio_chip_tick(&chip);
        w[i] = pio_sim_get_pin(p0, pin) ? 1U : 0U;
    }
    /* DMA delivered the word (channel drained) and the first bit is a '1' (high
     * run of 7). */
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&chip.dma, 0));
    uint32_t i = 0;
    while ((i < 120U) && (w[i] == 0U)) {
        i++;
    }
    uint8_t first_run = 0;
    while ((i < 120U) && (w[i] == 1U)) {
        first_run++;
        i++;
    }
    TEST_ASSERT_EQUAL_UINT8(7U, first_run);
    /* The frame duration has a defined wall-clock length under the default tree. */
    TEST_ASSERT_TRUE(pio_chip_ticks_to_ns(&chip, 120) > 0U);
    /* The chip-level time conversions delegate to the embedded clock tree
     * (frequency-independent: compare against pio_clk_* on chip.clk directly). */
    TEST_ASSERT_EQUAL_UINT64(pio_clk_ticks_to_us(&chip.clk, 1500000U),
                             pio_chip_ticks_to_us(&chip, 1500000U));
    TEST_ASSERT_EQUAL_UINT64(pio_clk_ns_to_ticks(&chip.clk, 1000000U),
                             pio_chip_ns_to_ticks(&chip, 1000000U));
    TEST_ASSERT_EQUAL_UINT64(pio_clk_us_to_ticks(&chip.clk, 1000U),
                             pio_chip_us_to_ticks(&chip, 1000U));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ws2812_bit_timing_waveform);
    RUN_TEST(test_uart_tx_frame_decodes);
    RUN_TEST(test_spi_tx_shifts_msb_first);
    RUN_TEST(test_chip_dma_feeds_ws2812);
    return UNITY_END();
}
