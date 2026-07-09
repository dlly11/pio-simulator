/* test_pio_dma.c — unit tests for the DMA controller model (pio_dma.h):
 * DREQ pacing, sizes, bswap, ring, chaining, null triggers, IRQs, IRQ_QUIET,
 * abort, sniffer CRCs, pacing timers, arbitration, and mem-to-mem. */

#include "unity.h"

#include "pio_chip.h"
#include "pio_dma.h"
#include "pio_sim.h"

#include "pio_gpio.h"

static pio_sim_t pio;
static pio_dma_t dma;

void setUp(void)
{
    pio_sim_init(&pio);
    pio_sim_t *blocks[] = {&pio};
    pio_dma_init(&dma, blocks, 1);
}

void tearDown(void) { (void)0; }

/* Load a program at offset 0, set wrap to span it, enable SM0. */
static void load_prog(const uint16_t *prog, uint8_t n)
{
    pio_sim_load(&pio, 0, prog, n);
    pio_sim_sm_set_wrap(&pio, 0, 0, (uint8_t)(n - 1U));
    pio_sim_sm_set_enabled(&pio, 0, true);
}

/* TX->OSR->ISR->RX echo, to round-trip DMA data through a state machine. */
static void load_echo_prog(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_mov(PIO_DST_ISR, PIO_MOV_NONE, PIO_SRC_OSR),
        pio_sim_encode_push(false, true),
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 0),
    };
    load_prog(prog, 4);
}

/* One combined system tick: SMs first, then the DMA services the FIFOs. */
static void tick_all(int n)
{
    for (int i = 0; i < n; i++) {
        pio_sim_tick(&pio);
        (void)pio_dma_tick(&dma);
    }
}

/* Feed `in` to SM0 TX on channel 0 and drain RX to `out` on channel 1. */
static void configure_echo_pair(uint32_t *in, uint32_t *out, uint32_t n)
{
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(in), n, true);

    c = pio_dma_channel_get_default_config(1);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_RX(0, 0));
    pio_dma_channel_configure(&dma, 1, &c, pio_dma_addr_mem(out), pio_dma_addr_rxf(0, 0), n, true);
}

static void test_dreq_paced_echo_roundtrip(void)
{
    load_echo_prog();
    uint32_t in[3] = {0x11111111U, 0x22222222U, 0x33333333U};
    uint32_t out[3] = {0};
    configure_echo_pair(in, out, 3);
    tick_all(100);
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 0));
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 1));
    TEST_ASSERT_EQUAL_HEX32(0x11111111U, out[0]);
    TEST_ASSERT_EQUAL_HEX32(0x22222222U, out[1]);
    TEST_ASSERT_EQUAL_HEX32(0x33333333U, out[2]);
}

static void test_tx_dreq_pacing_respects_fifo_depth(void)
{
    /* No SM consuming: a TX-paced channel fills the 4-deep FIFO and stalls. */
    static uint32_t in[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(in), 8, true);
    tick_all(20);
    TEST_ASSERT_TRUE(pio_dma_channel_is_busy(&dma, 0));
    TEST_ASSERT_EQUAL_UINT32(4U, pio_dma_channel_transfer_count(&dma, 0));
    TEST_ASSERT_TRUE(pio_sim_tx_full(&pio, 0));
}

static void test_size8_and_bswap(void)
{
    load_echo_prog();
    static uint8_t in8[4] = {0xAAU, 0xBBU, 0xCCU, 0xDDU};
    static uint8_t out8[4] = {0};
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(in8), 4, true);
    c = pio_dma_channel_get_default_config(1);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_RX(0, 0));
    pio_dma_channel_configure(&dma, 1, &c, pio_dma_addr_mem(out8), pio_dma_addr_rxf(0, 0), 4, true);
    tick_all(100);
    TEST_ASSERT_EQUAL_HEX8(0xAAU, out8[0]);
    TEST_ASSERT_EQUAL_HEX8(0xDDU, out8[3]);

    /* 32-bit byte swap, mem-to-mem. */
    static uint32_t src = 0x11223344U;
    static uint32_t dst = 0;
    c = pio_dma_channel_get_default_config(2);
    channel_config_set_bswap(&c, true);
    channel_config_set_write_increment(&c, true);
    pio_dma_channel_configure(&dma, 2, &c, pio_dma_addr_mem(&dst), pio_dma_addr_mem(&src), 1, true);
    (void)pio_dma_tick(&dma);
    TEST_ASSERT_EQUAL_HEX32(0x44332211U, dst);
}

static void test_ring_wraps_read_side(void)
{
    load_echo_prog();
    /* 2-word (8-byte) ring on the read side: buffer must be 8-aligned. */
    _Alignas(8) static uint32_t in[2] = {0x55U, 0x66U};
    static uint32_t out[4] = {0};
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_ring(&c, false, 3); /* wrap read addr, 2^3 bytes = two words */
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(in), 4, true);
    c = pio_dma_channel_get_default_config(1);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_RX(0, 0));
    pio_dma_channel_configure(&dma, 1, &c, pio_dma_addr_mem(out), pio_dma_addr_rxf(0, 0), 4, true);
    tick_all(120);
    const uint32_t expect[4] = {0x55U, 0x66U, 0x55U, 0x66U};
    for (uint8_t i = 0; i < 4U; i++) {
        TEST_ASSERT_EQUAL_HEX32(expect[i], out[i]);
    }
}

static void test_chain_to_next_channel(void)
{
    load_echo_prog();
    static uint32_t a[2] = {0x11U, 0x22U};
    static uint32_t b[2] = {0x33U, 0x44U};
    static uint32_t out[4] = {0};
    pio_dma_channel_config_t c;
    /* ch0 sends a[], chains to ch2 which sends b[]. ch2 is programmed but not
     * triggered — the chain starts it. */
    c = pio_dma_channel_get_default_config(2);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&dma, 2, &c, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(b), 2, false);
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(0, 0));
    channel_config_set_chain_to(&c, 2);
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(a), 2, true);
    c = pio_dma_channel_get_default_config(1);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_RX(0, 0));
    pio_dma_channel_configure(&dma, 1, &c, pio_dma_addr_mem(out), pio_dma_addr_rxf(0, 0), 4, true);
    tick_all(150);
    const uint32_t expect[4] = {0x11U, 0x22U, 0x33U, 0x44U};
    for (uint8_t i = 0; i < 4U; i++) {
        TEST_ASSERT_EQUAL_HEX32(expect[i], out[i]);
    }
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 2));
}

static uint8_t cb_hits;
static uint8_t cb_last_ch;

static void count_cb(pio_dma_t *d, uint8_t ch, void *ctx)
{
    (void)d;
    (void)ctx;
    cb_hits++;
    cb_last_ch = ch;
}

static void test_irq_on_complete_and_ack(void)
{
    static uint32_t src[2] = {1, 2};
    static uint32_t dst[2] = {0};
    cb_hits = 0;
    pio_dma_channel_set_callback(&dma, 3, count_cb, NULL);
    pio_dma_irqn_set_channel_enabled(&dma, 0, 3, true);
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(3);
    channel_config_set_write_increment(&c, true);
    pio_dma_channel_configure(&dma, 3, &c, pio_dma_addr_mem(dst), pio_dma_addr_mem(src), 2, true);
    tick_all(4);
    TEST_ASSERT_EQUAL_UINT8(1U, cb_hits);
    TEST_ASSERT_EQUAL_UINT8(3U, cb_last_ch);
    TEST_ASSERT_EQUAL_HEX32(1U << 3U, pio_dma_get_intr(&dma));
    TEST_ASSERT_EQUAL_HEX32(1U << 3U, pio_dma_get_ints(&dma, 0));
    TEST_ASSERT_EQUAL_HEX32(0U, pio_dma_get_ints(&dma, 1)); /* not enabled there */
    TEST_ASSERT_TRUE(pio_dma_irqn_get_channel_status(&dma, 0, 3));
    pio_dma_irqn_acknowledge_channel(&dma, 0, 3);
    TEST_ASSERT_EQUAL_HEX32(0U, pio_dma_get_intr(&dma));
    /* INTF forces a line regardless of INTR. */
    pio_dma_irqn_force_channel(&dma, 1, 5, true);
    TEST_ASSERT_EQUAL_HEX32(1U << 5U, pio_dma_get_ints(&dma, 1));
    pio_dma_irqn_force_channel(&dma, 1, 5, false);
    TEST_ASSERT_EQUAL_HEX32(0U, pio_dma_get_ints(&dma, 1));
}

static void test_irq_quiet_defers_to_null_trigger(void)
{
    static uint32_t src[1] = {7};
    static uint32_t dst[1] = {0};
    cb_hits = 0;
    pio_dma_channel_set_callback(&dma, 0, count_cb, NULL);
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_irq_quiet(&c, true);
    channel_config_set_write_increment(&c, true);
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_mem(dst), pio_dma_addr_mem(src), 1, true);
    tick_all(3);
    TEST_ASSERT_EQUAL_HEX32(7U, dst[0]);
    TEST_ASSERT_EQUAL_UINT8(0U, cb_hits); /* quiet: completion raised nothing */
    TEST_ASSERT_EQUAL_HEX32(0U, pio_dma_get_intr(&dma));
    /* A null trigger (reload = 0) fires the deferred IRQ. */
    dma.ch[0].trans_count_reload = 0;
    pio_dma_channel_start_mask(&dma, 1U << 0U);
    TEST_ASSERT_EQUAL_UINT8(1U, cb_hits);
    TEST_ASSERT_EQUAL_HEX32(1U << 0U, pio_dma_get_intr(&dma));
}

static void test_abort_midstream_and_retrigger(void)
{
    load_echo_prog();
    static uint32_t in[4] = {1, 2, 3, 4};
    static uint32_t out[4] = {0};
    configure_echo_pair(in, out, 4);
    tick_all(2); /* partway through: both channels still busy */
    TEST_ASSERT_TRUE(pio_dma_channel_is_busy(&dma, 0));
    pio_dma_channel_abort(&dma, (1U << 0U) | (1U << 1U));
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 0));
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 1));
    TEST_ASSERT_EQUAL_HEX32(0U, pio_dma_get_intr(&dma)); /* abort raises no IRQ */
    /* Retrigger works after an abort (the silicon erratum's observable):
     * reprogram addresses and run to completion. */
    pio_sim_sm_clear_fifos(&pio, 0);
    pio_sim_sm_restart(&pio, 0);
    configure_echo_pair(in, out, 4);
    tick_all(150);
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 1));
    TEST_ASSERT_EQUAL_HEX32(4U, out[3]);
}

static void test_sniffer_crc32_check_value(void)
{
    /* Standard CRC-32 check: "123456789" -> 0xCBF43926, via the reflected
     * update (CALC=CRC32R), seed 0xFFFFFFFF, OUT_INV on readback. */
    static const char msg[] = "123456789";
    static uint8_t dst[9];
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_write_increment(&c, true);
    channel_config_set_sniff_enable(&c, true);
    pio_dma_sniffer_enable(&dma, 0, PIO_DMA_SNIFF_CRC32R, false);
    pio_dma_sniffer_set_output_invert_enabled(&dma, true);
    pio_dma_sniffer_set_data_accumulator(&dma, 0xFFFFFFFFU);
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_mem(dst),
                              pio_dma_addr_mem((void *)(uintptr_t)msg), 9, true);
    tick_all(12);
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926U, pio_dma_sniffer_get_data_accumulator(&dma));
}

static void test_sniffer_crc16_ccitt_check_value(void)
{
    /* CRC-16-CCITT (false): "123456789" -> 0x29B1. MSB-first, seed 0xFFFF. */
    static const char msg[] = "123456789";
    static uint8_t dst[9];
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_write_increment(&c, true);
    channel_config_set_sniff_enable(&c, true);
    pio_dma_sniffer_enable(&dma, 0, PIO_DMA_SNIFF_CRC16, false);
    pio_dma_sniffer_set_data_accumulator(&dma, 0xFFFFU);
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_mem(dst),
                              pio_dma_addr_mem((void *)(uintptr_t)msg), 9, true);
    tick_all(12);
    TEST_ASSERT_EQUAL_HEX32(0x29B1U, pio_dma_sniffer_get_data_accumulator(&dma));
}

static void test_sniffer_sum_and_parity(void)
{
    static uint32_t src[3] = {1, 2, 3};
    static uint32_t dst[3];
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_write_increment(&c, true);
    channel_config_set_sniff_enable(&c, true);
    pio_dma_sniffer_enable(&dma, 0, PIO_DMA_SNIFF_SUM, false);
    pio_dma_sniffer_set_data_accumulator(&dma, 0);
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_mem(dst), pio_dma_addr_mem(src), 3, true);
    tick_all(5);
    TEST_ASSERT_EQUAL_HEX32(6U, pio_dma_sniffer_get_data_accumulator(&dma));

    pio_dma_sniffer_enable(&dma, 0, PIO_DMA_SNIFF_EVEN, false);
    pio_dma_sniffer_set_data_accumulator(&dma, 0);
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_mem(dst), pio_dma_addr_mem(src), 3, true);
    tick_all(5);
    TEST_ASSERT_EQUAL_HEX32(1U ^ 2U ^ 3U,
                            pio_dma_sniffer_get_data_accumulator(&dma)); /* lane XOR */
}

static void test_pacing_timer_quarter_rate(void)
{
    /* Timer 0 at X/Y = 1/4: one transfer every 4 ticks. */
    static uint32_t src[4] = {1, 2, 3, 4};
    static uint32_t dst[4] = {0};
    pio_dma_timer_set_fraction(&dma, 0, 1, 4);
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, PIO_DMA_TREQ_TIMER(0));
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_mem(dst), pio_dma_addr_mem(src), 4, true);
    for (int i = 0; i < 8; i++) {
        (void)pio_dma_tick(&dma);
    }
    TEST_ASSERT_EQUAL_UINT32(2U, pio_dma_channel_transfer_count(&dma, 0)); /* 2 of 4 done */
    for (int i = 0; i < 8; i++) {
        (void)pio_dma_tick(&dma);
    }
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 0));
    TEST_ASSERT_EQUAL_HEX32(4U, dst[3]);
}

static void test_one_transfer_per_tick_and_priority(void)
{
    /* Two unpaced mem-to-mem channels: only one element moves per tick, and
     * the high-priority channel finishes first. */
    static uint32_t src[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    static uint32_t dst_a[4] = {0};
    static uint32_t dst_b[4] = {0};
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_write_increment(&c, true);
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_mem(dst_a), pio_dma_addr_mem(src), 4, true);
    c = pio_dma_channel_get_default_config(1);
    channel_config_set_write_increment(&c, true);
    channel_config_set_high_priority(&c, true);
    pio_dma_channel_configure(&dma, 1, &c, pio_dma_addr_mem(dst_b), pio_dma_addr_mem(&src[4]), 4,
                              true);
    (void)pio_dma_tick(&dma); /* one element total */
    TEST_ASSERT_EQUAL_UINT32(4U + 3U, pio_dma_channel_transfer_count(&dma, 0) +
                                          pio_dma_channel_transfer_count(&dma, 1));
    for (int i = 0; i < 3; i++) {
        (void)pio_dma_tick(&dma);
    }
    /* 4 ticks: the high-priority channel is done; ch0 hasn't moved. */
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 1));
    TEST_ASSERT_EQUAL_UINT32(4U, pio_dma_channel_transfer_count(&dma, 0));
    for (int i = 0; i < 4; i++) {
        (void)pio_dma_tick(&dma);
    }
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 0));
    TEST_ASSERT_EQUAL_HEX32(8U, dst_b[3]);
    TEST_ASSERT_EQUAL_HEX32(4U, dst_a[3]);
}

static void test_channel_count_matches_platform(void)
{
#if PIO_SIM_PIO_VERSION >= 1
    TEST_ASSERT_EQUAL_UINT32(16U, (uint32_t)PIO_SIM_DMA_NUM_CHANNELS);
    TEST_ASSERT_EQUAL_UINT32(4U, (uint32_t)PIO_SIM_DMA_NUM_IRQS);
#else
    TEST_ASSERT_EQUAL_UINT32(12U, (uint32_t)PIO_SIM_DMA_NUM_CHANNELS);
    TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)PIO_SIM_DMA_NUM_IRQS);
#endif
}

static void test_cross_pio_transfer_in_group(void)
{
    /* PIO0 RX -> PIO1 TX on one channel: the controller spans blocks, fixing
     * the old helper's single-block limitation. */
    pio_sim_t p2;
    pio_sim_init(&p2);
    pio_sim_t *blocks[] = {&pio, &p2};
    pio_dma_init(&dma, blocks, 2);

    load_echo_prog(); /* pio echoes TX->RX */
    const uint16_t sink[] = {pio_sim_encode_pull(false, true),
                             pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&p2, 0, sink, 2);
    pio_sim_sm_set_wrap(&p2, 0, 0, 1);
    pio_sim_sm_set_enabled(&p2, 0, true);

    static uint32_t in[2] = {0xAB1U, 0xAB2U};
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(in), 2, true);
    c = pio_dma_channel_get_default_config(1);
    channel_config_set_read_increment(&c, false);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_RX(0, 0));
    pio_dma_channel_configure(&dma, 1, &c, pio_dma_addr_txf(1, 0), pio_dma_addr_rxf(0, 0), 2, true);
    for (int i = 0; i < 100; i++) {
        pio_sim_tick(&pio);
        pio_sim_tick(&p2);
        (void)pio_dma_tick(&dma);
    }
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&dma, 1));
    /* The sink SM pulled both words through PIO1's TX FIFO. */
    TEST_ASSERT_EQUAL_HEX32(0xAB2U, pio_sim_sm_get_osr(&p2, 0));
}

static pio_chip_t chip; /* file scope: too large for the test stack */

static void test_chip_loopback_dma_pin_dma(void)
{
    /* Whole-chip path: DMA -> PIO0 TX -> pin 0 (via FUNCSEL) -> PIO1 input ->
     * PIO1 RX -> DMA, using pio_chip_tick for lockstep, plus a wall-clock
     * conversion from the default tree. */
    pio_chip_init(&chip);
    pio_sim_t *p0 = pio_chip_pio(&chip, 0);
    pio_sim_t *p1 = pio_chip_pio(&chip, 1);

    /* PIO0 SM0: pull a word, drive its LSB onto pin 0 (shift right = LSB first). */
    pio_sim_sm_set_out_shift(p0, 0, PIO_SHIFT_RIGHT, false, 32);
    pio_sim_sm_set_out_pins(p0, 0, 0, 1);
    pio_sim_sm_set_pindirs(p0, 0, 0, 1, true);
    const uint16_t src_prog[] = {pio_sim_encode_pull(false, true),
                                 pio_sim_encode_out(PIO_DST_PINS, 1),
                                 pio_sim_encode_jmp(PIO_COND_ALWAYS, 2)};
    pio_sim_load(p0, 0, src_prog, 3);
    pio_sim_sm_set_wrap(p0, 0, 0, 2);
    pio_sim_sm_set_enabled(p0, 0, true);
    pio_sim_gpio_set_function(p0, 0, PIO_GPIO_FUNC_PIO0); /* pad follows PIO0 */

    /* PIO1 SM0: wait for the rising pin, capture it, push once, park. */
    pio_sim_sm_set_in_base(p1, 0, 0);
    const uint16_t cap_prog[] = {
        pio_sim_encode_wait(1, PIO_WAIT_PIN, 0), pio_sim_encode_in(PIO_SRC_PINS, 1),
        pio_sim_encode_push(false, true), pio_sim_encode_jmp(PIO_COND_ALWAYS, 3)};
    pio_sim_load(p1, 0, cap_prog, 4);
    pio_sim_sm_set_wrap(p1, 0, 0, 3);
    pio_sim_sm_set_enabled(p1, 0, true);

    static uint32_t word_in = 0x1U;
    static uint32_t word_out = 0;
    pio_dma_channel_config_t c;
    c = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(0, 0));
    pio_dma_channel_configure(&chip.dma, 0, &c, pio_dma_addr_txf(0, 0), pio_dma_addr_mem(&word_in),
                              1, true);
    c = pio_dma_channel_get_default_config(1);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_RX(1, 0));
    pio_dma_channel_configure(&chip.dma, 1, &c, pio_dma_addr_mem(&word_out), pio_dma_addr_rxf(1, 0),
                              1, true);

    pio_chip_run(&chip, 40);
    TEST_ASSERT_FALSE(pio_dma_channel_is_busy(&chip.dma, 1));
    TEST_ASSERT_EQUAL_HEX32(0x1U, word_out & 0x1U);
    /* 40 chip ticks at the boot-default clk_sys have a defined duration. */
#if PIO_SIM_PIO_VERSION >= 1
    TEST_ASSERT_EQUAL_UINT64(267ULL, pio_chip_ticks_to_ns(&chip, 40)); /* 150 MHz */
#else
    TEST_ASSERT_EQUAL_UINT64(320ULL, pio_chip_ticks_to_ns(&chip, 40)); /* 125 MHz */
#endif
}

/* Robustness (Part 1): a bogus ring size must not invoke shift UB. */
static void test_ring_size_out_of_range_no_ub(void)
{
    static uint32_t src[4] = {1, 2, 3, 4};
    static uint32_t dst[4] = {0};
    pio_dma_channel_config_t c = pio_dma_channel_get_default_config(0);
    channel_config_set_ring(&c, false, 64); /* > pointer width: guarded, no wrap */
    channel_config_set_write_increment(&c, true);
    pio_dma_channel_configure(&dma, 0, &c, pio_dma_addr_mem(dst), pio_dma_addr_mem(src), 2, true);
    (void)pio_dma_tick(&dma);
    TEST_ASSERT_EQUAL_HEX32(1U, dst[0]); /* survived; behaves as no-wrap */
}

/* Robustness (Part 1): a NULL block slot must not be dereferenced. */
static void test_null_block_slot_no_crash(void)
{
    pio_sim_t p0;
    pio_sim_init(&p0);
    pio_sim_t *blocks[] = {&p0, NULL};
    pio_dma_t d;
    pio_dma_init(&d, blocks, 2);
    static uint32_t src[2] = {7, 8};
    pio_dma_channel_config_t c = pio_dma_channel_get_default_config(0);
    channel_config_set_dreq(&c, PIO_DMA_DREQ_PIO_TX(1, 0)); /* endpoint on the NULL slot */
    pio_dma_channel_configure(&d, 0, &c, pio_dma_addr_txf(1, 0), pio_dma_addr_mem(src), 2, true);
    (void)pio_dma_tick(&d); /* must not crash; channel simply never becomes ready */
    TEST_ASSERT_TRUE(pio_dma_channel_is_busy(&d, 0));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ring_size_out_of_range_no_ub);
    RUN_TEST(test_null_block_slot_no_crash);
    RUN_TEST(test_dreq_paced_echo_roundtrip);
    RUN_TEST(test_tx_dreq_pacing_respects_fifo_depth);
    RUN_TEST(test_size8_and_bswap);
    RUN_TEST(test_ring_wraps_read_side);
    RUN_TEST(test_chain_to_next_channel);
    RUN_TEST(test_irq_on_complete_and_ack);
    RUN_TEST(test_irq_quiet_defers_to_null_trigger);
    RUN_TEST(test_abort_midstream_and_retrigger);
    RUN_TEST(test_sniffer_crc32_check_value);
    RUN_TEST(test_sniffer_crc16_ccitt_check_value);
    RUN_TEST(test_sniffer_sum_and_parity);
    RUN_TEST(test_pacing_timer_quarter_rate);
    RUN_TEST(test_one_transfer_per_tick_and_priority);
    RUN_TEST(test_channel_count_matches_platform);
    RUN_TEST(test_cross_pio_transfer_in_group);
    RUN_TEST(test_chip_loopback_dma_pin_dma);
    return UNITY_END();
}
