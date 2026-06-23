/* test_pio_sim.c — unit tests for the PIO instruction-set simulator.
 *
 * Exercises every instruction class (JMP/WAIT/IN/OUT/PUSH/PULL/MOV/IRQ/SET)
 * plus side-set, delay, the clock divider, wrap, shift directions, and
 * autopush/autopull, using hand-encoded one- and two-instruction programs. */

#include "unity.h"

#include "pio_sim.h"

static pio_sim_t pio;

void setUp(void) { pio_sim_init(&pio); }
void tearDown(void) { (void)0; }

/* Load a program at offset 0, set wrap to span it, enable SM0. */
static void load_prog(const uint16_t *prog, uint8_t n)
{
    pio_sim_load(&pio, 0, prog, n);
    pio_sim_sm_set_wrap(&pio, 0, 0, (uint8_t)(n - 1U));
    pio_sim_sm_set_enabled(&pio, 0, true);
}

/* ── SET ───────────────────────────────────────────────────────────────────── */

static void test_set_x_and_y(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 21),
        pio_sim_encode_set(PIO_DST_Y, 7),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(21U, pio.sm[0].x);
    TEST_ASSERT_EQUAL_UINT32(7U, pio.sm[0].y);
}

static void test_set_pins_drives_pads(void)
{
    /* SET pins maps to set_base..+count. */
    pio_sim_sm_set_set_pins(&pio, 0, 4, 3);      /* pins 4,5,6 */
    pio_sim_sm_set_pindirs(&pio, 0, 4, 3, true); /* drive them as outputs */
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINS, 0x5)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 4));  /* bit0 = 1 */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 5)); /* bit1 = 0 */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 6));  /* bit2 = 1 */
}

static void test_set_pindirs(void)
{
    pio_sim_sm_set_set_pins(&pio, 0, 8, 2);
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINDIRS, 0x1)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_pin_is_pio_output(&pio, 8));
    TEST_ASSERT_FALSE(pio_sim_pin_is_pio_output(&pio, 9));
}

/* ── OUT / shift directions / autopull ─────────────────────────────────────── */

static void test_out_pins_shift_right(void)
{
    pio_sim_sm_set_out_pins(&pio, 0, 0, 4);
    pio_sim_sm_set_pindirs(&pio, 0, 0, 4, true);
    pio_sim_sm_set_out_shift(&pio, 0, PIO_SHIFT_RIGHT, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_out(PIO_DST_PINS, 4), /* low 4 bits first */
    };
    load_prog(prog, 2);
    pio_sim_tx_push(&pio, 0, 0xA); /* 1010 */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 0)); /* bit0 = 0 */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 1));  /* bit1 = 1 */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 2));
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 3));
}

static void test_out_shift_left_takes_top_bits(void)
{
    pio_sim_sm_set_out_shift(&pio, 0, PIO_SHIFT_LEFT, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true), pio_sim_encode_out(PIO_DST_X, 4), /* top 4 bits of OSR */
    };
    load_prog(prog, 2);
    pio_sim_tx_push(&pio, 0, 0xC000000A);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(0xCU, pio.sm[0].x); /* top nibble */
}

static void test_out_autopull_refills(void)
{
    pio_sim_sm_set_out_shift(&pio, 0, PIO_SHIFT_RIGHT, true, 8);
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_X, 8)};
    load_prog(prog, 1);
    pio_sim_tx_push(&pio, 0, 0x11);
    pio_sim_tx_push(&pio, 0, 0x22);
    pio_sim_run(&pio, 1); /* first OUT autopulls 0x11, outputs low byte */
    TEST_ASSERT_EQUAL_UINT32(0x11U, pio.sm[0].x);
    pio_sim_run(&pio, 1); /* second OUT autopulls 0x22 */
    TEST_ASSERT_EQUAL_UINT32(0x22U, pio.sm[0].x);
}

/* Eager autopull: the OUT that empties the OSR refills it immediately, so the
 * next instruction sees a full OSR (the prefetched word) — not an empty one. */
static void test_out_autopull_eager_refill(void)
{
    pio_sim_sm_set_out_shift(&pio, 0, PIO_SHIFT_RIGHT, true, 32);
    const uint16_t prog[] = {
        pio_sim_encode_out(PIO_DST_NULL, 32), /* discard; triggers refill */
        pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_OSR),
    };
    load_prog(prog, 2);
    pio_sim_tx_push(&pio, 0, 0xAAAAAAAAU);
    pio_sim_tx_push(&pio, 0, 0xBBBBBBBBU);
    pio_sim_run(&pio, 1);                                /* OUT outputs 0xAAAA…, refills 0xBBBB… */
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].osr_count);    /* OSR full again (not empty) */
    TEST_ASSERT_EQUAL_HEX32(0xBBBBBBBBU, pio.sm[0].osr); /* prefetched word visible */
    pio_sim_run(&pio, 1);                                /* mov x, osr reads the new word */
    TEST_ASSERT_EQUAL_HEX32(0xBBBBBBBBU, pio.sm[0].x);
}

/* Under OUT_STICKY, an OUT whose inline-enable bit is 0 *releases* the pins (clears
 * the SM's output enable) so they fall back to the pull / float — distinct from the
 * non-sticky case, which merely holds the previous value. */
static void test_out_sticky_releases_on_inline_disable(void)
{
    pio_sim_sm_set_out_pins(&pio, 0, 0, 1);
    pio_sim_sm_set_pindirs(&pio, 0, 0, 1, true);
    pio_sim_sm_set_out_shift(&pio, 0, PIO_SHIFT_RIGHT, true, 2); /* autopull 2 bits */
    pio_sim_sm_set_out_special(&pio, 0, true, true, 1);          /* sticky + inline en = bit 1 */
    pio_sim_set_pull_level(&pio, (uint64_t)1U << 0, false);      /* pull-down on pin 0 */
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_PINS, 2)};
    load_prog(prog, 1);
    pio_sim_tx_push(&pio, 0, 0x3U); /* 0b11: pin bit0=1, enable bit1=1 -> drive high */
    pio_sim_tx_push(&pio, 0, 0x1U); /* 0b01: pin bit0=1, enable bit1=0 -> release     */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_pin_is_pio_output(&pio, 0));
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 0)); /* driven high */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_FALSE(pio_sim_pin_is_pio_output(&pio, 0)); /* released, no longer an output */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 0));           /* now reads the pull-down */
}

/* Inline OUT enable: bit out_en_sel of the OUT data gates the pin write. */
static void test_out_inline_enable_gates_write(void)
{
    pio_sim_sm_set_out_pins(&pio, 0, 0, 1);
    pio_sim_sm_set_pindirs(&pio, 0, 0, 1, true);
    pio_sim_sm_set_out_shift(&pio, 0, PIO_SHIFT_RIGHT, true, 2); /* autopull every 2 bits */
    pio_sim_sm_set_out_special(&pio, 0, false, true, 1); /* enable = data bit 1, no sticky */
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_PINS, 2)};
    load_prog(prog, 1);
    pio_sim_tx_push(&pio, 0, 0x1U); /* 0b01: enable bit1=0 -> suppressed, pin holds 0 */
    pio_sim_tx_push(&pio, 0, 0x3U); /* 0b11: enable bit1=1 -> driven high            */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 0));          /* suppressed; held low… */
    TEST_ASSERT_TRUE(pio_sim_pin_is_pio_output(&pio, 0)); /* …but still a PIO output */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 0)); /* driven high */
}

/* When two SMs drive one pin on the same cycle the higher-numbered SM wins; with
 * inline enable = 0 that SM yields and the lower-numbered SM's value shows. */
static void test_multi_sm_pin_priority_and_override(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_out(PIO_DST_PINS, 1),
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 1),
    };
    /* Priority: SM0 drives pin 4 high, SM1 drives it low, in lockstep -> SM1 wins. */
    pio_sim_load(&pio, 0, prog, 3);
    for (uint8_t s = 0; s < 2; s++) {
        pio_sim_sm_set_wrap(&pio, s, 0, 2);
        pio_sim_sm_set_out_pins(&pio, s, 4, 1);
        pio_sim_sm_set_pindirs(&pio, s, 4, 1, true);
        pio_sim_sm_set_out_shift(&pio, s, PIO_SHIFT_RIGHT, false, 32);
        pio_sim_sm_set_pc(&pio, s, 0);
    }
    pio_sim_tx_push(&pio, 0, 0xFFFFFFFFU);
    pio_sim_tx_push(&pio, 1, 0x0U);
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 4)); /* SM1 (higher) wins -> low */

    /* Override: SM1 (sticky) yields via inline enable=0, releasing pin 4, so SM0's
     * high shows (enable = bit 1 of a 1-bit OUT, which is always 0). */
    pio_sim_init(&pio);
    pio_sim_load(&pio, 0, prog, 3);
    for (uint8_t s = 0; s < 2; s++) {
        pio_sim_sm_set_wrap(&pio, s, 0, 2);
        pio_sim_sm_set_out_pins(&pio, s, 4, 1);
        pio_sim_sm_set_pindirs(&pio, s, 4, 1, true);
        pio_sim_sm_set_out_shift(&pio, s, PIO_SHIFT_RIGHT, false, 32);
        pio_sim_sm_set_pc(&pio, s, 0);
    }
    pio_sim_sm_set_out_special(&pio, 1, true, true, 1); /* sticky + inline: SM1 releases pin 4 */
    pio_sim_tx_push(&pio, 0, 0xFFFFFFFFU);
    pio_sim_tx_push(&pio, 1, 0xFFFFFFFFU);
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 4)); /* SM1 released -> SM0 high shows */
}

/* Higher-priority SM wins even across cycles: SM1 drives pin 5 low and holds it,
 * SM0 drives it high a cycle later — SM1 still wins (a temporal last-writer model
 * would show high). */
static void test_multi_sm_priority_across_cycles(void)
{
    const uint16_t sm1prog[] = {pio_sim_encode_set(PIO_DST_PINS, 0),
                                pio_sim_encode_jmp(PIO_COND_ALWAYS, 1)};
    const uint16_t sm0prog[] = {pio_sim_encode_nop(), pio_sim_encode_set(PIO_DST_PINS, 1),
                                pio_sim_encode_jmp(PIO_COND_ALWAYS, 12)};
    pio_sim_load(&pio, 0, sm1prog, 2);  /* SM1 at 0..1 */
    pio_sim_load(&pio, 10, sm0prog, 3); /* SM0 at 10..12 */
    for (uint8_t s = 0; s < 2; s++) {
        pio_sim_sm_set_set_pins(&pio, s, 5, 1);
        pio_sim_sm_set_pindirs(&pio, s, 5, 1, true);
    }
    pio_sim_sm_set_wrap(&pio, 1, 1, 1);
    pio_sim_sm_set_pc(&pio, 1, 0);
    pio_sim_sm_set_wrap(&pio, 0, 12, 12);
    pio_sim_sm_set_pc(&pio, 0, 10);
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    pio_sim_run(&pio, 3);                        /* SM1 drove low (cyc1); SM0 drove high (cyc2) */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 5)); /* SM1 priority holds it low */
}

/* True high-impedance: a PIO output released to input (no pull) floats and reads 0,
 * not the value it last drove. */
static void test_pin_floats_when_released(void)
{
    pio_sim_sm_set_set_pins(&pio, 0, 7, 1);
    pio_sim_sm_set_pindirs(&pio, 0, 7, 1, true);
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINS, 1)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 7));   /* driven high */
    pio_sim_sm_set_pindirs(&pio, 0, 7, 1, false); /* release to input, no pull */
    TEST_ASSERT_FALSE(pio_sim_pin_is_pio_output(&pio, 7));
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 7)); /* floats -> 0 (not the held 1) */
}

/* Host set_pin / release_pin: external drive, then float, then pull. */
static void test_host_release_pin(void)
{
    pio_sim_set_pin(&pio, 9, true);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 9));
    pio_sim_release_pin(&pio, 9);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 9)); /* floats -> 0 */
    pio_sim_set_pull_level(&pio, (uint64_t)1U << 9, true);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 9)); /* now reads the pull-up */
}

static void test_out_exec_injects_instruction(void)
{
    /* OUT EXEC takes the OSR value as an instruction and runs it next cycle. */
    pio_sim_sm_set_out_shift(&pio, 0, PIO_SHIFT_RIGHT, false, 32);
    uint16_t injected = pio_sim_encode_set(PIO_DST_Y, 9);
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_out(PIO_DST_OSR, 16), /* dest OSR == EXEC path (7) */
    };
    load_prog(prog, 2);
    pio_sim_tx_push(&pio, 0, injected);
    pio_sim_run(&pio, 3); /* pull, out-exec arms, injected runs */
    TEST_ASSERT_EQUAL_UINT32(9U, pio.sm[0].y);
}

/* ── IN / autopush ─────────────────────────────────────────────────────────── */

static void test_in_pins_shift_left(void)
{
    pio_sim_sm_set_in_base(&pio, 0, 0);
    pio_sim_sm_set_in_shift(&pio, 0, PIO_SHIFT_LEFT, false, 32);
    pio_sim_set_pin(&pio, 0, true);
    pio_sim_set_pin(&pio, 1, true);
    const uint16_t prog[] = {pio_sim_encode_in(PIO_SRC_PINS, 4)};
    load_prog(prog, 1);
    pio_sim_sync_settle(&pio); /* static input: skip the 2-cycle settle */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(0x3U, pio.sm[0].isr);
    TEST_ASSERT_EQUAL_UINT8(4U, pio.sm[0].isr_count);
}

static void test_in_autopush_to_rx(void)
{
    pio_sim_sm_set_in_base(&pio, 0, 0);
    pio_sim_sm_set_in_shift(&pio, 0, PIO_SHIFT_LEFT, true, 8);
    pio_sim_set_pin(&pio, 0, true); /* sample 0x...1 each time */
    const uint16_t prog[] = {pio_sim_encode_in(PIO_SRC_PINS, 8)};
    load_prog(prog, 1);
    pio_sim_sync_settle(&pio); /* static input: skip the 2-cycle settle */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_FALSE(pio_sim_rx_empty(&pio, 0));
    uint32_t w = 0;
    TEST_ASSERT_TRUE(pio_sim_rx_pop(&pio, 0, &w));
    TEST_ASSERT_EQUAL_UINT32(0x01U, w);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].isr_count); /* reset after push */
}

/* ── PUSH / PULL blocking ──────────────────────────────────────────────────── */

static void test_pull_blocks_until_tx(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_set(PIO_DST_X, 5),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 4); /* stalls on pull (empty FIFO) */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].x);
    pio_sim_tx_push(&pio, 0, 0xDEAD);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(0xDEADU, pio.sm[0].osr);
    TEST_ASSERT_EQUAL_UINT32(5U, pio.sm[0].x);
}

static void test_pull_noblock_on_empty_copies_x(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 12), pio_sim_encode_pull(false, false), /* noblock */
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(12U, pio.sm[0].osr); /* OSR <- X on empty noblock */
}

static void test_push_blocks_when_rx_full(void)
{
    pio_sim_sm_set_in_shift(&pio, 0, PIO_SHIFT_LEFT, false, 32);
    /* Fill RX (depth 4) then a 5th push stalls. */
    const uint16_t prog[] = {
        pio_sim_encode_in(PIO_SRC_X, 1),
        pio_sim_encode_push(false, true),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 100);
    TEST_ASSERT_TRUE(pio_sim_rx_full(&pio, 0));
    TEST_ASSERT_EQUAL_UINT8(4U, pio.sm[0].rx.count);
}

/* ── MOV ───────────────────────────────────────────────────────────────────── */

static void test_mov_x_to_y_with_invert(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 0),
        pio_sim_encode_mov(PIO_DST_Y, PIO_MOV_INVERT, PIO_SRC_X),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFU, pio.sm[0].y);
}

static void test_mov_reverse(void)
{
    pio_sim_sm_set_out_shift(&pio, 0, PIO_SHIFT_RIGHT, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_mov(PIO_DST_X, PIO_MOV_REVERSE, PIO_SRC_OSR),
    };
    load_prog(prog, 2);
    pio_sim_tx_push(&pio, 0, 0x00000001U);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_HEX32(0x80000000U, pio.sm[0].x);
}

/* ── JMP conditions ────────────────────────────────────────────────────────── */

static void test_jmp_not_x_taken_when_zero(void)
{
    /* addr 0: jmp !x -> 2 ; addr 1: set y,1 (skipped) ; addr 2: set y,2 */
    const uint16_t prog[] = {
        pio_sim_encode_jmp(PIO_COND_NOTX, 2),
        pio_sim_encode_set(PIO_DST_Y, 1),
        pio_sim_encode_set(PIO_DST_Y, 2),
    };
    load_prog(prog, 3);
    pio_sim_run(&pio, 2); /* x==0 so jump taken, then set y,2 */
    TEST_ASSERT_EQUAL_UINT32(2U, pio.sm[0].y);
}

static void test_jmp_x_dec_loop_counts(void)
{
    /* set x,3 ; loop: jmp x-- loop ; set y,1.  Loop body is just the jmp. */
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 3),
        pio_sim_encode_jmp(PIO_COND_XDEC, 1),
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    load_prog(prog, 3);
    /* x=3: jmp taken (x->2), taken (x->1), taken (x->0), not taken (x-- to
     * 0xFFFFFFFF) → fall through. 1 + 4 jmp cycles + 1 = 6. */
    pio_sim_run(&pio, 6);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, pio.sm[0].x); /* wrapped on final dec */
}

static void test_jmp_pin(void)
{
    pio_sim_sm_set_jmp_pin(&pio, 0, 5);
    const uint16_t prog[] = {
        pio_sim_encode_jmp(PIO_COND_PIN, 2),
        pio_sim_encode_set(PIO_DST_Y, 1),
        pio_sim_encode_set(PIO_DST_Y, 2),
    };
    load_prog(prog, 3);
    pio_sim_set_pin(&pio, 5, true);
    pio_sim_sync_settle(&pio); /* static input: skip the 2-cycle settle */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(2U, pio.sm[0].y);
}

/* ── WAIT ──────────────────────────────────────────────────────────────────── */

static void test_wait_gpio_high(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_wait(1, PIO_WAIT_GPIO, 7),
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 5); /* stalls: pin 7 low */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);
    pio_sim_set_pin(&pio, 7, true);
    pio_sim_run(&pio, 4); /* +2 cycles for the synchroniser, then WAIT + SET */
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y);
}

static void test_wait_irq_clears_flag(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_wait(1, PIO_WAIT_IRQ, 3),
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    load_prog(prog, 2);
    pio.irq |= (1U << 3);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y);
    TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, 3)); /* WAIT 1 IRQ clears it */
}

/* ── IRQ ───────────────────────────────────────────────────────────────────── */

static void test_irq_set_and_clear(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_irq(false, false, 2), /* set irq 2 */
        pio_sim_encode_irq(true, false, 2),  /* clear irq 2 */
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    load_prog(prog, 3);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_irq_get(&pio, 2));
    pio_sim_run(&pio, 1);
    TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, 2));
}

/* ── side-set / delay / clkdiv / wrap ──────────────────────────────────────── */

static void test_sideset_drives_pin(void)
{
    /* 1 side-set bit, no opt, base pin 2. SET x,0 side 1 then SET x,0 side 0. */
    pio_sim_sm_set_sideset(&pio, 0, 1, false, false);
    pio_sim_sm_set_sideset_base(&pio, 0, 2);
    pio_sim_sm_set_pindirs(&pio, 0, 2, 1, true); /* drive the side-set pin */
    uint16_t s1 = (uint16_t)(pio_sim_encode_set(PIO_DST_X, 0) | (1U << 12U)); /* side 1 */
    uint16_t s0 = pio_sim_encode_set(PIO_DST_X, 0);                           /* side 0 */
    const uint16_t prog[] = {s1, s0};
    load_prog(prog, 2);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 2));
    pio_sim_run(&pio, 1);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 2));
}

static void test_delay_holds_pc(void)
{
    /* SET x,1 [3] then SET y,1: the delay makes the SM idle 3 cycles. */
    uint16_t set_x_delay = (uint16_t)(pio_sim_encode_set(PIO_DST_X, 1) | (3U << 8U));
    const uint16_t prog[] = {set_x_delay, pio_sim_encode_set(PIO_DST_Y, 1)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 1); /* exec set x (delay armed) */
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].x);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);
    pio_sim_run(&pio, 3); /* 3 delay cycles consumed; set y not reached yet */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);
    pio_sim_run(&pio, 1); /* now set y executes (instruction occupied 1+3 cycles) */
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y);
}

static void test_clkdiv_slows_execution(void)
{
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_X, 1)};
    load_prog(prog, 1);
    pio_sim_sm_set_clkdiv(&pio, 0, 4, 0); /* one SM cycle every 4 ticks */
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].x); /* not yet */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].x); /* 4th tick */
}

/* A fractional divider averages its rate exactly: 2.5 fires on ticks 3,5,8,10,…
 * (a 3,2,3,2 cadence), so the SM advances 2 instructions in 5 ticks and 4 in 10. */
static void test_clkdiv_fractional_cadence(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_nop(), pio_sim_encode_nop(), pio_sim_encode_nop(),
        pio_sim_encode_nop(), pio_sim_encode_nop(), pio_sim_encode_nop(),
    };
    load_prog(prog, 6);
    pio_sim_sm_set_clkdiv(&pio, 0, 2, 128); /* 2.5 */
    pio_sim_run(&pio, 5);
    TEST_ASSERT_EQUAL_UINT8(2U, pio_sim_sm_get_pc(&pio, 0));
    pio_sim_run(&pio, 5);
    TEST_ASSERT_EQUAL_UINT8(4U, pio_sim_sm_get_pc(&pio, 0)); /* avg 2.5 over 10 ticks */
}

static void test_wrap_returns_to_bottom(void)
{
    /* Two-instruction loop incrementing y via set; wrap top=1 -> bottom=0. */
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 1),
        pio_sim_encode_set(PIO_DST_Y, 2),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 2); /* pc: 0 -> 1 -> wrap to 0 */
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].pc);
}

/* ── FIFO join ─────────────────────────────────────────────────────────────── */

static void test_fifo_join_rx_depth_8(void)
{
    pio_sim_sm_set_fifo_join(&pio, 0, PIO_FIFO_JOIN_RX);
    pio_sim_sm_set_in_shift(&pio, 0, PIO_SHIFT_LEFT, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_in(PIO_SRC_X, 1),
        pio_sim_encode_push(false, true),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 200);
    TEST_ASSERT_EQUAL_UINT8(8U, pio.sm[0].rx.count); /* 8-deep when joined */
    TEST_ASSERT_TRUE(pio_sim_tx_full(&pio, 0));      /* TX cap 0 → always full */
}

/* ── Public accessor coverage ──────────────────────────────────────────────── */

static void test_tx_empty_and_irq_clear_accessors(void)
{
    /* TX FIFO empty/non-empty transitions. */
    TEST_ASSERT_TRUE(pio_sim_tx_empty(&pio, 0));
    TEST_ASSERT_TRUE(pio_sim_tx_push(&pio, 0, 0x1234U));
    TEST_ASSERT_FALSE(pio_sim_tx_empty(&pio, 0));

    /* IRQ set via the raw flag, cleared through the public accessor. */
    pio.irq |= (uint8_t)(1U << 5U);
    TEST_ASSERT_TRUE(pio_sim_irq_get(&pio, 5));
    pio_sim_irq_clear(&pio, 5);
    TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, 5));
}

/* ── Regression tests for review findings #1–#4 ────────────────────────────── */

/* #1: an IN that crosses the autopush threshold while the RX FIFO is full must
 * stall *before* shifting — it must not shift the same source bits into the ISR
 * again on each retry. */
static void test_in_autopush_full_does_not_reshift(void)
{
    pio_sim_sm_set_in_base(&pio, 0, 0);
    pio_sim_sm_set_in_shift(&pio, 0, PIO_SHIFT_LEFT, true, 8);
    pio_sim_set_pin(&pio, 0, true); /* each sample = 0x01 */
    const uint16_t prog[] = {pio_sim_encode_in(PIO_SRC_PINS, 8)};
    load_prog(prog, 1);
    pio_sim_sync_settle(&pio); /* static input: skip the 2-cycle settle */

    /* Four pushes fill the 4-deep RX FIFO; the next IN must stall. */
    pio_sim_run(&pio, 4);
    TEST_ASSERT_TRUE(pio_sim_rx_full(&pio, 0));

    /* Many stalled cycles: the ISR must stay empty, not accumulate reshifts. */
    pio_sim_run(&pio, 20);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].isr);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].isr_count);

    /* Freeing a slot lets exactly one fresh sample through, uncorrupted. */
    uint32_t w = 0;
    TEST_ASSERT_TRUE(pio_sim_rx_pop(&pio, 0, &w));
    TEST_ASSERT_EQUAL_UINT32(0x01U, w);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_rx_full(&pio, 0));
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].isr_count);
}

/* #2: `irq wait` raises the flag and parks in-instruction until it is cleared;
 * it must not advance the PC on the cycle it raises. */
static void test_irq_wait_parks_until_cleared(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_irq(false, true, 0), /* irq 0 ; wait */
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    load_prog(prog, 2);

    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_irq_get(&pio, 0)); /* flag raised */
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].pc);  /* parked on the irq insn */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);

    pio_sim_run(&pio, 10); /* still parked while the flag stays set */
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].pc);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);

    pio_sim_irq_clear(&pio, 0); /* external acknowledge */
    pio_sim_run(&pio, 2);       /* irq commits, then set y runs */
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y);
}

/* An `irq ... wait` injected via pio_sim_sm_exec must raise its flag exactly once.
 * Once it has parked, a later external clear lets it commit — it must not re-raise
 * the flag on its retry cycle (which would defeat the acknowledge). */
static void test_sm_exec_irq_wait_does_not_rearm(void)
{
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_Y, 1)};
    load_prog(prog, 1);

    pio_sim_sm_exec(&pio, 0, pio_sim_encode_irq(false, true, 0)); /* irq 0 ; wait */
    TEST_ASSERT_TRUE(pio_sim_irq_get(&pio, 0));                   /* raised once, now parked */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);

    pio_sim_irq_clear(&pio, 0); /* external acknowledge */
    pio_sim_run(&pio, 1);       /* retry must NOT re-raise; the wait commits */
    TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, 0));
    pio_sim_run(&pio, 1); /* program resumes: set y, 1 */
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y);
}

/* pio_sim_sm_get_pc tracks the live PC and round-trips with pio_sim_sm_set_pc. */
static void test_sm_get_pc_reads_back(void)
{
    const uint16_t prog[] = {pio_sim_encode_nop(), pio_sim_encode_nop()};
    load_prog(prog, 2);
    TEST_ASSERT_EQUAL_UINT8(0U, pio_sim_sm_get_pc(&pio, 0)); /* starts at wrap_bottom */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT8(1U, pio_sim_sm_get_pc(&pio, 0)); /* advanced one instruction */
    pio_sim_sm_set_pc(&pio, 0, 7);
    TEST_ASSERT_EQUAL_UINT8(7U, pio_sim_sm_get_pc(&pio, 0)); /* set/get round-trip */
}

/* pio_sim_sm_is_enabled reflects the enable state, set either way. */
static void test_sm_is_enabled_reads_back(void)
{
    TEST_ASSERT_FALSE(pio_sim_sm_is_enabled(&pio, 0)); /* default after init */
    pio_sim_sm_set_enabled(&pio, 0, true);
    TEST_ASSERT_TRUE(pio_sim_sm_is_enabled(&pio, 0));
    pio_sim_set_sm_mask_enabled(&pio, 0x4U, true); /* enable SM2 via the mask form */
    TEST_ASSERT_TRUE(pio_sim_sm_is_enabled(&pio, 2));
    TEST_ASSERT_FALSE(pio_sim_sm_is_enabled(&pio, 1));
}

/* pio_sim_sm_get_instr returns the word at the PC, or a pending injected one. */
static void test_sm_get_instr_reads_current(void)
{
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_X, 5), pio_sim_encode_nop()};
    load_prog(prog, 2);
    TEST_ASSERT_EQUAL_HEX16(prog[0], pio_sim_sm_get_instr(&pio, 0)); /* at pc 0 */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX16(prog[1], pio_sim_sm_get_instr(&pio, 0)); /* advanced to pc 1 */
    pio_sim_sm_exec(&pio, 0, pio_sim_encode_pull(false, true));      /* TX empty: latches pending */
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_pull(false, true), pio_sim_sm_get_instr(&pio, 0));
}

/* pio_sim_sm_is_stalled reflects a blocking instruction that cannot make progress. */
static void test_sm_is_stalled(void)
{
    const uint16_t prog[] = {pio_sim_encode_pull(false, true), pio_sim_encode_set(PIO_DST_Y, 1)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 1); /* pull block on an empty TX FIFO → stalls */
    TEST_ASSERT_TRUE(pio_sim_sm_is_stalled(&pio, 0));
    pio_sim_tx_push(&pio, 0, 0x55U);
    pio_sim_run(&pio, 1); /* now the pull commits */
    TEST_ASSERT_FALSE(pio_sim_sm_is_stalled(&pio, 0));
}

/* #3: an instruction forced via pio_sim_sm_exec executes immediately; its delay
 * field is ignored (so the next program instruction is not held off). */
static void test_sm_exec_ignores_delay(void)
{
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_Y, 1)};
    load_prog(prog, 1);
    uint16_t set_x_delay = (uint16_t)(pio_sim_encode_set(PIO_DST_X, 1) | (3U << 8U));
    pio_sim_sm_exec(&pio, 0, set_x_delay);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].x); /* executed immediately */
    pio_sim_run(&pio, 1);                      /* no delay armed → set y runs now */
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y);
}

/* #4: side-set drives its pins every cycle the instruction is presented,
 * including while the instruction is stalled. */
static void test_sideset_applies_while_stalled(void)
{
    pio_sim_sm_set_sideset(&pio, 0, 1, false, false); /* 1 data bit, no opt */
    pio_sim_sm_set_sideset_base(&pio, 0, 2);
    pio_sim_sm_set_pindirs(&pio, 0, 2, 1, true);
    uint16_t pull_side1 = (uint16_t)(pio_sim_encode_pull(false, true) | (1U << 12U));
    const uint16_t prog[] = {pull_side1};
    load_prog(prog, 1);

    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 2));
    pio_sim_run(&pio, 1); /* TX empty → stalls on pull, but side 1 is driven */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 2));
    pio_sim_run(&pio, 5); /* still stalled, side-set held */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 2));
}

/* ── #5: dynamic MOV STATUS ─────────────────────────────────────────────────── */

static void test_mov_status_tx_level(void)
{
    pio_sim_sm_set_status_sel(&pio, 0, PIO_STATUS_TX_LEVEL, 2);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_STATUS)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, pio.sm[0].x); /* TX level 0 < 2 */
    pio_sim_tx_push(&pio, 0, 0);
    pio_sim_tx_push(&pio, 0, 0);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x); /* TX level 2, not < 2 */
}

static void test_mov_status_rx_level(void)
{
    pio_sim_sm_set_status_sel(&pio, 0, PIO_STATUS_RX_LEVEL, 1);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_STATUS)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, pio.sm[0].x); /* RX empty: 0 < 1 */
    /* Autopush a word into RX, then STATUS must read all-zeros (level 1). */
    pio_sim_sm_set_in_shift(&pio, 0, PIO_SHIFT_LEFT, true, 1);
    pio_sim_sm_exec(&pio, 0, pio_sim_encode_in(PIO_SRC_NULL, 1));
    TEST_ASSERT_FALSE(pio_sim_rx_empty(&pio, 0));
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x);
}

#if PIO_SIM_HAS_IRQ_STATUS
static void test_mov_status_irq_flag(void)
{
    pio_sim_sm_set_status_sel(&pio, 0, PIO_STATUS_IRQ_SET, 3);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_STATUS)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x); /* irq 3 clear */
    pio.irq |= (uint8_t)(1U << 3U);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, pio.sm[0].x);
}
#endif

static void test_mov_status_override(void)
{
    pio_sim_sm_set_status_value(&pio, 0, 0xA5A5A5A5U);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_STATUS)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xA5A5A5A5U, pio.sm[0].x);
}

/* ── #6: SM-relative IRQ indexing ──────────────────────────────────────────── */

static void test_irq_rel_maps_per_sm(void)
{
    /* `irq 0 rel` (set form): SM0 → flag 0, SM1 → flag 1. */
    const uint16_t prog[] = {pio_sim_encode_irq_rel(false, false, 0)};
    pio_sim_load(&pio, 0, prog, 1);
    for (uint8_t s = 0; s < 2U; s++) {
        pio_sim_sm_set_wrap(&pio, s, 0, 0);
        pio_sim_sm_set_enabled(&pio, s, true);
    }
    pio_sim_run(&pio, 4);
    TEST_ASSERT_TRUE(pio_sim_irq_get(&pio, 0));
    TEST_ASSERT_TRUE(pio_sim_irq_get(&pio, 1));
    TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, 2)); /* no SM2/SM3 running */
    TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, 3));
}

static void test_irq_rel_preserves_high_bit(void)
{
    /* `irq 4 rel` on SM3: (4 & 4) | ((0 + 3) & 3) = 4 | 3 = 7. */
    const uint16_t prog[] = {pio_sim_encode_irq_rel(false, false, 4)};
    pio_sim_load(&pio, 0, prog, 1);
    pio_sim_sm_set_wrap(&pio, 3, 0, 0);
    pio_sim_sm_set_enabled(&pio, 3, true);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_TRUE(pio_sim_irq_get(&pio, 7));
    TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, 4));
}

static void test_wait_irq_rel(void)
{
    /* SM1 does `wait 1 irq 0 rel` → waits on flag 1; raising it lets it run. */
    const uint16_t prog[] = {
        pio_sim_encode_wait_irq_rel(1, 0),
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    pio_sim_load(&pio, 0, prog, 2);
    pio_sim_sm_set_wrap(&pio, 1, 0, 1);
    pio_sim_sm_set_enabled(&pio, 1, true);
    pio_sim_run(&pio, 4); /* stalls: flag 1 low */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[1].y);
    pio.irq |= (uint8_t)(1U << 1U);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[1].y);
    TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, 1)); /* WAIT 1 IRQ clears it */
}

/* ── #9: clock-divider phase alignment ─────────────────────────────────────── */

static void test_clkdiv_restart_realigns_sms(void)
{
    /* A self-loop keeps each SM busy; the clk accumulator phase is what's tested.
     * Dividers free-run, so SMs enabled together stay in lockstep until one divider
     * is restarted out of phase. */
    const uint16_t prog[] = {pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&pio, 0, prog, 1);
    for (uint8_t s = 0; s < 2U; s++) {
        pio_sim_sm_set_wrap(&pio, s, 0, 0);
        pio_sim_sm_set_clkdiv(&pio, s, 4, 0); /* one SM cycle every 4 ticks */
    }
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true); /* enable both, dividers aligned */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(pio.sm[0].clk_accum, pio.sm[1].clk_accum); /* lockstep */

    /* Restart only SM0 → its divider phase resets while SM1's keeps running. */
    pio_sim_clkdiv_restart(&pio, 0x1U);
    TEST_ASSERT_TRUE(pio.sm[0].clk_accum != pio.sm[1].clk_accum);

    /* Re-align both: equal phase again, and they step together thereafter. */
    pio_sim_clkdiv_restart(&pio, 0x3U);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].clk_accum);
    TEST_ASSERT_EQUAL_UINT32(pio.sm[0].clk_accum, pio.sm[1].clk_accum);
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_UINT32(768U, pio.sm[0].clk_accum); /* 3 × 256, no fire yet */
    TEST_ASSERT_EQUAL_UINT32(pio.sm[0].clk_accum, pio.sm[1].clk_accum);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].clk_accum); /* both fired on this tick */
    TEST_ASSERT_EQUAL_UINT32(pio.sm[0].clk_accum, pio.sm[1].clk_accum);
}

/* The clock divider free-runs while the SM is disabled: the accumulator advances
 * but no instruction executes. */
static void test_clkdiv_freeruns_while_disabled(void)
{
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_X, 1)};
    pio_sim_load(&pio, 0, prog, 1);
    pio_sim_sm_set_wrap(&pio, 0, 0, 0);
    pio_sim_sm_set_clkdiv(&pio, 0, 4, 0); /* SM left disabled */
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_UINT32(768U, pio.sm[0].clk_accum);     /* divider free-ran */
    TEST_ASSERT_EQUAL_UINT8(0U, pio_sim_sm_get_pc(&pio, 0)); /* but did not execute */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].x);
}

/* ── #7: RP2350 instructions ───────────────────────────────────────────────── */

#if PIO_SIM_HAS_RXFIFO_MOV
static void test_mov_rxfifo_roundtrip(void)
{
    pio_sim_sm_set_fifo_join(&pio, 0, PIO_FIFO_JOIN_RX_PUT);
    const uint16_t prog[] = {
        pio_sim_encode_mov_to_rxfifo(2),
        pio_sim_encode_mov_from_rxfifo(2),
    };
    load_prog(prog, 2);
    pio.sm[0].isr = 0xDEADBEEFU;
    pio.sm[0].isr_count = 32;
    pio_sim_run(&pio, 1); /* rxfifo[2] <- ISR */
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFU, pio.sm[0].rx.buf[2]);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].isr);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].isr_count);
    pio_sim_run(&pio, 1); /* OSR <- rxfifo[2] */
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFU, pio.sm[0].osr);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].osr_count);
}

static void test_mov_rxfifo_y_indexed(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_Y, 3),
        pio_sim_encode_mov_to_rxfifo_y(),
    };
    load_prog(prog, 2);
    pio.sm[0].isr = 0x12345678U;
    pio_sim_run(&pio, 2); /* set y,3 ; rxfifo[y=3] <- ISR */
    TEST_ASSERT_EQUAL_HEX32(0x12345678U, pio.sm[0].rx.buf[3]);
}

/* The host reaches the RX register file by index in PUT/GET mode: it reads what
 * the SM PUT, and the SM reads back what the host PUT (GET). */
static void test_rxfifo_host_index_access(void)
{
    pio_sim_sm_set_fifo_join(&pio, 0, PIO_FIFO_JOIN_RX_PUT);
    /* PUT: SM writes rxfifo[2], host reads it by index. */
    const uint16_t put[] = {pio_sim_encode_mov_to_rxfifo(2)};
    load_prog(put, 1);
    pio.sm[0].isr = 0xCAFEF00DU;
    pio.sm[0].isr_count = 32;
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xCAFEF00DU, pio_sim_rxfifo_get(&pio, 0, 2));

    /* GET: host writes rxfifo[1], SM reads it into OSR. */
    pio_sim_init(&pio); /* fresh SM */
    pio_sim_sm_set_fifo_join(&pio, 0, PIO_FIFO_JOIN_RX_GET);
    pio_sim_rxfifo_put(&pio, 0, 1, 0x0BADC0DEU);
    const uint16_t get[] = {pio_sim_encode_mov_from_rxfifo(1)};
    load_prog(get, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0BADC0DEU, pio.sm[0].osr);
}
#endif /* PIO_SIM_HAS_RXFIFO_MOV */

static void test_irq_next_prev_have_no_local_effect(void)
{
    /* `irq 0 next`/`prev` target a neighbouring PIO block. With no neighbour
     * linked (the default) they must raise no local flag, and crucially must not
     * be misdecoded as `rel` (the `next` mode shares bit 4 with the rel bit). */
    const uint16_t prog[] = {
        (uint16_t)(pio_sim_encode_irq(false, false, 0) | PIO_IRQ_NEXT),
        (uint16_t)(pio_sim_encode_irq(false, false, 0) | PIO_IRQ_PREV),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 4);
    for (uint8_t f = 0; f < 8U; f++) {
        TEST_ASSERT_FALSE(pio_sim_irq_get(&pio, f));
    }
}

#if PIO_SIM_HAS_WAIT_JMPPIN
static void test_wait_jmppin(void)
{
    /* `wait 1 jmppin 0` blocks until the JMP pin (pin 7 here) reads high. */
    pio_sim_sm_set_jmp_pin(&pio, 0, 7);
    const uint16_t prog[] = {
        pio_sim_encode_wait_jmppin(1, 0),
        pio_sim_encode_set(PIO_DST_X, 1),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 4); /* pin low: parked on the wait */
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x);
    pio_sim_set_pin(&pio, 7, true);
    pio_sim_sync_settle(&pio); /* skip the two-cycle input synchroniser */
    pio_sim_run(&pio, 2);      /* wait satisfied, then set x,1 */
    TEST_ASSERT_EQUAL_HEX32(0x1U, pio.sm[0].x);
}

static void test_wait_jmppin_index_offset(void)
{
    /* The index field offsets the JMP pin: jmp_pin=4, index 2 -> wait on pin 6. */
    pio_sim_sm_set_jmp_pin(&pio, 0, 4);
    const uint16_t prog[] = {
        pio_sim_encode_wait_jmppin(1, 2),
        pio_sim_encode_set(PIO_DST_X, 1),
    };
    load_prog(prog, 2);
    pio_sim_set_pin(&pio, 6, true);
    pio_sim_sync_settle(&pio);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_HEX32(0x1U, pio.sm[0].x);
}
#endif /* PIO_SIM_HAS_WAIT_JMPPIN */

#if PIO_SIM_HAS_MOV_PINDIRS
static void test_mov_pindirs_drives_dirs(void)
{
    pio_sim_sm_set_out_pins(&pio, 0, 0, 3); /* OUT/MOV pins map to GPIO 0..2 */
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 5),               /* x = 0b101         */
        pio_sim_encode_mov(3, PIO_MOV_NONE, PIO_SRC_X), /* mov pindirs, x    */
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_TRUE(pio_sim_pin_is_pio_output(&pio, 0));
    TEST_ASSERT_FALSE(pio_sim_pin_is_pio_output(&pio, 1));
    TEST_ASSERT_TRUE(pio_sim_pin_is_pio_output(&pio, 2));
}
#endif

#if PIO_SIM_HAS_IN_PIN_COUNT
/* RP2350 IN pin count: IN PINS / MOV x,PINS see only the low `in_count` pins of
 * the IN group; higher pins read as 0. WAIT PIN past the count never satisfies. */
static void test_in_pin_count_masks_high_pins(void)
{
    pio_sim_sm_set_in_base(&pio, 0, 0);
    pio_sim_sm_set_in_shift(&pio, 0, PIO_SHIFT_LEFT, false, 32);
    pio_sim_sm_set_in_pin_count(&pio, 0, 5); /* only pins 0..4 visible */
    for (uint8_t p = 0; p < 8; p++) {
        pio_sim_set_pin(&pio, p, true); /* drive 0..7 high */
    }
    const uint16_t prog[] = {pio_sim_encode_in(PIO_SRC_PINS, 8)};
    load_prog(prog, 1);
    pio_sim_sync_settle(&pio);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(0x1FU, pio.sm[0].isr); /* bits 5..7 masked to 0 */
}

static void test_wait_pin_above_in_count_never_ready(void)
{
    pio_sim_sm_set_in_base(&pio, 0, 0);
    pio_sim_sm_set_in_pin_count(&pio, 0, 5);
    pio_sim_set_pin(&pio, 6, true); /* pin 6 is high but masked out (>= count) */
    const uint16_t prog[] = {
        pio_sim_encode_wait(1, PIO_WAIT_PIN, 6), /* wait 1 pin 6 */
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    load_prog(prog, 2);
    pio_sim_sync_settle(&pio);
    pio_sim_run(&pio, 5);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].pc); /* parked on the wait */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);

    pio_sim_sm_set_in_pin_count(&pio, 0, 8); /* now pin 6 is visible */
    pio_sim_sync_settle(&pio);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y); /* wait satisfied, set y ran */
}
#endif /* PIO_SIM_HAS_IN_PIN_COUNT */

#if PIO_SIM_HAS_GPIO_BASE
static void test_gpio_base_offsets_output(void)
{
    /* With GPIOBASE=16, SET pins on view pin 2 drives physical GPIO 18. */
    pio_sim_set_gpio_base(&pio, 16);
    pio_sim_sm_set_set_pins(&pio, 0, 2, 1);
    pio_sim_sm_set_pindirs(&pio, 0, 2, 1, true); /* view pin 2 → drives GPIO 18 */
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINS, 1)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 18)); /* 16 + 2 */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 2)); /* the un-offset pin stays low */
}

static void test_gpio_base_offsets_input(void)
{
    /* With GPIOBASE=16, `wait 1 gpio 5` waits on physical GPIO 21. */
    pio_sim_set_gpio_base(&pio, 16);
    const uint16_t prog[] = {
        pio_sim_encode_wait(1, PIO_WAIT_GPIO, 5),
        pio_sim_encode_set(PIO_DST_X, 1),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x); /* GPIO21 still low */
    pio_sim_set_pin(&pio, 21, true);
    pio_sim_sync_settle(&pio);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_HEX32(0x1U, pio.sm[0].x);
}

static void test_gpio_base_get_roundtrips(void)
{
    /* The getter mirrors the setter, including the hardware 0/16 clamp. */
    TEST_ASSERT_EQUAL_UINT8(0U, pio_sim_get_gpio_base(&pio)); /* default after init */
    pio_sim_set_gpio_base(&pio, 16);
    TEST_ASSERT_EQUAL_UINT8(16U, pio_sim_get_gpio_base(&pio));
    pio_sim_set_gpio_base(&pio, 5); /* clamps down to 0 */
    TEST_ASSERT_EQUAL_UINT8(0U, pio_sim_get_gpio_base(&pio));
    pio_sim_set_gpio_base(&pio, 40); /* clamps to 16 */
    TEST_ASSERT_EQUAL_UINT8(16U, pio_sim_get_gpio_base(&pio));
}
#endif /* PIO_SIM_HAS_GPIO_BASE */

#if PIO_SIM_HAS_IRQ_PREVNEXT
static void test_irq_next_signals_neighbour_block(void)
{
    /* Two linked blocks: `irq 5 next` on `pio` sets flag 5 on `nbr`, and a
     * `wait 1 irq 5` on `nbr` then completes and clears it. */
    pio_sim_t nbr;
    pio_sim_init(&nbr);
    pio_sim_set_irq_neighbors(&pio, NULL, &nbr);

    const uint16_t setter[] = {(uint16_t)(pio_sim_encode_irq(false, false, 5) | PIO_IRQ_NEXT)};
    pio_sim_load(&pio, 0, setter, 1);
    pio_sim_sm_set_enabled(&pio, 0, true);

    const uint16_t waiter[] = {
        pio_sim_encode_wait(1, PIO_WAIT_IRQ, 5),
        pio_sim_encode_set(PIO_DST_X, 1),
    };
    pio_sim_load(&nbr, 0, waiter, 2);
    pio_sim_sm_set_enabled(&nbr, 0, true);

    /* nbr parks on the wait until pio raises the flag. */
    pio_sim_run(&nbr, 2);
    TEST_ASSERT_EQUAL_HEX32(0x0U, nbr.sm[0].x);
    pio_sim_run(&pio, 1); /* irq 5 next -> raises flag 5 on nbr */
    TEST_ASSERT_TRUE(pio_sim_irq_get(&nbr, 5));
    pio_sim_run(&nbr, 2); /* wait satisfied (clears flag), then set x,1 */
    TEST_ASSERT_EQUAL_HEX32(0x1U, nbr.sm[0].x);
    TEST_ASSERT_FALSE(pio_sim_irq_get(&nbr, 5));
}
#endif /* PIO_SIM_HAS_IRQ_PREVNEXT */

/* ── multi-PIO group ───────────────────────────────────────────────────────── */

static void test_group_runs_blocks_in_lockstep(void)
{
    pio_sim_t a;
    pio_sim_t b;
    pio_sim_init(&a);
    pio_sim_init(&b);
    pio_sim_t *blocks[] = {&a, &b};
    pio_sim_group_t g;
    pio_sim_group_init(&g, blocks, 2);
    pio_sim_group_run(&g, 5);
    TEST_ASSERT_EQUAL_UINT64(5U, a.cycle);
    TEST_ASSERT_EQUAL_UINT64(5U, b.cycle);
}

#if PIO_SIM_HAS_IRQ_PREVNEXT
static void test_group_wires_irq_ring(void)
{
    /* group_init must wire prev/next so block 0's `irq next` reaches block 1. */
    pio_sim_t a;
    pio_sim_t b;
    pio_sim_t c;
    pio_sim_init(&a);
    pio_sim_init(&b);
    pio_sim_init(&c);
    pio_sim_t *blocks[] = {&a, &b, &c};
    pio_sim_group_t g;
    pio_sim_group_init(&g, blocks, 3);

    const uint16_t prog[] = {(uint16_t)(pio_sim_encode_irq(false, false, 6) | PIO_IRQ_NEXT)};
    pio_sim_load(&a, 0, prog, 1);
    pio_sim_sm_set_enabled(&a, 0, true);
    pio_sim_group_run(&g, 1);
    TEST_ASSERT_TRUE(pio_sim_irq_get(&b, 6));  /* a's next is b */
    TEST_ASSERT_FALSE(pio_sim_irq_get(&a, 6)); /* not raised locally */
    TEST_ASSERT_FALSE(pio_sim_irq_get(&c, 6));
}
#endif /* PIO_SIM_HAS_IRQ_PREVNEXT */

/* ── Shared pads / pulls / sync bypass / masked enable ─────────────────────── */

static void test_group_shared_pads(void)
{
    /* Block A drives GPIO 0; block B (sharing pads) samples it via MOV X, PINS. */
    pio_sim_t a;
    pio_sim_t b;
    pio_sim_init(&a);
    pio_sim_init(&b);
    pio_sim_t *blocks[] = {&a, &b};
    pio_sim_group_t g;
    pio_sim_group_init_shared(&g, blocks, 2);

    pio_sim_sm_set_set_pins(&a, 0, 0, 1);
    pio_sim_sm_set_pindirs(&a, 0, 0, 1, true); /* A drives GPIO 0 as output */
    const uint16_t aprog[] = {pio_sim_encode_set(PIO_DST_PINS, 1),
                              pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&a, 0, aprog, 2);
    pio_sim_sm_set_wrap(&a, 0, 0, 1);
    pio_sim_sm_set_enabled(&a, 0, true);

    pio_sim_sm_set_in_base(&b, 0, 0);
    const uint16_t bprog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_PINS),
                              pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&b, 0, bprog, 2);
    pio_sim_sm_set_wrap(&b, 0, 0, 1);
    pio_sim_sm_set_enabled(&b, 0, true);

    pio_sim_group_run(&g, 6); /* A drives, shared synchroniser settles, B samples */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&a, 0));
    TEST_ASSERT_TRUE((b.sm[0].x & 1U) != 0U);
}

static void test_pull_open_drain(void)
{
    /* GPIO 3 pulled high: reads high while released, low while a SM drives it. */
    pio_sim_set_pull_level(&pio, (uint64_t)1U << 3U, true);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 3)); /* undriven -> pull high */

    pio_sim_sm_set_set_pins(&pio, 0, 3, 1);
    pio_sim_sm_set_pindirs(&pio, 0, 3, 1, true); /* drive as output */
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINS, 0), pio_sim_encode_jmp(0, 1)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 3)); /* driven low */
}

static void test_input_sync_bypass(void)
{
    /* With the synchroniser bypassed, a pin change is seen the same cycle. */
    pio_sim_sm_set_jmp_pin(&pio, 0, 6);
    pio_sim_set_input_sync_bypass(&pio, (uint64_t)1U << 6U);
    const uint16_t prog[] = {
        pio_sim_encode_jmp(PIO_COND_PIN, 2), /* if pin high -> addr 2 */
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 0),
        pio_sim_encode_set(PIO_DST_X, 1),
    };
    load_prog(prog, 3);
    pio_sim_set_pin(&pio, 6, true); /* no sync_settle: bypass makes it immediate */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_HEX32(0x1U, pio.sm[0].x);
}

static void test_sm_mask_enabled(void)
{
    const uint16_t prog[] = {pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&pio, 0, prog, 1);
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    TEST_ASSERT_TRUE(pio.sm[0].enabled);
    TEST_ASSERT_TRUE(pio.sm[1].enabled);
    TEST_ASSERT_FALSE(pio.sm[2].enabled);
    /* Restarted in phase: accumulators aligned. */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].clk_accum);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[1].clk_accum);
}

/* ── DMA pacing ────────────────────────────────────────────────────────────── */

static void test_dma_feeds_and_drains_fifos(void)
{
    /* An echo program (TX -> OSR -> ISR -> RX) with a DMA channel feeding TX and
     * another draining RX. Both channels run to completion, paced by DREQ. */
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),                           /* pull block      */
        pio_sim_encode_mov(PIO_DST_ISR, PIO_MOV_NONE, PIO_SRC_OSR), /* mov isr, osr    */
        pio_sim_encode_push(false, true),                           /* push block      */
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 0),                     /* jmp 0           */
    };
    load_prog(prog, 4);

    uint32_t in_buf[3] = {0x11111111U, 0x22222222U, 0x33333333U};
    uint32_t out_buf[3] = {0};
    pio_sim_dma_t tx;
    pio_sim_dma_t rx;
    pio_sim_dma_init(&tx, &pio, 0, PIO_DMA_TO_SM, in_buf, 3);
    pio_sim_dma_init(&rx, &pio, 0, PIO_DMA_FROM_SM, out_buf, 3);

    for (int i = 0; (i < 200) && !(pio_sim_dma_done(&tx) && pio_sim_dma_done(&rx)); i++) {
        (void)pio_sim_dma_step(&tx);
        (void)pio_sim_dma_step(&rx);
        pio_sim_tick(&pio);
    }
    TEST_ASSERT_TRUE(pio_sim_dma_done(&tx));
    TEST_ASSERT_TRUE(pio_sim_dma_done(&rx));
    TEST_ASSERT_EQUAL_HEX32(0x11111111U, out_buf[0]);
    TEST_ASSERT_EQUAL_HEX32(0x22222222U, out_buf[1]);
    TEST_ASSERT_EQUAL_HEX32(0x33333333U, out_buf[2]);
}

/* TX->OSR->ISR->RX echo, used to round-trip DMA through a state machine. */
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

/* Drive a TX channel (following its chain) and an RX channel concurrently until
 * the RX channel finishes or `limit` ticks pass. */
static void run_dma_pair(pio_sim_dma_t *tx, pio_sim_dma_t *rx, int limit)
{
    pio_sim_dma_t *txcur = tx;
    for (int i = 0; (i < limit) && !pio_sim_dma_done(rx); i++) {
        if ((txcur != NULL) && pio_sim_dma_done(txcur)) {
            txcur = txcur->chain;
        }
        if (txcur != NULL) {
            (void)pio_sim_dma_step(txcur);
        }
        (void)pio_sim_dma_step(rx);
        pio_sim_tick(&pio);
    }
}

static void test_dma_byte_size_roundtrip(void)
{
    load_echo_prog();
    uint8_t in[4] = {0xAAU, 0xBBU, 0xCCU, 0xDDU};
    uint8_t out[4] = {0};
    pio_sim_dma_t tx;
    pio_sim_dma_t rx;
    pio_sim_dma_init_ex(&tx, &pio, 0, PIO_DMA_TO_SM, in, 4, PIO_DMA_SIZE_8, true, 0, NULL);
    pio_sim_dma_init_ex(&rx, &pio, 0, PIO_DMA_FROM_SM, out, 4, PIO_DMA_SIZE_8, true, 0, NULL);
    run_dma_pair(&tx, &rx, 200);
    TEST_ASSERT_TRUE(pio_sim_dma_done(&rx));
    TEST_ASSERT_EQUAL_HEX8(0xAAU, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBBU, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0xCCU, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0xDDU, out[3]);
}

static void test_dma_ring_wraps_source(void)
{
    load_echo_prog();
    uint32_t in[2] = {0x55U, 0x66U};
    uint32_t out[4] = {0};
    pio_sim_dma_t tx;
    pio_sim_dma_t rx;
    pio_sim_dma_init_ex(&tx, &pio, 0, PIO_DMA_TO_SM, in, 4, PIO_DMA_SIZE_32, true, 2, NULL);
    pio_sim_dma_init(&rx, &pio, 0, PIO_DMA_FROM_SM, out, 4);
    run_dma_pair(&tx, &rx, 200);
    const uint32_t expect[4] = {0x55U, 0x66U, 0x55U, 0x66U};
    for (uint8_t i = 0; i < 4U; i++) {
        TEST_ASSERT_EQUAL_HEX32(expect[i], out[i]);
    }
}

static void test_dma_chain_to_next(void)
{
    load_echo_prog();
    uint32_t a[2] = {0x11U, 0x22U};
    uint32_t b[2] = {0x33U, 0x44U};
    uint32_t out[4] = {0};
    pio_sim_dma_t tx2;
    pio_sim_dma_t tx1;
    pio_sim_dma_t rx;
    pio_sim_dma_init(&tx2, &pio, 0, PIO_DMA_TO_SM, b, 2);
    pio_sim_dma_init_ex(&tx1, &pio, 0, PIO_DMA_TO_SM, a, 2, PIO_DMA_SIZE_32, true, 0, &tx2);
    pio_sim_dma_init(&rx, &pio, 0, PIO_DMA_FROM_SM, out, 4);
    run_dma_pair(&tx1, &rx, 200);
    const uint32_t expect[4] = {0x11U, 0x22U, 0x33U, 0x44U};
    for (uint8_t i = 0; i < 4U; i++) {
        TEST_ASSERT_EQUAL_HEX32(expect[i], out[i]);
    }
    TEST_ASSERT_TRUE(pio_sim_dma_done(&tx2));
}

static void test_dma_run_drains_tx(void)
{
    /* `pull` in a loop consumes TX; pio_sim_dma_run feeds it until all words
     * have been transferred. */
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 0),
    };
    load_prog(prog, 2);

    uint32_t in_buf[4] = {1U, 2U, 3U, 4U};
    pio_sim_dma_t tx;
    pio_sim_dma_init(&tx, &pio, 0, PIO_DMA_TO_SM, in_buf, 4);
    (void)pio_sim_dma_run(&tx, 200);
    TEST_ASSERT_TRUE(pio_sim_dma_done(&tx));
    TEST_ASSERT_EQUAL_UINT32(4U, tx.pos);
}

/* ── FIFO debug / status (FDEBUG / FSTAT) ──────────────────────────────────── */

static void test_fdebug_flags(void)
{
    /* PULL (blocking) on an empty TX FIFO stalls -> TXSTALL. */
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 0),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_TRUE((pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_TXSTALL) != 0U);
    pio_sim_clear_fdebug(&pio, 0, PIO_FDEBUG_TXSTALL);
    TEST_ASSERT_EQUAL_UINT8(0U, (uint8_t)(pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_TXSTALL));

    /* Host overflow -> TXOVER; levels track occupancy. */
    for (uint32_t i = 0; i < 4U; i++) {
        TEST_ASSERT_TRUE(pio_sim_tx_push(&pio, 0, i));
    }
    TEST_ASSERT_EQUAL_UINT8(4U, pio_sim_tx_level(&pio, 0));
    TEST_ASSERT_FALSE(pio_sim_tx_push(&pio, 0, 99U));
    TEST_ASSERT_TRUE((pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_TXOVER) != 0U);

    /* Host underflow -> RXUNDER. */
    uint32_t w;
    TEST_ASSERT_FALSE(pio_sim_rx_pop(&pio, 0, &w));
    TEST_ASSERT_TRUE((pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_RXUNDER) != 0U);
    TEST_ASSERT_EQUAL_UINT8(0U, pio_sim_rx_level(&pio, 0));
}

/* ── System interrupt lines (IRQ0 / IRQ1) ──────────────────────────────────── */

static void test_system_irq_lines(void)
{
    /* A fresh SM has a non-full TX FIFO, so TXNFULL is a live INTR source. */
    TEST_ASSERT_TRUE((pio_sim_get_irq_raw(&pio) & PIO_INTR_SM_TXNFULL(0)) != 0U);
    TEST_ASSERT_FALSE(pio_sim_interrupt_line(&pio, 0)); /* nothing enabled yet */
    pio_sim_set_irq_enable(&pio, 0, PIO_INTR_SM_TXNFULL(0));
    TEST_ASSERT_TRUE(pio_sim_interrupt_line(&pio, 0));

    /* An SM IRQ flag drives line 1 once enabled. */
    pio_sim_set_irq_enable(&pio, 1, PIO_INTR_SM_IRQ(2));
    TEST_ASSERT_FALSE(pio_sim_interrupt_line(&pio, 1));
    const uint16_t prog[] = {pio_sim_encode_irq(false, false, 2)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_interrupt_line(&pio, 1));

    /* INTF forces a line regardless of sources/enables. */
    pio_sim_set_irq_enable(&pio, 0, 0);
    TEST_ASSERT_FALSE(pio_sim_interrupt_line(&pio, 0));
    pio_sim_set_irq_force(&pio, 0, PIO_INTR_SM_IRQ(0));
    TEST_ASSERT_TRUE(pio_sim_interrupt_line(&pio, 0));
}

/* The INTE/INTF masks read back exactly as set, per line and independently. */
static void test_irq_enable_force_read_back(void)
{
    TEST_ASSERT_EQUAL_HEX32(0U, pio_sim_get_irq_enable(&pio, 0)); /* default after init */
    TEST_ASSERT_EQUAL_HEX32(0U, pio_sim_get_irq_force(&pio, 1));

    pio_sim_set_irq_enable(&pio, 0, 0x00F0U);
    pio_sim_set_irq_force(&pio, 1, 0x0A00U);
    TEST_ASSERT_EQUAL_HEX32(0x00F0U, pio_sim_get_irq_enable(&pio, 0));
    TEST_ASSERT_EQUAL_HEX32(0x0A00U, pio_sim_get_irq_force(&pio, 1));
    /* The other line of each mask stays untouched. */
    TEST_ASSERT_EQUAL_HEX32(0U, pio_sim_get_irq_enable(&pio, 1));
    TEST_ASSERT_EQUAL_HEX32(0U, pio_sim_get_irq_force(&pio, 0));
}

/* ── #8: input synchroniser (always-on two-cycle delay) ────────────────────── */

static void test_input_sync_delays_jmp_pin_by_two_cycles(void)
{
    /* A pin change is seen by the PIO two system clocks later. A self-looping
     * `jmp pin -> escape` waits on pin 5; the loop falls through only once the
     * delayed input reaches the PIO. */
    pio_sim_sm_set_jmp_pin(&pio, 0, 5);
    const uint16_t prog[] = {
        pio_sim_encode_jmp(PIO_COND_PIN, 2), /* if pin: jump out of the loop */
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 0),
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    load_prog(prog, 3);

    pio_sim_run(&pio, 4); /* spins in the loop: pin still low */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);

    pio_sim_set_pin(&pio, 5, true);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y); /* not visible yet (2-cycle lag) */
    pio_sim_run(&pio, 4);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y); /* observed after the delay */
}

static void test_sync_settle_makes_static_input_immediate(void)
{
    /* pio_sim_sync_settle fast-forwards the pipeline to a held input, so a
     * level set before the run is seen on the first sample. */
    pio_sim_sm_set_jmp_pin(&pio, 0, 5);
    const uint16_t prog[] = {
        pio_sim_encode_jmp(PIO_COND_PIN, 2),
        pio_sim_encode_set(PIO_DST_Y, 9),
        pio_sim_encode_set(PIO_DST_Y, 1),
    };
    load_prog(prog, 3);
    pio_sim_set_pin(&pio, 5, true);
    pio_sim_sync_settle(&pio);
    pio_sim_run(&pio, 2); /* jmp taken immediately → set y,1 */
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].y);
}

/* ── Hardening: FIFO-join survives restart; unwritten-fetch diagnostic ─────── */

static void test_fifo_join_survives_restart(void)
{
    pio_sim_sm_set_fifo_join(&pio, 0, PIO_FIFO_JOIN_RX);
    TEST_ASSERT_EQUAL_UINT8(PIO_SIM_FIFO_MAX, pio.sm[0].rx.cap);
    pio_sim_sm_restart(&pio, 0); /* must not revert the 8-deep join to 4+4 */
    TEST_ASSERT_EQUAL_UINT8(PIO_SIM_FIFO_MAX, pio.sm[0].rx.cap);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].tx.cap);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].rx.count); /* contents cleared though */
}

static void test_clear_fifos(void)
{
    pio_sim_tx_push(&pio, 0, 1U);
    pio_sim_tx_push(&pio, 0, 2U);
    TEST_ASSERT_FALSE(pio_sim_tx_empty(&pio, 0));
    pio_sim_sm_clear_fifos(&pio, 0);
    TEST_ASSERT_TRUE(pio_sim_tx_empty(&pio, 0));
    TEST_ASSERT_TRUE(pio_sim_rx_empty(&pio, 0));
}

static void test_group_enable_sm_mask_sync(void)
{
    pio_sim_t a, b;
    pio_sim_init(&a);
    pio_sim_init(&b);
    const uint16_t prog[] = {pio_sim_encode_nop(), pio_sim_encode_nop(), pio_sim_encode_nop()};
    pio_sim_load(&a, 0, prog, 3);
    pio_sim_load(&b, 0, prog, 3);
    pio_sim_sm_set_wrap(&a, 0, 0, 2);
    pio_sim_sm_set_wrap(&b, 0, 0, 2);
    pio_sim_t *blocks[] = {&a, &b};
    pio_sim_group_t g;
    pio_sim_group_init(&g, blocks, 2);
    const uint8_t masks[] = {0x1U, 0x1U};
    pio_sim_group_enable_sm_mask_sync(&g, masks);
    pio_sim_group_run(&g, 5);
    TEST_ASSERT_TRUE(pio_sim_sm_is_enabled(&a, 0));
    TEST_ASSERT_TRUE(pio_sim_sm_is_enabled(&b, 0));
    /* Started cycle-aligned with equal dividers → identical, advanced PCs. */
    TEST_ASSERT_NOT_EQUAL(0U, pio_sim_sm_get_pc(&a, 0));
    TEST_ASSERT_EQUAL_UINT8(pio_sim_sm_get_pc(&a, 0), pio_sim_sm_get_pc(&b, 0));
}

static void test_unwritten_fetch_is_flagged(void)
{
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_X, 1)};
    load_prog(prog, 1); /* loads only address 0 */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT64(0U, pio.unwritten_fetches); /* stays within program */
    /* Force the PC past the loaded program; the next fetch is from unwritten
     * memory and must be counted. */
    pio_sim_sm_set_wrap(&pio, 0, 0, PIO_SIM_INSN_COUNT - 1U);
    pio_sim_sm_set_pc(&pio, 0, 5);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio.unwritten_fetches > 0U);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_set_x_and_y);
    RUN_TEST(test_set_pins_drives_pads);
    RUN_TEST(test_set_pindirs);

    RUN_TEST(test_out_pins_shift_right);
    RUN_TEST(test_out_shift_left_takes_top_bits);
    RUN_TEST(test_out_autopull_refills);
    RUN_TEST(test_out_autopull_eager_refill);
    RUN_TEST(test_out_sticky_releases_on_inline_disable);
    RUN_TEST(test_out_inline_enable_gates_write);
    RUN_TEST(test_multi_sm_pin_priority_and_override);
    RUN_TEST(test_multi_sm_priority_across_cycles);
    RUN_TEST(test_pin_floats_when_released);
    RUN_TEST(test_host_release_pin);
    RUN_TEST(test_out_exec_injects_instruction);

    RUN_TEST(test_in_pins_shift_left);
    RUN_TEST(test_in_autopush_to_rx);

    RUN_TEST(test_pull_blocks_until_tx);
    RUN_TEST(test_pull_noblock_on_empty_copies_x);
    RUN_TEST(test_push_blocks_when_rx_full);

    RUN_TEST(test_mov_x_to_y_with_invert);
    RUN_TEST(test_mov_reverse);

    RUN_TEST(test_jmp_not_x_taken_when_zero);
    RUN_TEST(test_jmp_x_dec_loop_counts);
    RUN_TEST(test_jmp_pin);

    RUN_TEST(test_wait_gpio_high);
    RUN_TEST(test_wait_irq_clears_flag);

    RUN_TEST(test_irq_set_and_clear);

    RUN_TEST(test_sideset_drives_pin);
    RUN_TEST(test_delay_holds_pc);
    RUN_TEST(test_clkdiv_slows_execution);
    RUN_TEST(test_clkdiv_fractional_cadence);
    RUN_TEST(test_wrap_returns_to_bottom);

    RUN_TEST(test_fifo_join_rx_depth_8);
    RUN_TEST(test_tx_empty_and_irq_clear_accessors);

    RUN_TEST(test_in_autopush_full_does_not_reshift);
    RUN_TEST(test_irq_wait_parks_until_cleared);
    RUN_TEST(test_sm_exec_irq_wait_does_not_rearm);
    RUN_TEST(test_sm_get_pc_reads_back);
    RUN_TEST(test_sm_is_enabled_reads_back);
    RUN_TEST(test_sm_get_instr_reads_current);
    RUN_TEST(test_sm_is_stalled);
    RUN_TEST(test_sm_exec_ignores_delay);
    RUN_TEST(test_sideset_applies_while_stalled);

    RUN_TEST(test_mov_status_tx_level);
    RUN_TEST(test_mov_status_rx_level);
#if PIO_SIM_HAS_IRQ_STATUS
    RUN_TEST(test_mov_status_irq_flag);
#endif
    RUN_TEST(test_mov_status_override);

    RUN_TEST(test_irq_rel_maps_per_sm);
    RUN_TEST(test_irq_rel_preserves_high_bit);
    RUN_TEST(test_wait_irq_rel);
    RUN_TEST(test_clkdiv_restart_realigns_sms);
    RUN_TEST(test_clkdiv_freeruns_while_disabled);

#if PIO_SIM_HAS_RXFIFO_MOV
    RUN_TEST(test_mov_rxfifo_roundtrip);
    RUN_TEST(test_mov_rxfifo_y_indexed);
    RUN_TEST(test_rxfifo_host_index_access);
#endif
    RUN_TEST(test_irq_next_prev_have_no_local_effect);
#if PIO_SIM_HAS_WAIT_JMPPIN
    RUN_TEST(test_wait_jmppin);
    RUN_TEST(test_wait_jmppin_index_offset);
#endif
#if PIO_SIM_HAS_MOV_PINDIRS
    RUN_TEST(test_mov_pindirs_drives_dirs);
#endif
#if PIO_SIM_HAS_IN_PIN_COUNT
    RUN_TEST(test_in_pin_count_masks_high_pins);
    RUN_TEST(test_wait_pin_above_in_count_never_ready);
#endif
#if PIO_SIM_HAS_GPIO_BASE
    RUN_TEST(test_gpio_base_offsets_output);
    RUN_TEST(test_gpio_base_offsets_input);
    RUN_TEST(test_gpio_base_get_roundtrips);
#endif
#if PIO_SIM_HAS_IRQ_PREVNEXT
    RUN_TEST(test_irq_next_signals_neighbour_block);
#endif
    RUN_TEST(test_group_runs_blocks_in_lockstep);
#if PIO_SIM_HAS_IRQ_PREVNEXT
    RUN_TEST(test_group_wires_irq_ring);
#endif
    RUN_TEST(test_group_shared_pads);
    RUN_TEST(test_pull_open_drain);
    RUN_TEST(test_input_sync_bypass);
    RUN_TEST(test_sm_mask_enabled);
    RUN_TEST(test_dma_feeds_and_drains_fifos);
    RUN_TEST(test_dma_run_drains_tx);
    RUN_TEST(test_dma_byte_size_roundtrip);
    RUN_TEST(test_dma_ring_wraps_source);
    RUN_TEST(test_dma_chain_to_next);
    RUN_TEST(test_fdebug_flags);
    RUN_TEST(test_system_irq_lines);
    RUN_TEST(test_irq_enable_force_read_back);

    RUN_TEST(test_input_sync_delays_jmp_pin_by_two_cycles);
    RUN_TEST(test_sync_settle_makes_static_input_immediate);

    RUN_TEST(test_fifo_join_survives_restart);
    RUN_TEST(test_clear_fifos);
    RUN_TEST(test_group_enable_sm_mask_sync);
    RUN_TEST(test_unwritten_fetch_is_flagged);

    return UNITY_END();
}
