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
    pio_sim_sm_set_set_pins(&pio, 0, 4, 3); /* pins 4,5,6 */
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
    /* A self-loop keeps each SM busy without advancing the PC, so the divider
     * phase (the clk accumulator) is the only thing under test. */
    const uint16_t prog[] = {pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&pio, 0, prog, 1);
    for (uint8_t s = 0; s < 2U; s++) {
        pio_sim_sm_set_wrap(&pio, s, 0, 0);
        pio_sim_sm_set_clkdiv(&pio, s, 4, 0); /* one SM cycle every 4 ticks */
    }
    /* Start SM0 a tick ahead of SM1 so their dividers run out of phase. */
    pio_sim_sm_set_enabled(&pio, 0, true);
    pio_sim_run(&pio, 1);
    pio_sim_sm_set_enabled(&pio, 1, true);
    pio_sim_run(&pio, 3);
    TEST_ASSERT_TRUE(pio.sm[0].clk_accum != pio.sm[1].clk_accum);

    /* Re-align: both accumulators reset to the same phase and advance together,
     * so the two SMs now step on identical ticks (lockstep). */
    pio_sim_clkdiv_restart(&pio, 0x3U);
    TEST_ASSERT_EQUAL_UINT32(pio.sm[0].clk_accum, pio.sm[1].clk_accum);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].clk_accum);
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_UINT32(pio.sm[0].clk_accum, pio.sm[1].clk_accum);
    TEST_ASSERT_EQUAL_UINT32(768U, pio.sm[0].clk_accum); /* 3 × 256, no fire yet */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(pio.sm[0].clk_accum, pio.sm[1].clk_accum);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].clk_accum); /* both fired on this tick */
}

/* ── #7: RP2350 instructions ───────────────────────────────────────────────── */

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

static void test_irq_next_prev_have_no_local_effect(void)
{
    /* `irq 0 next`/`prev` target a neighbouring PIO block (not modelled): they
     * must raise no local flag, and crucially must not be misdecoded as `rel`
     * (the `next` mode shares bit 4 with the rel bit). */
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
    RUN_TEST(test_wrap_returns_to_bottom);

    RUN_TEST(test_fifo_join_rx_depth_8);
    RUN_TEST(test_tx_empty_and_irq_clear_accessors);

    RUN_TEST(test_in_autopush_full_does_not_reshift);
    RUN_TEST(test_irq_wait_parks_until_cleared);
    RUN_TEST(test_sm_exec_ignores_delay);
    RUN_TEST(test_sideset_applies_while_stalled);

    RUN_TEST(test_mov_status_tx_level);
    RUN_TEST(test_mov_status_rx_level);
    RUN_TEST(test_mov_status_irq_flag);
    RUN_TEST(test_mov_status_override);

    RUN_TEST(test_irq_rel_maps_per_sm);
    RUN_TEST(test_irq_rel_preserves_high_bit);
    RUN_TEST(test_wait_irq_rel);
    RUN_TEST(test_clkdiv_restart_realigns_sms);

    RUN_TEST(test_mov_rxfifo_roundtrip);
    RUN_TEST(test_mov_rxfifo_y_indexed);
    RUN_TEST(test_irq_next_prev_have_no_local_effect);

    RUN_TEST(test_input_sync_delays_jmp_pin_by_two_cycles);
    RUN_TEST(test_sync_settle_makes_static_input_immediate);

    RUN_TEST(test_fifo_join_survives_restart);
    RUN_TEST(test_unwritten_fetch_is_flagged);

    return UNITY_END();
}
