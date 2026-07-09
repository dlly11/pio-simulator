/* test_pio_sim.c — unit tests for the PIO instruction-set simulator.
 *
 * Exercises every instruction class (JMP/WAIT/IN/OUT/PUSH/PULL/MOV/IRQ/SET)
 * plus side-set, delay, the clock divider, wrap, shift directions, and
 * autopush/autopull, using hand-encoded one- and two-instruction programs. */

#include "unity.h"

#include "pio_sim.h"

static pio_sim_t pio;
/* SM0 config the tests build with sm_config_set_*; reset each test. A test that
 * needs non-default config mutates `cfg` before calling load_prog. */
static pio_sm_config cfg;

void setUp(void)
{
    pio_sim_init(&pio);
    cfg = pio_get_default_sm_config();
}
void tearDown(void) { (void)0; }

/* Load a program at offset 0, apply cfg (spanning wrap over the program unless
 * the test already set a custom wrap), init + enable SM0. */
static void load_prog(const uint16_t *prog, uint8_t n)
{
    pio_sim_load(&pio, 0, prog, n);
    if ((cfg.wrap_bottom == 0U) && (cfg.wrap_top == PIO_SIM_INSN_COUNT - 1U)) {
        sm_config_set_wrap(&cfg, 0, (uint8_t)(n - 1U));
    }
    pio_sim_sm_init(&pio, 0, 0, &cfg);
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
    sm_config_set_set_pins(&cfg, 4, 3);                      /* pins 4,5,6 */
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 4, 3, true); /* drive them as outputs */
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINS, 0x5)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 4));  /* bit0 = 1 */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 5)); /* bit1 = 0 */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 6));  /* bit2 = 1 */
}

static void test_set_pindirs(void)
{
    sm_config_set_set_pins(&cfg, 8, 2);
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINDIRS, 0x1)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_pin_is_pio_output(&pio, 8));
    TEST_ASSERT_FALSE(pio_sim_pin_is_pio_output(&pio, 9));
}

/* ── OUT / shift directions / autopull ─────────────────────────────────────── */

static void test_out_pins_shift_right(void)
{
    sm_config_set_out_pins(&cfg, 0, 4);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 0, 4, true);
    sm_config_set_out_shift(&cfg, true, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_out(PIO_DST_PINS, 4), /* low 4 bits first */
    };
    load_prog(prog, 2);
    pio_sim_sm_put(&pio, 0, 0xA); /* 1010 */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 0)); /* bit0 = 0 */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 1));  /* bit1 = 1 */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 2));
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 3));
}

static void test_out_shift_left_takes_top_bits(void)
{
    sm_config_set_out_shift(&cfg, false, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        /* top 4 bits of OSR */
        pio_sim_encode_out(PIO_DST_X, 4),
    };
    load_prog(prog, 2);
    pio_sim_sm_put(&pio, 0, 0xC000000A);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(0xCU, pio.sm[0].x); /* top nibble */
}

static void test_out_autopull_refills(void)
{
    sm_config_set_out_shift(&cfg, true, true, 8);
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_X, 8)};
    load_prog(prog, 1);
    pio_sim_sm_put(&pio, 0, 0x11);
    pio_sim_sm_put(&pio, 0, 0x22);
    pio_sim_run(&pio, 1); /* first OUT autopulls 0x11, outputs low byte */
    TEST_ASSERT_EQUAL_UINT32(0x11U, pio.sm[0].x);
    pio_sim_run(&pio, 1); /* second OUT autopulls 0x22 */
    TEST_ASSERT_EQUAL_UINT32(0x22U, pio.sm[0].x);
}

/* Eager autopull: the OUT that empties the OSR refills it immediately, so the
 * next instruction sees a full OSR (the prefetched word) — not an empty one. */
static void test_out_autopull_background_refill(void)
{
    /* Autopull is a background refill at the top of each SM cycle: the OUT
     * that exhausts the OSR leaves it exhausted at end of tick; the refill
     * lands before the next cycle's instruction executes, so a following MOV
     * from OSR reads the fresh word and streaming never stalls. */
    sm_config_set_out_shift(&cfg, true, true, 32);
    const uint16_t prog[] = {
        pio_sim_encode_out(PIO_DST_NULL, 32), /* discard; exhausts the OSR */
        pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_OSR),
    };
    load_prog(prog, 2);
    pio_sim_sm_put(&pio, 0, 0xAAAAAAAAU);
    pio_sim_sm_put(&pio, 0, 0xBBBBBBBBU);
    pio_sim_run(&pio, 1); /* OUT shifts 0xAAAA… out; OSR exhausted at end of tick */
    TEST_ASSERT_EQUAL_UINT8(32U, pio.sm[0].osr_count);
    pio_sim_run(&pio, 1); /* background refill, then mov x, osr reads the new word */
    TEST_ASSERT_EQUAL_HEX32(0xBBBBBBBBU, pio.sm[0].x);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].osr_count); /* refilled, nothing consumed */
    TEST_ASSERT_FALSE((pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_TXSTALL) != 0U);
}

static void test_out_autopull_streams_word_per_cycle(void)
{
    /* With a fed FIFO, back-to-back `out null, 32` sustains one word per SM
     * cycle with no TXSTALL: the background refill lands before each OUT. */
    sm_config_set_out_shift(&cfg, true, true, 32);
    sm_config_set_wrap(&cfg, 0, 0); /* single-instruction loop */
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_NULL, 32),
                             pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    load_prog(prog, 2);
    for (uint8_t i = 0; i < 4U; i++) {
        pio_sim_sm_put(&pio, 0, i);
    }
    pio_sim_run(&pio, 4); /* 4 OUT cycles consume all 4 words */
    TEST_ASSERT_TRUE(pio_sim_sm_is_tx_fifo_empty(&pio, 0));
    TEST_ASSERT_EQUAL_UINT8(32U, pio.sm[0].osr_count); /* last word fully shifted */
    TEST_ASSERT_FALSE((pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_TXSTALL) != 0U);
}

static void test_out_autopull_empty_then_fed_stalls_one_extra_cycle(void)
{
    /* An OUT that finds OSR exhausted and TX empty stalls with TXSTALL; after
     * a word is pushed it cannot complete the same cycle the refill happens —
     * the OUT commits on the following SM cycle (§3.5.4.1). */
    sm_config_set_out_shift(&cfg, true, true, 32);
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_X, 32),
                             pio_sim_encode_jmp(PIO_COND_ALWAYS, 1)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 2); /* OSR empty at reset + TX empty: OUT stalls */
    TEST_ASSERT_TRUE(pio_sim_sm_is_stalled(&pio, 0));
    TEST_ASSERT_TRUE((pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_TXSTALL) != 0U);
    pio_sim_sm_put(&pio, 0, 0x12345678U);
    pio_sim_run(&pio, 1); /* refill lands this cycle; the OUT retries and commits */
    TEST_ASSERT_EQUAL_HEX32(0x12345678U, pio.sm[0].x);
    TEST_ASSERT_EQUAL_UINT8(1U, pio_sim_sm_get_pc(&pio, 0)); /* moved on */
}

static void test_register_accessors_read_back(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 5),
        pio_sim_encode_set(PIO_DST_Y, 9),
        pio_sim_encode_in(PIO_SRC_X, 4),
    };
    load_prog(prog, 3);
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_UINT32(5U, pio_sim_sm_get_x(&pio, 0));
    TEST_ASSERT_EQUAL_UINT32(9U, pio_sim_sm_get_y(&pio, 0));
    TEST_ASSERT_EQUAL_UINT32(5U, pio_sim_sm_get_isr(&pio, 0)); /* shifted left 4, in 5 */
    TEST_ASSERT_EQUAL_UINT8(4U, pio_sim_sm_get_isr_count(&pio, 0));
    TEST_ASSERT_EQUAL_UINT8(32U, pio_sim_sm_get_osr_count(&pio, 0)); /* untouched: exhausted */
    TEST_ASSERT_EQUAL_UINT32(0U, pio_sim_sm_get_osr(&pio, 0));
}

static uint8_t trace_pcs[8];
static uint16_t trace_insns[8];
static uint8_t trace_count;

static void trace_record(const pio_sim_t *p, uint8_t sm, uint8_t pc, uint16_t insn, void *ctx)
{
    (void)p;
    (void)sm;
    (void)ctx;
    if (trace_count < 8U) {
        trace_pcs[trace_count] = pc;
        trace_insns[trace_count] = insn;
    }
    trace_count++;
}

static void test_trace_hook_fires_on_commit_only(void)
{
    /* 3-instruction program: a pull that stalls (TX empty) must not trace
     * until it commits; a delay cycle must not trace either. */
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 3),
        (uint16_t)(pio_sim_encode_set(PIO_DST_Y, 1) | (1U << 8U)), /* [1] delay */
        pio_sim_encode_pull(false, true),
    };
    load_prog(prog, 3);
    trace_count = 0;
    pio_sim_set_trace(&pio, trace_record, NULL);

    pio_sim_run(&pio, 5);                     /* set x; set y [1]; delay; pull stalls twice */
    TEST_ASSERT_EQUAL_UINT8(2U, trace_count); /* stall + delay cycles: no trace */
    TEST_ASSERT_EQUAL_UINT8(0U, trace_pcs[0]);
    TEST_ASSERT_EQUAL_HEX16(prog[0], trace_insns[0]);
    TEST_ASSERT_EQUAL_UINT8(1U, trace_pcs[1]);
    TEST_ASSERT_EQUAL_HEX16(prog[1], trace_insns[1]);

    pio_sim_sm_put(&pio, 0, 42U);
    pio_sim_run(&pio, 1); /* pull commits now */
    TEST_ASSERT_EQUAL_UINT8(3U, trace_count);
    TEST_ASSERT_EQUAL_UINT8(2U, trace_pcs[2]);
    TEST_ASSERT_EQUAL_HEX16(prog[2], trace_insns[2]);

    /* Forced instructions trace too, with the parked PC. */
    pio_sim_sm_exec(&pio, 0, pio_sim_encode_set(PIO_DST_X, 1));
    TEST_ASSERT_EQUAL_UINT8(4U, trace_count);

    pio_sim_set_trace(&pio, NULL, NULL);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT8(4U, trace_count); /* cleared: no more callbacks */
}

static void test_run_until_tx_drained_waits_for_osr(void)
{
    /* Stream two words out a pin with autopull: run_until_tx_empty returns
     * while the last word is still shifting from the OSR; _drained returns
     * only once the SM stalls on truly exhausted data. */
    sm_config_set_out_shift(&cfg, true, true, 32);
    sm_config_set_out_pins(&cfg, 0, 1);
    sm_config_set_wrap(&cfg, 0, 0);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 0, 1, true);
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_PINS, 1),
                             pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    load_prog(prog, 2);
    pio_sim_sm_put(&pio, 0, 0xFFFFFFFFU);
    pio_sim_sm_put(&pio, 0, 0xFFFFFFFFU);

    uint64_t t_empty = pio_sim_run_until_tx_empty(&pio, 0, 1000);
    TEST_ASSERT_TRUE(pio_sim_sm_is_tx_fifo_empty(&pio, 0));
    TEST_ASSERT_TRUE(t_empty < 40U); /* FIFO empty well before 64 bits are out */

    uint64_t t_drained = pio_sim_run_until_tx_drained(&pio, 0, 1000);
    TEST_ASSERT_TRUE(t_drained < 1000U);            /* did stall, not time out */
    TEST_ASSERT_TRUE((t_empty + t_drained) >= 64U); /* all 64 bits shifted */
    TEST_ASSERT_EQUAL_UINT8(32U, pio_sim_sm_get_osr_count(&pio, 0));
}

static void test_autopull_background_refill_without_out(void)
{
    /* The refill is not tied to OUT: after a push, an SM parked on non-OUT
     * instructions still tops up its OSR, so JMP !OSRE sees a full OSR and a
     * PULL acts as a no-op barrier instead of popping a second word. */
    sm_config_set_out_shift(&cfg, true, true, 32);
    const uint16_t prog[] = {
        pio_sim_encode_jmp(PIO_COND_NOTOSRE, 2), /* taken once the OSR holds data */
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 0),  /* else keep polling */
        pio_sim_encode_pull(false, true),        /* barrier: OSR already full */
        pio_sim_encode_jmp(PIO_COND_ALWAYS, 3),  /* park */
    };
    load_prog(prog, 4);
    pio_sim_run(&pio, 2); /* OSR exhausted at reset + TX empty: still polling */
    TEST_ASSERT_EQUAL_UINT8(0U, pio_sim_sm_get_pc(&pio, 0));
    pio_sim_sm_put(&pio, 0, 0xCAFEF00DU);
    pio_sim_sm_put(&pio, 0, 0xDEADBEEFU);
    /* Background refill pulls 0xCAFEF00D before the jmp executes — no OUT ran. */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xCAFEF00DU, pio.sm[0].osr);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].osr_count);
    TEST_ASSERT_EQUAL_UINT8(2U, pio_sim_sm_get_pc(&pio, 0)); /* !OSRE taken */
    pio_sim_run(&pio, 1); /* pull: no-op barrier — must not pop 0xDEADBEEF */
    TEST_ASSERT_EQUAL_HEX32(0xCAFEF00DU, pio.sm[0].osr);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].tx.count);
    TEST_ASSERT_EQUAL_UINT8(3U, pio_sim_sm_get_pc(&pio, 0));
}

/* With autopull enabled, PULL is a no-op (a barrier) while the OSR is full —
 * it must not pop and discard another TX word (RP2040 §3.5.4.2). */
static void test_pull_noop_when_autopull_osr_full(void)
{
    sm_config_set_out_shift(&cfg, true, true, 32);
    const uint16_t prog[] = {
        pio_sim_encode_out(PIO_DST_X, 32), /* autopulls word A into OSR, X <- A */
        pio_sim_encode_pull(false, true),  /* OSR already refilled with B: no-op */
        pio_sim_encode_mov(PIO_DST_Y, PIO_MOV_NONE, PIO_SRC_OSR),
    };
    load_prog(prog, 3);
    pio_sim_sm_put(&pio, 0, 0xAAAAAAAAU);
    pio_sim_sm_put(&pio, 0, 0xBBBBBBBBU);
    pio_sim_sm_put(&pio, 0, 0xCCCCCCCCU);
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_HEX32(0xAAAAAAAAU, pio.sm[0].x);
    TEST_ASSERT_EQUAL_HEX32(0xBBBBBBBBU, pio.sm[0].y); /* PULL did not clobber OSR */
    TEST_ASSERT_EQUAL_UINT8(1U, pio_sim_sm_get_tx_fifo_level(&pio, 0)); /* C still queued */
}

/* Under OUT_STICKY, an OUT whose inline-enable bit is 0 *releases* the pins (clears
 * the SM's output enable) so they fall back to the pull / float — distinct from the
 * non-sticky case, which merely holds the previous value. */
static void test_out_sticky_releases_on_inline_disable(void)
{
    sm_config_set_out_pins(&cfg, 0, 1);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 0, 1, true);
    sm_config_set_out_shift(&cfg, true, true, 2);           /* autopull 2 bits */
    sm_config_set_out_special(&cfg, true, true, 1);         /* sticky + inline en = bit 1 */
    pio_sim_set_pull_level(&pio, (uint64_t)1U << 0, false); /* pull-down on pin 0 */
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_PINS, 2)};
    load_prog(prog, 1);
    pio_sim_sm_put(&pio, 0, 0x3U); /* 0b11: pin bit0=1, enable bit1=1 -> drive high */
    pio_sim_sm_put(&pio, 0, 0x1U); /* 0b01: pin bit0=1, enable bit1=0 -> release     */
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
    sm_config_set_out_pins(&cfg, 0, 1);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 0, 1, true);
    sm_config_set_out_shift(&cfg, true, true, 2);    /* autopull every 2 bits */
    sm_config_set_out_special(&cfg, false, true, 1); /* enable = data bit 1, no sticky */
    const uint16_t prog[] = {pio_sim_encode_out(PIO_DST_PINS, 2)};
    load_prog(prog, 1);
    pio_sim_sm_put(&pio, 0, 0x1U); /* 0b01: enable bit1=0 -> suppressed, pin holds 0 */
    pio_sim_sm_put(&pio, 0, 0x3U); /* 0b11: enable bit1=1 -> driven high            */
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
        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, 0, 2);
        sm_config_set_out_pins(&c, 4, 1);
        sm_config_set_out_shift(&c, true, false, 32);
        pio_sim_sm_init(&pio, s, 0, &c);
        pio_sim_sm_set_consecutive_pindirs(&pio, s, 4, 1, true);
    }
    pio_sim_sm_put(&pio, 0, 0xFFFFFFFFU);
    pio_sim_sm_put(&pio, 1, 0x0U);
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 4)); /* SM1 (higher) wins -> low */

    /* Override: SM1 (sticky) yields via inline enable=0, releasing pin 4, so SM0's
     * high shows (enable = bit 1 of a 1-bit OUT, which is always 0). */
    pio_sim_init(&pio);
    pio_sim_load(&pio, 0, prog, 3);
    for (uint8_t s = 0; s < 2; s++) {
        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, 0, 2);
        sm_config_set_out_pins(&c, 4, 1);
        sm_config_set_out_shift(&c, true, false, 32);
        if (s == 1) { /* sticky + inline: SM1 releases pin 4 */
            sm_config_set_out_special(&c, true, true, 1);
        }
        pio_sim_sm_init(&pio, s, 0, &c);
        pio_sim_sm_set_consecutive_pindirs(&pio, s, 4, 1, true);
    }
    pio_sim_sm_put(&pio, 0, 0xFFFFFFFFU);
    pio_sim_sm_put(&pio, 1, 0xFFFFFFFFU);
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 4)); /* SM1 released -> SM0 high shows */
}

/* Shared helper: SM0 and SM1 both own pin 5 (drive it as output). */
static void set_pin5_dirs_both(void)
{
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 5, 1, true);
    pio_sim_sm_set_consecutive_pindirs(&pio, 1, 5, 1, true);
}

/* Build a SET-pin-5 config with the given wrap, and init `sm` at `pc`. */
static void init_pin5_sm(uint8_t sm, uint8_t pc, uint8_t wrap_lo, uint8_t wrap_hi, bool sticky)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_set_pins(&c, 5, 1);
    sm_config_set_wrap(&c, wrap_lo, wrap_hi);
    if (sticky) {
        sm_config_set_out_special(&c, true, false, 0);
    }
    pio_sim_sm_init(&pio, sm, pc, &c);
}

/* Hardware collates only *simultaneous* writes: when SM0 and SM1 write pin 5 on
 * the same cycle, the higher-numbered SM1 wins (RP2040 §3.5.6.1). */
static void test_pin_priority_same_cycle_higher_sm_wins(void)
{
    const uint16_t sm1prog[] = {pio_sim_encode_set(PIO_DST_PINS, 0)}; /* low  */
    const uint16_t sm0prog[] = {pio_sim_encode_set(PIO_DST_PINS, 1)}; /* high */
    pio_sim_load(&pio, 0, sm1prog, 1);
    pio_sim_load(&pio, 10, sm0prog, 1);
    init_pin5_sm(1, 0, 0, 0, false);
    init_pin5_sm(0, 10, 10, 10, false);
    set_pin5_dirs_both();
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    pio_sim_run(&pio, 1);                        /* both write pin 5 this cycle */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 5)); /* SM1 (higher) wins -> low */
}

/* Without OUT_STICKY a pin write is a one-shot event: SM1 writes pin 5 low, and
 * on a *later* cycle SM0 writes it high with no competing write — SM0's write
 * lands (last writer), regardless of SM numbers. */
static void test_pin_no_sticky_last_writer_wins(void)
{
    /* SM1 writes low on cycle 1 then parks on a jmp loop (no further writes);
     * SM0 nops through cycle 1 and writes high on cycle 2. */
    const uint16_t sm1prog[] = {pio_sim_encode_set(PIO_DST_PINS, 0),
                                pio_sim_encode_jmp(PIO_COND_ALWAYS, 1)};
    const uint16_t sm0prog[] = {pio_sim_encode_nop(), pio_sim_encode_set(PIO_DST_PINS, 1),
                                pio_sim_encode_jmp(PIO_COND_ALWAYS, 12)};
    pio_sim_load(&pio, 0, sm1prog, 2);  /* SM1 at 0..1 */
    pio_sim_load(&pio, 10, sm0prog, 3); /* SM0 at 10..12 */
    init_pin5_sm(1, 0, 1, 1, false);
    init_pin5_sm(0, 10, 12, 12, false);
    set_pin5_dirs_both();
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 5)); /* cycle 1: SM1's low landed */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 5)); /* cycle 2: SM0's uncontested high wins */
}

/* OUT_STICKY re-asserts the SM's driven pins every cycle, so a sticky SM1 keeps
 * winning the per-cycle collation against SM0's later write even while parked. */
static void test_out_sticky_retains_priority_each_cycle(void)
{
    const uint16_t sm1prog[] = {pio_sim_encode_set(PIO_DST_PINS, 0),
                                pio_sim_encode_jmp(PIO_COND_ALWAYS, 1)};
    const uint16_t sm0prog[] = {pio_sim_encode_nop(), pio_sim_encode_set(PIO_DST_PINS, 1),
                                pio_sim_encode_jmp(PIO_COND_ALWAYS, 12)};
    pio_sim_load(&pio, 0, sm1prog, 2);
    pio_sim_load(&pio, 10, sm0prog, 3);
    init_pin5_sm(1, 0, 1, 1, true); /* SM1 sticky */
    init_pin5_sm(0, 10, 12, 12, false);
    set_pin5_dirs_both();
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true);
    pio_sim_run(&pio, 3); /* SM0's high on cycle 2 collides with SM1's sticky low */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 5)); /* sticky SM1 still wins */
}

/* Pin-span counts clamp to 32 (the SM pin window), so an oversized count can
 * never shift a 32-bit value out of range. */
static void test_out_count_clamped_to_32(void)
{
    /* The config mutators clamp the pin span to 32 (the SM pin window). */
    sm_config_set_out_pins(&cfg, 0, 40);
    TEST_ASSERT_EQUAL_UINT8(32U, cfg.out_count);
    sm_config_set_set_pins(&cfg, 0, 33);
    TEST_ASSERT_EQUAL_UINT8(32U, cfg.set_count);
    sm_config_set_out_pin_count(&cfg, 255);
    TEST_ASSERT_EQUAL_UINT8(32U, cfg.out_count);
    sm_config_set_set_pin_count(&cfg, 64);
    TEST_ASSERT_EQUAL_UINT8(32U, cfg.set_count);
    /* And a MOV PINS across the full clamped span executes without tripping
     * undefined shifts (UBSan-guarded in CI). */
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 0, 32, true);
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_X, 21),
                             pio_sim_encode_mov(PIO_DST_PINS, PIO_MOV_NONE, PIO_SRC_X)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 0));
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 1));
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 2));
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 31));
}

/* True high-impedance: a PIO output released to input (no pull) floats and reads 0,
 * not the value it last drove. */
static void test_pin_floats_when_released(void)
{
    sm_config_set_set_pins(&cfg, 7, 1);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 7, 1, true);
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINS, 1)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 7));               /* driven high */
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 7, 1, false); /* release to input, no pull */
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
    sm_config_set_out_shift(&cfg, true, false, 32);
    uint16_t injected = pio_sim_encode_set(PIO_DST_Y, 9);
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_out(PIO_DST_OSR, 16), /* dest OSR == EXEC path (7) */
    };
    load_prog(prog, 2);
    pio_sim_sm_put(&pio, 0, injected);
    pio_sim_run(&pio, 3); /* pull, out-exec arms, injected runs */
    TEST_ASSERT_EQUAL_UINT32(9U, pio.sm[0].y);
}

static void test_out_exec_injected_instruction_honors_delay(void)
{
    /* Unlike a forced pio_sim_sm_exec instruction, an instruction injected via
     * OUT EXEC executes its delay field (RP2040 datasheet §3.4.5.2). */
    sm_config_set_out_shift(&cfg, true, false, 32);
    uint16_t injected = (uint16_t)(pio_sim_encode_set(PIO_DST_X, 7) | (2U << 8U)); /* [2] */
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_out(PIO_DST_OSR, 16), /* dest OSR == EXEC path (7) */
        pio_sim_encode_set(PIO_DST_Y, 5),
    };
    load_prog(prog, 3);
    pio_sim_sm_put(&pio, 0, injected);
    pio_sim_run(&pio, 3); /* pull, out-exec arms, injected `set x,7 [2]` runs */
    TEST_ASSERT_EQUAL_UINT32(7U, pio.sm[0].x);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y); /* delay holds the SM… */
    pio_sim_run(&pio, 2);                      /* …for two more cycles */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].y);
    pio_sim_run(&pio, 1); /* delay elapsed: set y executes */
    TEST_ASSERT_EQUAL_UINT32(5U, pio.sm[0].y);
}

/* ── IN / autopush ─────────────────────────────────────────────────────────── */

static void test_in_pins_shift_left(void)
{
    sm_config_set_in_pins(&cfg, 0);
    sm_config_set_in_shift(&cfg, false, false, 32);
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
    sm_config_set_in_pins(&cfg, 0);
    sm_config_set_in_shift(&cfg, false, true, 8);
    pio_sim_set_pin(&pio, 0, true); /* sample 0x...1 each time */
    const uint16_t prog[] = {pio_sim_encode_in(PIO_SRC_PINS, 8)};
    load_prog(prog, 1);
    pio_sim_sync_settle(&pio); /* static input: skip the 2-cycle settle */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_FALSE(pio_sim_sm_is_rx_fifo_empty(&pio, 0));
    uint32_t w = 0;
    TEST_ASSERT_TRUE(pio_sim_sm_get(&pio, 0, &w));
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
    pio_sim_sm_put(&pio, 0, 0xDEAD);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(0xDEADU, pio.sm[0].osr);
    TEST_ASSERT_EQUAL_UINT32(5U, pio.sm[0].x);
}

static void test_pull_noblock_on_empty_copies_x(void)
{
    const uint16_t prog[] = {
        pio_sim_encode_set(PIO_DST_X, 12),
        /* noblock */
        pio_sim_encode_pull(false, false),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(12U, pio.sm[0].osr); /* OSR <- X on empty noblock */
}

static void test_push_blocks_when_rx_full(void)
{
    sm_config_set_in_shift(&cfg, false, false, 32);
    /* Fill RX (depth 4) then a 5th push stalls. */
    const uint16_t prog[] = {
        pio_sim_encode_in(PIO_SRC_X, 1),
        pio_sim_encode_push(false, true),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 100);
    TEST_ASSERT_TRUE(pio_sim_sm_is_rx_fifo_full(&pio, 0));
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
    sm_config_set_out_shift(&cfg, true, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_pull(false, true),
        pio_sim_encode_mov(PIO_DST_X, PIO_MOV_REVERSE, PIO_SRC_OSR),
    };
    load_prog(prog, 2);
    pio_sim_sm_put(&pio, 0, 0x00000001U);
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
    sm_config_set_jmp_pin(&cfg, 5);
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
    sm_config_set_sideset(&cfg, 1, false, false);
    sm_config_set_sideset_pins(&cfg, 2);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 2, 1, true); /* drive the side-set pin */
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
    sm_config_set_clkdiv_int_frac8(&cfg, 4, 0); /* one SM cycle every 4 ticks */
    load_prog(prog, 1);
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
    sm_config_set_clkdiv_int_frac8(&cfg, 2, 128); /* 2.5 */
    load_prog(prog, 6);
    pio_sim_run(&pio, 5);
    TEST_ASSERT_EQUAL_UINT8(2U, pio_sim_sm_get_pc(&pio, 0));
    pio_sim_run(&pio, 5);
    TEST_ASSERT_EQUAL_UINT8(4U, pio_sim_sm_get_pc(&pio, 0)); /* avg 2.5 over 10 ticks */
}

/* div_int == 0 encodes a divider of 65536 (plus the fraction) — it must not
 * collapse to full speed when div_frac is non-zero. */
static void test_clkdiv_int0_frac_is_65536(void)
{
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_X, 1)};
    sm_config_set_clkdiv_int_frac8(&cfg, 0, 128); /* 65536.5 */
    load_prog(prog, 1);
    pio_sim_run(&pio, 65536);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].x); /* not yet: divider is 65536.5 */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(1U, pio.sm[0].x); /* fires on tick 65537 */
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
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    sm_config_set_in_shift(&cfg, false, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_in(PIO_SRC_X, 1),
        pio_sim_encode_push(false, true),
    };
    load_prog(prog, 2);
    pio_sim_run(&pio, 200);
    TEST_ASSERT_EQUAL_UINT8(8U, pio.sm[0].rx.count);       /* 8-deep when joined */
    TEST_ASSERT_TRUE(pio_sim_sm_is_tx_fifo_full(&pio, 0)); /* TX cap 0 → always full */
}

/* ── Public accessor coverage ──────────────────────────────────────────────── */

static void test_tx_empty_and_irq_clear_accessors(void)
{
    /* TX FIFO empty/non-empty transitions. */
    TEST_ASSERT_TRUE(pio_sim_sm_is_tx_fifo_empty(&pio, 0));
    TEST_ASSERT_TRUE(pio_sim_sm_put(&pio, 0, 0x1234U));
    TEST_ASSERT_FALSE(pio_sim_sm_is_tx_fifo_empty(&pio, 0));

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
    sm_config_set_in_pins(&cfg, 0);
    sm_config_set_in_shift(&cfg, false, true, 8);
    pio_sim_set_pin(&pio, 0, true); /* each sample = 0x01 */
    const uint16_t prog[] = {pio_sim_encode_in(PIO_SRC_PINS, 8)};
    load_prog(prog, 1);
    pio_sim_sync_settle(&pio); /* static input: skip the 2-cycle settle */

    /* Four pushes fill the 4-deep RX FIFO; the next IN must stall. */
    pio_sim_run(&pio, 4);
    TEST_ASSERT_TRUE(pio_sim_sm_is_rx_fifo_full(&pio, 0));

    /* Many stalled cycles: the ISR must stay empty, not accumulate reshifts. */
    pio_sim_run(&pio, 20);
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].isr);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].isr_count);

    /* Freeing a slot lets exactly one fresh sample through, uncorrupted. */
    uint32_t w = 0;
    TEST_ASSERT_TRUE(pio_sim_sm_get(&pio, 0, &w));
    TEST_ASSERT_EQUAL_UINT32(0x01U, w);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_sm_is_rx_fifo_full(&pio, 0));
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

/* An `irq n wait` injected while the SM is already stalled (e.g. parked on a
 * blocking PULL) must still raise its flag on first presentation — the stale
 * program stall must not suppress it, or the wait deadlocks forever. */
static void test_sm_exec_irq_wait_from_stalled_sm(void)
{
    const uint16_t prog[] = {pio_sim_encode_pull(false, true), pio_sim_encode_set(PIO_DST_Y, 1)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 1); /* PULL stalls on the empty TX FIFO */
    TEST_ASSERT_TRUE(pio_sim_sm_is_stalled(&pio, 0));

    pio_sim_sm_exec(&pio, 0, pio_sim_encode_irq(false, true, 1)); /* irq 1 ; wait */
    TEST_ASSERT_TRUE(pio_sim_irq_get(&pio, 1)); /* raised despite the prior stall */

    pio_sim_irq_clear(&pio, 1); /* acknowledge: the injected wait can commit */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_FALSE(pio.sm[0].exec_pending);

    pio_sim_sm_put(&pio, 0, 0x123U); /* program resumes on its own PULL */
    pio_sim_run(&pio, 2);
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
    pio_sim_sm_put(&pio, 0, 0x55U);
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
    sm_config_set_sideset(&cfg, 1, false, false); /* 1 data bit, no opt */
    sm_config_set_sideset_pins(&cfg, 2);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 2, 1, true);
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
    sm_config_set_mov_status(&cfg, PIO_STATUS_TX_LEVEL, 2);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_STATUS)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, pio.sm[0].x); /* TX level 0 < 2 */
    pio_sim_sm_put(&pio, 0, 0);
    pio_sim_sm_put(&pio, 0, 0);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x); /* TX level 2, not < 2 */
}

static void test_mov_status_rx_level(void)
{
    sm_config_set_mov_status(&cfg, PIO_STATUS_RX_LEVEL, 1);
    sm_config_set_in_shift(&cfg, false, true, 1); /* autopush at 1 bit */
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_STATUS)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, pio.sm[0].x); /* RX empty: 0 < 1 */
    /* Autopush a word into RX, then STATUS must read all-zeros (level 1). */
    pio_sim_sm_exec(&pio, 0, pio_sim_encode_in(PIO_SRC_NULL, 1));
    TEST_ASSERT_FALSE(pio_sim_sm_is_rx_fifo_empty(&pio, 0));
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x);
}

#if PIO_SIM_HAS_IRQ_STATUS
static void test_mov_status_irq_flag(void)
{
    sm_config_set_mov_status(&cfg, PIO_STATUS_IRQ_SET, 3);
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
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_STATUS)};
    load_prog(prog, 1);
    pio_sim_sm_set_status_value(&pio, 0, 0xA5A5A5A5U); /* runtime override, after init */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xA5A5A5A5U, pio.sm[0].x);
}

/* ── #6: SM-relative IRQ indexing ──────────────────────────────────────────── */

static void test_irq_rel_maps_per_sm(void)
{
    /* `irq 0 rel` (set form): SM0 → flag 0, SM1 → flag 1. */
    const uint16_t prog[] = {pio_sim_encode_irq_rel(false, false, 0)};
    pio_sim_load(&pio, 0, prog, 1);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, 0, 0);
    for (uint8_t s = 0; s < 2U; s++) {
        pio_sim_sm_init(&pio, s, 0, &c);
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
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, 0, 0);
    pio_sim_sm_init(&pio, 3, 0, &c);
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
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, 0, 1);
    pio_sim_sm_init(&pio, 1, 0, &c);
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
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, 0, 0);
    sm_config_set_clkdiv_int_frac8(&c, 4, 0); /* one SM cycle every 4 ticks */
    for (uint8_t s = 0; s < 2U; s++) {
        pio_sim_sm_init(&pio, s, 0, &c);
    }
    pio_sim_set_sm_mask_enabled(&pio, 0x3U, true); /* enable both, dividers aligned */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_UINT32(pio.sm[0].clk_accum, pio.sm[1].clk_accum); /* lockstep */

    /* Restart only SM0 → its divider phase resets while SM1's keeps running. */
    pio_sim_clkdiv_restart_sm_mask(&pio, 0x1U);
    TEST_ASSERT_TRUE(pio.sm[0].clk_accum != pio.sm[1].clk_accum);

    /* Re-align both: equal phase again, and they step together thereafter. */
    pio_sim_clkdiv_restart_sm_mask(&pio, 0x3U);
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
    sm_config_set_wrap(&cfg, 0, 0);
    sm_config_set_clkdiv_int_frac8(&cfg, 4, 0);
    pio_sim_sm_init(&pio, 0, 0, &cfg); /* apply config; SM left disabled */
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_UINT32(768U, pio.sm[0].clk_accum);     /* divider free-ran */
    TEST_ASSERT_EQUAL_UINT8(0U, pio_sim_sm_get_pc(&pio, 0)); /* but did not execute */
    TEST_ASSERT_EQUAL_UINT32(0U, pio.sm[0].x);
}

/* ── #7: RP2350 instructions ───────────────────────────────────────────────── */

#if PIO_SIM_HAS_RXFIFO_MOV
static void test_mov_rxfifo_roundtrip(void)
{
    /* PUTGET (both FJOIN bits): the SM may randomly put *and* get. */
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX_PUTGET);
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
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX_PUT);
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
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX_PUT);
    /* PUT: SM writes rxfifo[2], host reads it by index. */
    const uint16_t put[] = {pio_sim_encode_mov_to_rxfifo(2)};
    load_prog(put, 1);
    pio.sm[0].isr = 0xCAFEF00DU;
    pio.sm[0].isr_count = 32;
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0xCAFEF00DU, pio_sim_rxfifo_get(&pio, 0, 2));

    /* GET: host writes rxfifo[1], SM reads it into OSR. */
    pio_sim_init(&pio); /* fresh SM */
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX_GET);
    pio_sim_rxfifo_put(&pio, 0, 1, 0x0BADC0DEU);
    const uint16_t get[] = {pio_sim_encode_mov_from_rxfifo(1)};
    load_prog(get, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0BADC0DEU, pio.sm[0].osr);
}
/* The RX register-file direction restrictions are enforced: an SM get is a
 * no-op outside GET/PUTGET mode, an SM put a no-op outside PUT/PUTGET mode. */
static void test_rxfifo_mode_enforced(void)
{
    /* PUT mode: an SM get must not load the OSR. */
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX_PUT);
    const uint16_t get[] = {pio_sim_encode_mov_from_rxfifo(1)};
    load_prog(get, 1);
    pio.sm[0].rx.buf[1] = 0x11111111U; /* after init (which cleared the RX file) */
    pio.sm[0].osr = 0x0U;
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].osr); /* get rejected in PUT mode */

    /* GET mode: an SM put must not write the register file. */
    pio_sim_init(&pio);
    cfg = pio_get_default_sm_config();
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX_GET);
    const uint16_t put[] = {pio_sim_encode_mov_to_rxfifo(1)};
    load_prog(put, 1);
    pio.sm[0].isr = 0x22222222U;
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].rx.buf[1]);  /* put rejected in GET mode */
    TEST_ASSERT_EQUAL_HEX32(0x22222222U, pio.sm[0].isr); /* ISR untouched */
}

/* Operand bit 4 discriminates the indexed RX-FIFO MOV from PUSH/PULL: a
 * PUSH/PULL word with only reserved low bits set must still execute as a
 * PUSH/PULL, not be misrouted to the RX register file. */
static void test_mov_rxfifo_discriminator_bit4(void)
{
    sm_config_set_in_shift(&cfg, false, false, 32);
    const uint16_t prog[] = {
        pio_sim_encode_in(PIO_SRC_X, 1),
        (uint16_t)(((uint32_t)PIO_OP_PUSHPULL << 13U) | 0x20U | 0x01U), /* push block + rsvd bit0 */
    };
    load_prog(prog, 2);
    pio.sm[0].x = 1;
    pio_sim_run(&pio, 2);
    TEST_ASSERT_FALSE(pio_sim_sm_is_rx_fifo_empty(&pio, 0)); /* pushed as a normal PUSH */
    uint32_t w = 0;
    TEST_ASSERT_TRUE(pio_sim_sm_get(&pio, 0, &w));
    TEST_ASSERT_EQUAL_UINT32(1U, w);
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
    sm_config_set_jmp_pin(&cfg, 7);
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
    sm_config_set_jmp_pin(&cfg, 4);
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
    sm_config_set_out_pins(&cfg, 0, 3); /* OUT/MOV pins map to GPIO 0..2 */
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
    sm_config_set_in_pins(&cfg, 0);
    sm_config_set_in_shift(&cfg, false, false, 32);
    sm_config_set_in_pin_count(&cfg, 5); /* only pins 0..4 visible */
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
    sm_config_set_in_pins(&cfg, 0);
    sm_config_set_in_pin_count(&cfg, 5);
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

    sm_config_set_in_pin_count(&cfg, 8);  /* now pin 6 is visible */
    pio_sim_sm_set_config(&pio, 0, &cfg); /* apply the new IN count in place */
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
    sm_config_set_set_pins(&cfg, 2, 1);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 2, 1, true); /* view pin 2 → drives GPIO 18 */
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

#if PIO_SIM_HAS_IRQ_STATUS
static void test_mov_status_irq_prev_next_blocks(void)
{
    /* STATUS_SEL=IRQ with STATUS_N[4:3] prev/next reads the neighbouring PIO
     * block's flag; the local flag must not leak in, and an unlinked
     * neighbour reads as clear. */
    pio_sim_t nbr;
    pio_sim_init(&nbr);
    pio_sim_set_irq_neighbors(&pio, &nbr, &nbr); /* nbr is both prev and next */

    sm_config_set_mov_status(&cfg, PIO_STATUS_IRQ_SET_NEXT, 3);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_STATUS),
                             pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    load_prog(prog, 2);
    sm_config_set_wrap(&cfg, 0, 1);

    pio.irq |= (uint8_t)(1U << 3U); /* LOCAL flag: must not affect prev/next */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x);

    nbr.irq |= (uint8_t)(1U << 3U); /* neighbour flag: selects all-ones */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, pio.sm[0].x);

    sm_config_set_mov_status(&cfg, PIO_STATUS_IRQ_SET_PREV, 3);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, pio.sm[0].x);

    pio_sim_set_irq_neighbors(&pio, NULL, NULL); /* unlinked: reads clear */
    pio_sim_run(&pio, 2);
    TEST_ASSERT_EQUAL_HEX32(0x0U, pio.sm[0].x);
}
#endif
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

    pio_sm_config ca = pio_get_default_sm_config();
    sm_config_set_set_pins(&ca, 0, 1);
    sm_config_set_wrap(&ca, 0, 1);
    const uint16_t aprog[] = {pio_sim_encode_set(PIO_DST_PINS, 1),
                              pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&a, 0, aprog, 2);
    pio_sim_sm_init(&a, 0, 0, &ca);
    pio_sim_sm_set_consecutive_pindirs(&a, 0, 0, 1, true); /* A drives GPIO 0 as output */
    pio_sim_sm_set_enabled(&a, 0, true);

    pio_sm_config cb = pio_get_default_sm_config();
    sm_config_set_in_pins(&cb, 0);
    sm_config_set_wrap(&cb, 0, 1);
    const uint16_t bprog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_PINS),
                              pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&b, 0, bprog, 2);
    pio_sim_sm_init(&b, 0, 0, &cb);
    pio_sim_sm_set_enabled(&b, 0, true);

    pio_sim_group_run(&g, 6); /* A drives, shared synchroniser settles, B samples */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&a, 0));
    TEST_ASSERT_TRUE((b.sm[0].x & 1U) != 0U);
}

/* Regression: with shared pads, a block's previous-tick pin writes must not
 * compete for same-cycle priority during a later block's tick of the next
 * cycle. B (the later, higher-priority owner) writes GPIO 0 high on tick 1
 * only; A writes it low on tick 2. A's on_tick hook — which fires after A's
 * own resolve but before B's tick — must see A's write win on tick 2, because
 * B wrote nothing that cycle. */
static uint8_t group_seen_levels[8];
static uint8_t group_seen_count;

static void group_record_pin0(pio_sim_t *p, void *ctx)
{
    (void)ctx;
    if (group_seen_count < 8U) {
        group_seen_levels[group_seen_count] = pio_sim_get_pin(p, 0) ? 1U : 0U;
    }
    group_seen_count++;
}

static void test_group_stale_write_does_not_win_priority(void)
{
    pio_sim_t a;
    pio_sim_t b;
    pio_sim_init(&a);
    pio_sim_init(&b);
    pio_sim_t *blocks[] = {&a, &b};
    pio_sim_group_t g;
    pio_sim_group_init_shared(&g, blocks, 2);

    const uint16_t nop = pio_sim_encode_mov(PIO_DST_Y, PIO_MOV_NONE, PIO_SRC_Y);

    /* B: tick 1 sets GPIO 0 high, then parks on a nop self-loop. */
    pio_sm_config cb = pio_get_default_sm_config();
    sm_config_set_set_pins(&cb, 0, 1);
    sm_config_set_wrap(&cb, 0, 2);
    const uint16_t bprog[] = {pio_sim_encode_set(PIO_DST_PINS, 1), nop,
                              pio_sim_encode_jmp(PIO_COND_ALWAYS, 1)};
    pio_sim_load(&b, 0, bprog, 3);
    pio_sim_sm_init(&b, 0, 0, &cb);
    pio_sim_sm_set_consecutive_pindirs(&b, 0, 0, 1, true);
    pio_sim_sm_set_enabled(&b, 0, true);

    /* A: tick 1 is a nop, tick 2 drives GPIO 0 low, then parks. */
    pio_sm_config ca = pio_get_default_sm_config();
    sm_config_set_set_pins(&ca, 0, 1);
    sm_config_set_wrap(&ca, 0, 2);
    const uint16_t aprog[] = {nop, pio_sim_encode_set(PIO_DST_PINS, 0),
                              pio_sim_encode_jmp(PIO_COND_ALWAYS, 2)};
    pio_sim_load(&a, 0, aprog, 3);
    pio_sim_sm_init(&a, 0, 0, &ca);
    pio_sim_sm_set_consecutive_pindirs(&a, 0, 0, 1, true);
    pio_sim_sm_set_enabled(&a, 0, true);

    group_seen_count = 0;
    pio_sim_set_device(&a, group_record_pin0, NULL);

    pio_sim_group_run(&g, 3);
    TEST_ASSERT_EQUAL_UINT8(3, group_seen_count);
    /* Tick 1: B's set lands after A's hook ran, so A saw the idle level. */
    TEST_ASSERT_EQUAL_UINT8(0, group_seen_levels[0]);
    /* Tick 2: only A wrote this cycle — its low write must win even though
     * B is the higher-priority owner (B's tick-1 write is stale). */
    TEST_ASSERT_EQUAL_UINT8(0, group_seen_levels[1]);
    /* Tick 3: nobody writes; the pad holds A's latched level. */
    TEST_ASSERT_EQUAL_UINT8(0, group_seen_levels[2]);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&a, 0));
}

static void test_pull_open_drain(void)
{
    /* GPIO 3 pulled high: reads high while released, low while a SM drives it. */
    pio_sim_set_pull_level(&pio, (uint64_t)1U << 3U, true);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 3)); /* undriven -> pull high */

    sm_config_set_set_pins(&cfg, 3, 1);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, 3, 1, true); /* drive as output */
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_PINS, 0), pio_sim_encode_jmp(0, 1)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 3)); /* driven low */
}

static void test_input_sync_bypass(void)
{
    /* With the synchroniser bypassed, a pin change is seen the same cycle. */
    sm_config_set_jmp_pin(&cfg, 6);
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
        TEST_ASSERT_TRUE(pio_sim_sm_put(&pio, 0, i));
    }
    TEST_ASSERT_EQUAL_UINT8(4U, pio_sim_sm_get_tx_fifo_level(&pio, 0));
    TEST_ASSERT_FALSE(pio_sim_sm_put(&pio, 0, 99U));
    TEST_ASSERT_TRUE((pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_TXOVER) != 0U);

    /* Host underflow -> RXUNDER. */
    uint32_t w;
    TEST_ASSERT_FALSE(pio_sim_sm_get(&pio, 0, &w));
    TEST_ASSERT_TRUE((pio_sim_get_fdebug(&pio, 0) & PIO_FDEBUG_RXUNDER) != 0U);
    TEST_ASSERT_EQUAL_UINT8(0U, pio_sim_sm_get_rx_fifo_level(&pio, 0));
}

/* ── System interrupt lines (IRQ0 / IRQ1) ──────────────────────────────────── */

static void test_system_irq_lines(void)
{
    /* A fresh SM has a non-full TX FIFO, so TXNFULL is a live INTR source. */
    TEST_ASSERT_TRUE((pio_sim_get_irq_raw(&pio) & PIO_INTR_SM_TXNFULL(0)) != 0U);
    TEST_ASSERT_FALSE(pio_sim_interrupt_line(&pio, 0)); /* nothing enabled yet */
    pio_sim_set_irqn_source_mask_enabled(&pio, 0, PIO_INTR_SM_TXNFULL(0), true);
    TEST_ASSERT_TRUE(pio_sim_interrupt_line(&pio, 0));

    /* An SM IRQ flag drives line 1 once enabled. */
    pio_sim_set_irqn_source_mask_enabled(&pio, 1, PIO_INTR_SM_IRQ(2), true);
    TEST_ASSERT_FALSE(pio_sim_interrupt_line(&pio, 1));
    const uint16_t prog[] = {pio_sim_encode_irq(false, false, 2)};
    load_prog(prog, 1);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_interrupt_line(&pio, 1));

    /* INTF forces a line regardless of sources/enables. */
    pio_sim_set_irqn_source_mask_enabled(&pio, 0, PIO_INTR_SM_TXNFULL(0), false);
    TEST_ASSERT_FALSE(pio_sim_interrupt_line(&pio, 0));
    pio_sim_set_irq_force(&pio, 0, PIO_INTR_SM_IRQ(0), true);
    TEST_ASSERT_TRUE(pio_sim_interrupt_line(&pio, 0));
}

/* IRQ-flag routing into INTR: RP2350 routes all eight SM IRQ flags (INTR bits
 * 8..15); RP2040 routes only the low four. */
static void test_intr_irq_flag_routing(void)
{
    pio.irq = 0xFFU; /* all eight flags raised */
    uint32_t intr = pio_sim_get_irq_raw(&pio);
#if PIO_SIM_HAS_INTR_IRQ8
    for (uint8_t i = 0; i < 8U; i++) {
        TEST_ASSERT_TRUE((intr & PIO_INTR_SM_IRQ(i)) != 0U);
    }
#else
    for (uint8_t i = 0; i < 4U; i++) {
        TEST_ASSERT_TRUE((intr & PIO_INTR_SM_IRQ(i)) != 0U);
    }
    for (uint8_t i = 4U; i < 8U; i++) {
        TEST_ASSERT_TRUE((intr & PIO_INTR_SM_IRQ(i)) == 0U);
    }
#endif
}

/* The INTE/INTF masks read back exactly as set, per line and independently. */
static void test_irq_enable_force_read_back(void)
{
    TEST_ASSERT_EQUAL_HEX32(0U, pio_sim_get_irq_enable(&pio, 0)); /* default after init */
    TEST_ASSERT_EQUAL_HEX32(0U, pio_sim_get_irq_force(&pio, 1));

    pio_sim_set_irqn_source_mask_enabled(&pio, 0, 0x00F0U, true);
    pio_sim_set_irq_force(&pio, 1, 0x0A00U, true);
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
    sm_config_set_jmp_pin(&cfg, 5);
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
    sm_config_set_jmp_pin(&cfg, 5);
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
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    pio_sim_sm_set_config(&pio, 0, &cfg);
    TEST_ASSERT_EQUAL_UINT8(PIO_SIM_FIFO_MAX, pio.sm[0].rx.cap);
    pio_sim_sm_restart(&pio, 0); /* must not revert the 8-deep join to 4+4 */
    TEST_ASSERT_EQUAL_UINT8(PIO_SIM_FIFO_MAX, pio.sm[0].rx.cap);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].tx.cap);
    TEST_ASSERT_EQUAL_UINT8(0U, pio.sm[0].rx.count); /* contents cleared though */
}

static void test_clear_fifos(void)
{
    pio_sim_sm_put(&pio, 0, 1U);
    pio_sim_sm_put(&pio, 0, 2U);
    TEST_ASSERT_FALSE(pio_sim_sm_is_tx_fifo_empty(&pio, 0));
    pio_sim_sm_clear_fifos(&pio, 0);
    TEST_ASSERT_TRUE(pio_sim_sm_is_tx_fifo_empty(&pio, 0));
    TEST_ASSERT_TRUE(pio_sim_sm_is_rx_fifo_empty(&pio, 0));
}

static void test_group_enable_sm_mask_sync(void)
{
    pio_sim_t a, b;
    pio_sim_init(&a);
    pio_sim_init(&b);
    const uint16_t prog[] = {pio_sim_encode_nop(), pio_sim_encode_nop(), pio_sim_encode_nop()};
    pio_sm_config c3 = pio_get_default_sm_config();
    sm_config_set_wrap(&c3, 0, 2);
    pio_sim_load(&a, 0, prog, 3);
    pio_sim_load(&b, 0, prog, 3);
    pio_sim_sm_init(&a, 0, 0, &c3);
    pio_sim_sm_init(&b, 0, 0, &c3);
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
    sm_config_set_wrap(&cfg, 0, PIO_SIM_INSN_COUNT - 1U);
    pio_sim_sm_set_pc(&pio, 0, 5);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio.unwritten_fetches > 0U);
}

/* A caller-poked PC past the 32-word instruction memory must not index out of
 * bounds: pio_sim_sm_set_pc keeps it in range and stepping stays safe (the
 * sanitizer CI build turns any OOB access here into a failure). */
static void test_set_pc_out_of_range_is_bounded(void)
{
    const uint16_t prog[] = {pio_sim_encode_set(PIO_DST_X, 1)};
    load_prog(prog, 1);
    pio_sim_sm_set_pc(&pio, 0, 200);
    TEST_ASSERT_TRUE(pio_sim_sm_get_pc(&pio, 0) < PIO_SIM_INSN_COUNT);
    (void)pio_sim_sm_get_instr(&pio, 0); /* fetch must stay in bounds */
    pio_sim_run(&pio, 2);                /* stepping must not read OOB */
}

/* sm_config_set_clkdiv must not invoke float→int UB for out-of-range divisors
 * (negative / NaN / >= 65536): they clamp into the valid range. */
static void test_clkdiv_float_out_of_range_clamped(void)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_clkdiv(&c, 0.0F); /* below 1 → clamped to 1 */
    TEST_ASSERT_EQUAL_UINT16(1U, c.clkdiv_int);
    sm_config_set_clkdiv(&c, -5.0F); /* negative → clamped to 1 */
    TEST_ASSERT_EQUAL_UINT16(1U, c.clkdiv_int);
    sm_config_set_clkdiv(&c, 1.0e9F); /* huge → clamped, no UB */
    TEST_ASSERT_TRUE(c.clkdiv_int >= 1U);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_set_pc_out_of_range_is_bounded);
    RUN_TEST(test_clkdiv_float_out_of_range_clamped);
    RUN_TEST(test_set_x_and_y);
    RUN_TEST(test_set_pins_drives_pads);
    RUN_TEST(test_set_pindirs);

    RUN_TEST(test_out_pins_shift_right);
    RUN_TEST(test_out_shift_left_takes_top_bits);
    RUN_TEST(test_out_autopull_refills);
    RUN_TEST(test_out_autopull_background_refill);
    RUN_TEST(test_out_autopull_streams_word_per_cycle);
    RUN_TEST(test_out_autopull_empty_then_fed_stalls_one_extra_cycle);
    RUN_TEST(test_autopull_background_refill_without_out);
    RUN_TEST(test_register_accessors_read_back);
    RUN_TEST(test_trace_hook_fires_on_commit_only);
    RUN_TEST(test_run_until_tx_drained_waits_for_osr);
    RUN_TEST(test_pull_noop_when_autopull_osr_full);
    RUN_TEST(test_out_sticky_releases_on_inline_disable);
    RUN_TEST(test_out_inline_enable_gates_write);
    RUN_TEST(test_multi_sm_pin_priority_and_override);
    RUN_TEST(test_pin_priority_same_cycle_higher_sm_wins);
    RUN_TEST(test_pin_no_sticky_last_writer_wins);
    RUN_TEST(test_out_sticky_retains_priority_each_cycle);
    RUN_TEST(test_out_count_clamped_to_32);
    RUN_TEST(test_pin_floats_when_released);
    RUN_TEST(test_host_release_pin);
    RUN_TEST(test_out_exec_injects_instruction);
    RUN_TEST(test_out_exec_injected_instruction_honors_delay);

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
    RUN_TEST(test_clkdiv_int0_frac_is_65536);
    RUN_TEST(test_wrap_returns_to_bottom);

    RUN_TEST(test_fifo_join_rx_depth_8);
    RUN_TEST(test_tx_empty_and_irq_clear_accessors);

    RUN_TEST(test_in_autopush_full_does_not_reshift);
    RUN_TEST(test_irq_wait_parks_until_cleared);
    RUN_TEST(test_sm_exec_irq_wait_does_not_rearm);
    RUN_TEST(test_sm_exec_irq_wait_from_stalled_sm);
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
    RUN_TEST(test_rxfifo_mode_enforced);
    RUN_TEST(test_mov_rxfifo_discriminator_bit4);
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
#if PIO_SIM_HAS_IRQ_STATUS
    RUN_TEST(test_mov_status_irq_prev_next_blocks);
#endif
#endif
    RUN_TEST(test_group_runs_blocks_in_lockstep);
#if PIO_SIM_HAS_IRQ_PREVNEXT
    RUN_TEST(test_group_wires_irq_ring);
#endif
    RUN_TEST(test_group_shared_pads);
    RUN_TEST(test_group_stale_write_does_not_win_priority);
    RUN_TEST(test_pull_open_drain);
    RUN_TEST(test_input_sync_bypass);
    RUN_TEST(test_sm_mask_enabled);
    RUN_TEST(test_fdebug_flags);
    RUN_TEST(test_system_irq_lines);
    RUN_TEST(test_intr_irq_flag_routing);
    RUN_TEST(test_irq_enable_force_read_back);

    RUN_TEST(test_input_sync_delays_jmp_pin_by_two_cycles);
    RUN_TEST(test_sync_settle_makes_static_input_immediate);

    RUN_TEST(test_fifo_join_survives_restart);
    RUN_TEST(test_clear_fifos);
    RUN_TEST(test_group_enable_sm_mask_sync);
    RUN_TEST(test_unwritten_fetch_is_flagged);

    return UNITY_END();
}
