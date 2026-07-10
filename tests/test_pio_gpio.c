/* test_pio_gpio.c — unit tests for the PADS_BANK0 pad register model and the
 * IO_BANK0 FUNCSEL mux / overrides (pio_gpio.h). */

#include "unity.h"

#include "pio_gpio.h"
#include "pio_sim.h"

static pio_sim_t pio;

static pio_sm_config cfg;

void setUp(void)
{
    pio_sim_init(&pio);
    cfg = pio_get_default_sm_config();
}
void tearDown(void) { (void)0; }

/* Load a program at offset 0, apply cfg (spanning wrap unless already set),
 * init + enable SM0. */
static void load_prog(const uint16_t *prog, uint8_t n)
{
    pio_sim_load(&pio, 0, prog, n);
    if ((cfg.wrap_bottom == 0U) && (cfg.wrap_top == PIO_SIM_INSN_COUNT - 1U)) {
        sm_config_set_wrap(&cfg, 0, (uint8_t)(n - 1U));
    }
    pio_sim_sm_init(&pio, 0, 0, &cfg);
    pio_sim_sm_set_enabled(&pio, 0, true);
}

/* SM0 drives GPIO `pin` high forever. */
static void drive_pin_high(pio_sim_t *p, uint8_t pin)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_wrap(&c, 0, 1);
    static uint16_t prog[2];
    prog[0] = pio_sim_encode_set(PIO_DST_PINS, 1);
    prog[1] = pio_sim_encode_jmp(PIO_COND_ALWAYS, 1);
    pio_sim_load(p, 0, prog, 2);
    pio_sim_sm_init(p, 0, 0, &c);
    pio_sim_sm_set_consecutive_pindirs(p, 0, pin, 1, true);
    pio_sim_sm_set_enabled(p, 0, true);
}

/* ── Pad registers ─────────────────────────────────────────────────────────── */

static void test_ie_zero_sm_reads_zero_but_host_sees_wire(void)
{
    pio_sim_set_pin(&pio, 4, true); /* external drive high */
    pio_sim_pad_set_input_enable(&pio, 4, false);
    sm_config_set_in_pins(&cfg, 4);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_PINS),
                             pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 4);                                    /* synchroniser settled */
    TEST_ASSERT_EQUAL_UINT32(0U, pio_sim_sm_get_x(&pio, 0)); /* PIO reads 0  */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 4));              /* wire is high */

    pio_sim_pad_set_input_enable(&pio, 4, true);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_EQUAL_UINT32(1U, pio_sim_sm_get_x(&pio, 0)); /* now visible */
}

static void test_od_blocks_chip_drive_pull_shows_through(void)
{
    drive_pin_high(&pio, 2);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 2));

    pio_sim_pad_set_output_disable(&pio, 2, true);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 2));           /* pad released: floats 0 */
    TEST_ASSERT_FALSE(pio_sim_pin_is_pio_output(&pio, 2)); /* not driven any more    */

    pio_sim_pad_set_pulls(&pio, 2, true, false); /* pull-up shows through OD */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 2));

    pio_sim_pad_set_output_disable(&pio, 2, false);
    pio_sim_pad_set_pulls(&pio, 2, false, false);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 2)); /* chip drive reconnected */
}

static void test_pulls_up_down_and_float(void)
{
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 7)); /* floats: reads 0 */
    pio_sim_pad_set_pulls(&pio, 7, true, false);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 7)); /* pull-up: 1 */
    pio_sim_pad_set_pulls(&pio, 7, false, true);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 7)); /* pull-down: 0 */
}

static void test_buskeep_holds_last_driven_level(void)
{
    pio_sim_pad_set_pulls(&pio, 3, true, true); /* both = bus keeper */
    pio_sim_set_pin(&pio, 3, true);             /* drive high */
    pio_sim_run(&pio, 1);                       /* keeper latches */
    pio_sim_release_pin(&pio, 3);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 3)); /* held high */

    pio_sim_set_pin(&pio, 3, false); /* drive low */
    pio_sim_run(&pio, 1);
    pio_sim_release_pin(&pio, 3);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 3)); /* held low */
}

static void test_analog_fields_stored_noop(void)
{
    pio_sim_set_pin(&pio, 9, true);
    pio_sim_pad_set_drive(&pio, 9, 3); /* 12mA */
    pio_sim_pad_set_slew_fast(&pio, 9, true);
    pio_sim_pad_set_schmitt(&pio, 9, true);
    TEST_ASSERT_EQUAL_UINT8(3U, pio_sim_pad_get_drive(&pio, 9));
    TEST_ASSERT_TRUE(pio_sim_pad_get_slew_fast(&pio, 9));
    TEST_ASSERT_TRUE(pio_sim_pad_get_schmitt(&pio, 9));
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 9)); /* behaviour unchanged */
}

/* The functional pad fields (IE / OD / pulls) read back through their getters,
 * symmetric with the analog getters. */
static void test_functional_pad_fields_read_back(void)
{
    pio_sim_pad_set_input_enable(&pio, 5, false);
    TEST_ASSERT_FALSE(pio_sim_pad_get_input_enable(&pio, 5));
    pio_sim_pad_set_input_enable(&pio, 5, true);
    TEST_ASSERT_TRUE(pio_sim_pad_get_input_enable(&pio, 5));

    pio_sim_pad_set_output_disable(&pio, 5, true);
    TEST_ASSERT_TRUE(pio_sim_pad_get_output_disable(&pio, 5));

    pio_sim_pad_set_pulls(&pio, 5, true, false);
    bool up = false;
    bool down = true;
    pio_sim_pad_get_pulls(&pio, 5, &up, &down);
    TEST_ASSERT_TRUE(up);
    TEST_ASSERT_FALSE(down);
    pio_sim_pad_get_pulls(&pio, 5, NULL, NULL); /* NULL out-params tolerated */

    /* FUNCSEL read-back accessor. */
    pio_sim_gpio_set_function(&pio, 5, PIO_GPIO_FUNC_PIO0);
    TEST_ASSERT_EQUAL_INT(PIO_GPIO_FUNC_PIO0, pio_sim_gpio_get_function(&pio, 5));
    pio_sim_gpio_set_function(&pio, 5, PIO_GPIO_FUNC_NULL);
    TEST_ASSERT_EQUAL_INT(PIO_GPIO_FUNC_NULL, pio_sim_gpio_get_function(&pio, 5));
    /* FUNCSEL is a 5-bit field: a wider value truncates (as on real silicon)
     * rather than reading back as an out-of-enum value. */
    pio_sim_gpio_set_function(&pio, 5, (pio_gpio_func_t)(0x20U | (unsigned)PIO_GPIO_FUNC_SIO));
    TEST_ASSERT_EQUAL_INT(PIO_GPIO_FUNC_SIO, pio_sim_gpio_get_function(&pio, 5));
}

static void test_pads_reset_hw_matches_datasheet(void)
{
    pio_sim_pads_reset_hw(&pio);
    TEST_ASSERT_EQUAL_UINT8(1U, pio_sim_pad_get_drive(&pio, 0)); /* 4mA   */
    TEST_ASSERT_TRUE(pio_sim_pad_get_schmitt(&pio, 0));
    TEST_ASSERT_FALSE(pio_sim_pad_get_slew_fast(&pio, 0));
#if PIO_SIM_HAS_PAD_ISO
    /* Datasheet reset isolates the pads: even an external drive is the wire's
     * only driver, and PIO inputs are gated until ISO is released. */
    pio_sim_pad_set_iso(&pio, 5, false);
#endif
    /* PDE=1 at reset: an undriven pin reads low (not floating-undefined). */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 5));
    pio_sim_set_pin(&pio, 5, true);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 5)); /* ext drive beats the pull */
}

#if PIO_SIM_HAS_PAD_ISO
static void test_iso_freezes_output_and_gates_input(void)
{
    drive_pin_high(&pio, 6);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 6));

    pio_sim_pad_set_iso(&pio, 6, true); /* latch: driven high */
    /* Remove the live drive behind the latch — the pad must stay frozen. */
    pio_sim_gpio_set_function(&pio, 6, PIO_GPIO_FUNC_NULL);
    pio_sim_run(&pio, 1);                       /* resolve drops the PIO's routing */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 6)); /* frozen at latched level */

    pio_sim_pad_set_iso(&pio, 6, false); /* release: live (no) drive resumes */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 6));
}
#endif

/* ── FUNCSEL mux ───────────────────────────────────────────────────────────── */

static void test_func_null_blocks_pio_output(void)
{
    drive_pin_high(&pio, 1);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 1));

    pio_sim_gpio_set_function(&pio, 1, PIO_GPIO_FUNC_NULL);
    pio_sim_run(&pio, 1); /* next resolve drops the PIO's drive */
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 1));
    TEST_ASSERT_FALSE(pio_sim_pin_is_pio_output(&pio, 1));

    pio_sim_gpio_set_function(&pio, 1, PIO_GPIO_FUNC_PIO0);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 1)); /* routed again (owner slot 0) */
}

static void test_funcsel_selects_between_shared_blocks(void)
{
    /* Two blocks share pads and both drive GPIO 0 with opposite levels; the
     * pin follows whichever block FUNCSEL routes to the pad. */
    pio_sim_t a;
    pio_sim_t b;
    pio_sim_init(&a);
    pio_sim_init(&b);
    pio_sim_t *blocks[] = {&a, &b};
    pio_sim_group_t g;
    pio_sim_group_init_shared(&g, blocks, 2);

    /* a (slot 0 = PIO0) drives high; b (slot 1 = PIO1) drives low. Both use
     * OUT_STICKY so they re-assert every cycle and keep competing. */
    pio_sm_config ca = pio_get_default_sm_config();
    sm_config_set_set_pins(&ca, 0, 1);
    sm_config_set_wrap(&ca, 0, 1);
    sm_config_set_out_special(&ca, true, false, 0);
    const uint16_t aprog[] = {pio_sim_encode_set(PIO_DST_PINS, 1),
                              pio_sim_encode_jmp(PIO_COND_ALWAYS, 1)};
    pio_sim_load(&a, 0, aprog, 2);
    pio_sim_sm_init(&a, 0, 0, &ca);
    pio_sim_sm_set_consecutive_pindirs(&a, 0, 0, 1, true);
    pio_sim_sm_set_enabled(&a, 0, true);

    pio_sm_config cb = pio_get_default_sm_config();
    sm_config_set_set_pins(&cb, 0, 1);
    sm_config_set_wrap(&cb, 0, 1);
    sm_config_set_out_special(&cb, true, false, 0);
    const uint16_t bprog[] = {pio_sim_encode_set(PIO_DST_PINS, 0),
                              pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    pio_sim_load(&b, 0, bprog, 2);
    pio_sim_sm_init(&b, 0, 0, &cb);
    pio_sim_sm_set_consecutive_pindirs(&b, 0, 0, 1, true);
    pio_sim_sm_set_enabled(&b, 0, true);

    pio_sim_gpio_set_function(&a, 0, PIO_GPIO_FUNC_PIO0); /* route to a */
    pio_sim_group_run(&g, 2);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&a, 0));

    pio_sim_gpio_set_function(&a, 0, PIO_GPIO_FUNC_PIO1); /* route to b */
    pio_sim_group_run(&g, 2);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&a, 0));
    TEST_ASSERT_TRUE(pio_sim_pin_is_pio_output(&a, 0)); /* still PIO-driven */
}

static void test_input_visible_when_funcsel_is_sio(void)
{
    /* WAIT PIN completes even though the pin's output routing is SIO: PIO
     * inputs always see the pad. */
    pio_sim_gpio_set_function(&pio, 5, PIO_GPIO_FUNC_SIO);
    sm_config_set_in_pins(&cfg, 5);
    const uint16_t prog[] = {pio_sim_encode_wait(1, PIO_WAIT_PIN, 0),
                             pio_sim_encode_set(PIO_DST_X, 1)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 3);
    TEST_ASSERT_EQUAL_UINT32(0U, pio_sim_sm_get_x(&pio, 0)); /* parked on wait */
    pio_sim_set_pin(&pio, 5, true);
    pio_sim_run(&pio, 4); /* through the synchroniser, wait completes, set runs */
    TEST_ASSERT_EQUAL_UINT32(1U, pio_sim_sm_get_x(&pio, 0));
}

static void test_periph_output_drives_pad_when_selected(void)
{
    pio_sim_gpio_set_periph_output(&pio, 8, true, true);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 8)); /* legacy routing: not selected */

    pio_sim_gpio_set_function(&pio, 8, PIO_GPIO_FUNC_UART);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 8)); /* selected: peripheral drives */

    pio_sim_gpio_set_function(&pio, 8, PIO_GPIO_FUNC_NULL);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 8)); /* deselected again */
}

static void test_outover_inverts_and_forces(void)
{
    drive_pin_high(&pio, 10);
    pio_sim_run(&pio, 2);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 10));

    pio_sim_gpio_set_outover(&pio, 10, PIO_GPIO_OVERRIDE_INVERT);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 10));
    pio_sim_gpio_set_outover(&pio, 10, PIO_GPIO_OVERRIDE_LOW);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 10));
    pio_sim_gpio_set_outover(&pio, 10, PIO_GPIO_OVERRIDE_HIGH);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 10));
    pio_sim_gpio_set_outover(&pio, 10, PIO_GPIO_OVERRIDE_NORMAL);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 10));
}

static void test_oeover_forces_drive_with_no_function(void)
{
    /* No function output on pin 11 at all, but OEOVER=HIGH drives the pad
     * (level comes from the — all zero — mux output). */
    pio_sim_gpio_set_function(&pio, 11, PIO_GPIO_FUNC_NULL);
    pio_sim_pad_set_pulls(&pio, 11, true, false); /* pull-up to show OE wins */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 11));  /* pulled high, undriven */

    pio_sim_gpio_set_oeover(&pio, 11, PIO_GPIO_OVERRIDE_HIGH);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 11)); /* driven low by the pad now */

    pio_sim_gpio_set_oeover(&pio, 11, PIO_GPIO_OVERRIDE_NORMAL);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 11));
}

static void test_inover_inverts_through_sync_and_bypass(void)
{
    /* INOVER=INVERT: an undriven (low) pin reads 1 to the PIO. */
    pio_sim_gpio_set_inover(&pio, 12, PIO_GPIO_OVERRIDE_INVERT);
    sm_config_set_in_pins(&cfg, 12);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_PINS),
                             pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};
    load_prog(prog, 2);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_EQUAL_UINT32(1U, pio_sim_sm_get_x(&pio, 0) & 1U);

    /* Same through the synchroniser bypass. */
    pio_sim_set_input_sync_bypass(&pio, (uint64_t)1U << 12U);
    pio_sim_set_pin(&pio, 12, true); /* wire high → PIO reads 0 (inverted) */
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(0U, pio_sim_sm_get_x(&pio, 0) & 1U);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 12)); /* host still sees the wire */
}

static void test_oeover_low_and_invert_release_the_pad(void)
{
    /* PIO drives pin 13 high; a pull-down reveals when the pad goes hi-Z. */
    drive_pin_high(&pio, 13);
    pio_sim_run(&pio, 2);
    pio_sim_pad_set_pulls(&pio, 13, false, true); /* pull-down */
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 13));  /* still driven high */

    /* OEOVER=LOW forces the output-enable low → hi-Z → pull-down shows. */
    pio_sim_gpio_set_oeover(&pio, 13, PIO_GPIO_OVERRIDE_LOW);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 13));

    /* OEOVER=INVERT flips the (high) function OE to low → hi-Z as well. */
    pio_sim_gpio_set_oeover(&pio, 13, PIO_GPIO_OVERRIDE_INVERT);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&pio, 13));

    pio_sim_gpio_set_oeover(&pio, 13, PIO_GPIO_OVERRIDE_NORMAL);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 13)); /* driven high again */
}

static void test_inover_high_and_low_force_the_read(void)
{
    sm_config_set_in_pins(&cfg, 14);
    const uint16_t prog[] = {pio_sim_encode_mov(PIO_DST_X, PIO_MOV_NONE, PIO_SRC_PINS),
                             pio_sim_encode_jmp(PIO_COND_ALWAYS, 0)};

    /* INOVER=HIGH: the PIO reads 1 even though the wire is low. */
    pio_sim_gpio_set_inover(&pio, 14, PIO_GPIO_OVERRIDE_HIGH);
    load_prog(prog, 2);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_EQUAL_UINT32(1U, pio_sim_sm_get_x(&pio, 0) & 1U);

    /* INOVER=LOW: the PIO reads 0 even though the wire is driven high. */
    pio_sim_gpio_set_inover(&pio, 14, PIO_GPIO_OVERRIDE_LOW);
    pio_sim_set_pin(&pio, 14, true);
    pio_sim_run(&pio, 4);
    TEST_ASSERT_EQUAL_UINT32(0U, pio_sim_sm_get_x(&pio, 0) & 1U);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&pio, 14)); /* host still sees the high wire */
}

#if PIO_SIM_HAS_FUNCSEL_PIO2
static void test_funcsel_pio2_third_block(void)
{
    /* In a 3-block shared group, F8 routes the pad to slot 2 (PIO2). */
    pio_sim_t a;
    pio_sim_t b;
    pio_sim_t c;
    pio_sim_init(&a);
    pio_sim_init(&b);
    pio_sim_init(&c);
    pio_sim_t *blocks[] = {&a, &b, &c};
    pio_sim_group_t g;
    pio_sim_group_init_shared(&g, blocks, 3);

    drive_pin_high(&c, 0);
    pio_sim_gpio_set_function(&a, 0, PIO_GPIO_FUNC_PIO2);
    pio_sim_group_run(&g, 2);
    TEST_ASSERT_TRUE(pio_sim_get_pin(&a, 0));

    pio_sim_gpio_set_function(&a, 0, PIO_GPIO_FUNC_PIO0); /* a drives nothing */
    pio_sim_group_run(&g, 2);
    TEST_ASSERT_FALSE(pio_sim_get_pin(&a, 0));
}
#endif

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ie_zero_sm_reads_zero_but_host_sees_wire);
    RUN_TEST(test_od_blocks_chip_drive_pull_shows_through);
    RUN_TEST(test_pulls_up_down_and_float);
    RUN_TEST(test_buskeep_holds_last_driven_level);
    RUN_TEST(test_analog_fields_stored_noop);
    RUN_TEST(test_functional_pad_fields_read_back);
    RUN_TEST(test_pads_reset_hw_matches_datasheet);
#if PIO_SIM_HAS_PAD_ISO
    RUN_TEST(test_iso_freezes_output_and_gates_input);
#endif
    RUN_TEST(test_func_null_blocks_pio_output);
    RUN_TEST(test_funcsel_selects_between_shared_blocks);
    RUN_TEST(test_input_visible_when_funcsel_is_sio);
    RUN_TEST(test_periph_output_drives_pad_when_selected);
    RUN_TEST(test_outover_inverts_and_forces);
    RUN_TEST(test_oeover_forces_drive_with_no_function);
    RUN_TEST(test_inover_inverts_through_sync_and_bypass);
    RUN_TEST(test_oeover_low_and_invert_release_the_pad);
    RUN_TEST(test_inover_high_and_low_force_the_read);
#if PIO_SIM_HAS_FUNCSEL_PIO2
    RUN_TEST(test_funcsel_pio2_third_block);
#endif
    return UNITY_END();
}
