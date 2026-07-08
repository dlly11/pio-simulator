/* test_pio_clock.c — unit tests for the clk_sys clock-tree model. */

#include "unity.h"

#include "pio_clock.h"

static pio_clk_tree_t clk;

void setUp(void) { pio_clk_init_default(&clk); }
void tearDown(void) { (void)0; }

static void test_default_tree_matches_boot_firmware(void)
{
    TEST_ASSERT_EQUAL_INT(PIO_CLK_OK, pio_clk_validate(&clk));
#if PIO_SIM_PIO_VERSION >= 1
    TEST_ASSERT_EQUAL_UINT64(150000000ULL, pio_clk_sys_hz(&clk)); /* RP2350 */
#else
    TEST_ASSERT_EQUAL_UINT64(125000000ULL, pio_clk_sys_hz(&clk)); /* RP2040 */
#endif
    TEST_ASSERT_EQUAL_UINT64(1500000000ULL, pio_clk_vco_hz(&clk));
}

static void test_pll_rejects_vco_out_of_range(void)
{
    /* Below the per-chip floor: 12 MHz × 16 = 192 MHz VCO. */
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_VCO_RANGE, pio_clk_configure_pll(&clk, 1, 16, 1, 1));
#if PIO_SIM_PIO_VERSION >= 1
    /* 12 × 40 = 480 MHz: legal on RP2350 (min 400), illegal on RP2040. */
    TEST_ASSERT_EQUAL_INT(PIO_CLK_OK, pio_clk_configure_pll(&clk, 1, 40, 4, 1));
#else
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_VCO_RANGE, pio_clk_configure_pll(&clk, 1, 40, 4, 1));
#endif
    /* Above 1600 MHz: 12 × 140 = 1680 MHz. */
    pio_clk_init_default(&clk);
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_VCO_RANGE, pio_clk_configure_pll(&clk, 1, 140, 6, 2));
    /* A rejected configure leaves the tree untouched. */
    TEST_ASSERT_EQUAL_UINT16(125U, clk.pll_fbdiv);
}

static void test_pll_rejects_fbdiv_refdiv_postdiv(void)
{
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_FBDIV_RANGE, pio_clk_configure_pll(&clk, 1, 15, 6, 2));
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_FBDIV_RANGE, pio_clk_configure_pll(&clk, 1, 321, 6, 2));
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_REFDIV_RANGE, pio_clk_configure_pll(&clk, 0, 125, 6, 2));
    /* refdiv 3 on a 12 MHz crystal: FREF = 4 MHz < 5 MHz floor. */
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_REFDIV_RANGE, pio_clk_configure_pll(&clk, 3, 125, 6, 2));
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_POSTDIV_RANGE, pio_clk_configure_pll(&clk, 1, 125, 0, 2));
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_POSTDIV_RANGE, pio_clk_configure_pll(&clk, 1, 125, 6, 8));
}

static void test_xosc_and_divider_validation(void)
{
    clk.xosc_hz = 100000; /* 100 kHz: out of range */
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_XOSC_RANGE, pio_clk_validate(&clk));
    pio_clk_init_default(&clk);
    clk.clksys_div_int = 0;
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_DIV_RANGE, pio_clk_validate(&clk));
    pio_clk_init_default(&clk);
#if PIO_SIM_PIO_VERSION >= 1
    clk.clksys_div_frac = 0x10000U; /* 17 bits: too wide for 16.16 */
#else
    clk.clksys_div_frac = 0x100U; /* 9 bits: too wide for 24.8 */
#endif
    TEST_ASSERT_EQUAL_INT(PIO_CLK_ERR_DIV_RANGE, pio_clk_validate(&clk));
}

static void test_fractional_divider_hz(void)
{
    /* Divide the PLL output by 2.5: frac = half of the frac range. */
    clk.clksys_div_int = 2;
#if PIO_SIM_PIO_VERSION >= 1
    clk.clksys_div_frac = 0x8000U;                                /* 2.5 in 16.16 */
    TEST_ASSERT_EQUAL_UINT64(60000000ULL, pio_clk_sys_hz(&clk));  /* 150 / 2.5 */
#else
    clk.clksys_div_frac = 0x80U;                                  /* 2.5 in 24.8 */
    TEST_ASSERT_EQUAL_UINT64(50000000ULL, pio_clk_sys_hz(&clk));  /* 125 / 2.5 */
#endif
}

static void test_bypass_pll_runs_from_xosc(void)
{
    clk.bypass_pll = true;
    TEST_ASSERT_EQUAL_UINT64(12000000ULL, pio_clk_sys_hz(&clk));
    clk.clksys_div_int = 4;
    TEST_ASSERT_EQUAL_UINT64(3000000ULL, pio_clk_sys_hz(&clk));
}

static void test_ticks_to_ns_exact_and_rounded(void)
{
#if PIO_SIM_PIO_VERSION >= 1
    /* 150 MHz: 6.66… ns per tick — rounding to nearest. */
    TEST_ASSERT_EQUAL_UINT64(7ULL, pio_clk_ticks_to_ns(&clk, 1));
    TEST_ASSERT_EQUAL_UINT64(20ULL, pio_clk_ticks_to_ns(&clk, 3)); /* exactly 20 ns */
    TEST_ASSERT_EQUAL_UINT64(1000000000ULL, pio_clk_ticks_to_ns(&clk, 150000000ULL));
#else
    /* 125 MHz: exactly 8 ns per tick. */
    TEST_ASSERT_EQUAL_UINT64(8ULL, pio_clk_ticks_to_ns(&clk, 1));
    TEST_ASSERT_EQUAL_UINT64(1000000000ULL, pio_clk_ticks_to_ns(&clk, 125000000ULL));
#endif
}

static void test_ns_to_ticks_ceils(void)
{
#if PIO_SIM_PIO_VERSION >= 1
    TEST_ASSERT_EQUAL_UINT64(1ULL, pio_clk_ns_to_ticks(&clk, 1));   /* < 1 tick → 1 */
    TEST_ASSERT_EQUAL_UINT64(2ULL, pio_clk_ns_to_ticks(&clk, 7));   /* 6.67 < 7 → 2 */
    TEST_ASSERT_EQUAL_UINT64(3ULL, pio_clk_ns_to_ticks(&clk, 20));  /* exact: 3     */
#else
    TEST_ASSERT_EQUAL_UINT64(1ULL, pio_clk_ns_to_ticks(&clk, 1));
    TEST_ASSERT_EQUAL_UINT64(1ULL, pio_clk_ns_to_ticks(&clk, 8));   /* exact: 1     */
    TEST_ASSERT_EQUAL_UINT64(2ULL, pio_clk_ns_to_ticks(&clk, 9));   /* ceil         */
#endif
}

static void test_us_roundtrip_large_counts(void)
{
    /* An hour of ticks converts without overflow and round-trips. */
    uint64_t hz = pio_clk_sys_hz(&clk);
    uint64_t hour_ticks = hz * 3600ULL;
    TEST_ASSERT_EQUAL_UINT64(3600000000ULL, pio_clk_ticks_to_us(&clk, hour_ticks));
    TEST_ASSERT_EQUAL_UINT64(hour_ticks, pio_clk_us_to_ticks(&clk, 3600000000ULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_default_tree_matches_boot_firmware);
    RUN_TEST(test_pll_rejects_vco_out_of_range);
    RUN_TEST(test_pll_rejects_fbdiv_refdiv_postdiv);
    RUN_TEST(test_xosc_and_divider_validation);
    RUN_TEST(test_fractional_divider_hz);
    RUN_TEST(test_bypass_pll_runs_from_xosc);
    RUN_TEST(test_ticks_to_ns_exact_and_rounded);
    RUN_TEST(test_ns_to_ticks_ceils);
    RUN_TEST(test_us_roundtrip_large_counts);
    return UNITY_END();
}
