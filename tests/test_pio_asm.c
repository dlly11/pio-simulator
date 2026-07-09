/* test_pio_asm.c — PIO assembler tests.
 *
 * Verifies the assembler against the production SWD PIO programs: it checks
 * the emitted instruction words against independently hand-derived encodings,
 * resolves labels and side-set, and then runs the assembled programs on the
 * simulator to confirm they behave (clock + data on the pads). */

#include "unity.h"

#include "pio_asm.h"
#include "pio_sim.h"

#include <string.h>

/* Verbatim copies of the production SWD programs (src/pio/swd.pio). */
static const char *SWD_WRITE = ".program swd_write\n"
                               ".side_set 1\n"
                               ".wrap_target\n"
                               "    pull block          side 0\n"
                               "    mov  x, osr         side 0\n"
                               "    pull block          side 0\n"
                               "bitloop:\n"
                               "    out  pins, 1        side 0\n"
                               "    jmp  x-- bitloop    side 1\n"
                               ".wrap\n";

static const char *SWD_READ = ".program swd_read\n"
                              ".side_set 1\n"
                              ".wrap_target\n"
                              "    pull block          side 0\n"
                              "    mov  x, osr         side 0\n"
                              "    nop                 side 0\n"
                              "bitloop:\n"
                              "    in   pins, 1        side 1\n"
                              "    jmp  x-- bitloop    side 0\n"
                              "    push                side 0\n"
                              ".wrap\n";

static pio_sim_t pio;

void setUp(void) { pio_sim_init(&pio); }
void tearDown(void) { (void)0; }

/* ── Encoding ──────────────────────────────────────────────────────────────── */

static void test_assemble_swd_write_encoding(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(SWD_WRITE, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(5U, p.count);
    TEST_ASSERT_EQUAL_UINT8(1U, p.sideset_bits);
    TEST_ASSERT_FALSE(p.sideset_opt);
    TEST_ASSERT_EQUAL_UINT8(0U, p.wrap_bottom);
    TEST_ASSERT_EQUAL_UINT8(4U, p.wrap_top);

    /* Hand-derived from the PIO encoding (1 side-set bit, no opt). */
    TEST_ASSERT_EQUAL_HEX16(0x80A0U, p.insns[0]); /* pull block      side 0 */
    TEST_ASSERT_EQUAL_HEX16(0xA027U, p.insns[1]); /* mov x, osr      side 0 */
    TEST_ASSERT_EQUAL_HEX16(0x80A0U, p.insns[2]); /* pull block      side 0 */
    TEST_ASSERT_EQUAL_HEX16(0x6001U, p.insns[3]); /* out pins, 1     side 0 */
    TEST_ASSERT_EQUAL_HEX16(0x1043U, p.insns[4]); /* jmp x-- bitloop side 1 */
}

static void test_assemble_swd_read_encoding(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(SWD_READ, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(6U, p.count);

    TEST_ASSERT_EQUAL_HEX16(0x80A0U, p.insns[0]); /* pull block   side 0 */
    TEST_ASSERT_EQUAL_HEX16(0xA027U, p.insns[1]); /* mov x, osr   side 0 */
    TEST_ASSERT_EQUAL_HEX16(0xA042U, p.insns[2]); /* nop          side 0 */
    TEST_ASSERT_EQUAL_HEX16(0x5001U, p.insns[3]); /* in pins, 1   side 1 */
    TEST_ASSERT_EQUAL_HEX16(0x0043U, p.insns[4]); /* jmp x-- bitloop side 0 */
    TEST_ASSERT_EQUAL_HEX16(0x8020U, p.insns[5]); /* push         side 0 */
}

/* ── Label relocation across a load offset ─────────────────────────────────── */

static void test_jmp_relocated_by_offset(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE(pio_asm_assemble(SWD_WRITE, NULL, &p));
    /* Load at offset 10: the jmp's bitloop target (index 3) must become 13. */
    TEST_ASSERT_TRUE(pio_asm_load_program(&pio, 0, 10, &p));
    uint16_t jmp = pio.insn[10 + 4];
    TEST_ASSERT_EQUAL_UINT8(13U, (uint8_t)(jmp & 0x1FU));
}

/* ── Behavioural: run swd_write and watch the pads ─────────────────────────── */

#define PIN_SWCLK 2U
#define PIN_SWDIO 3U

static void config_swd_write(const pio_program_t *p)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, PIN_SWDIO, 1);
    sm_config_set_sideset_pins(&c, PIN_SWCLK);
    sm_config_set_out_shift(&c, true, false, 32); /* LSB first */
    pio_sim_sm_set_config(&pio, 0, &c);
    TEST_ASSERT_TRUE(pio_asm_load_program(&pio, 0, 0, p)); /* overlays wrap/side-set/pc */
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, PIN_SWCLK, 1, true);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, PIN_SWDIO, 1, true);
    pio_sim_sm_set_enabled(&pio, 0, true);
}

static void test_swd_write_clocks_out_lsb_first(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE(pio_asm_assemble(SWD_WRITE, NULL, &p));
    config_swd_write(&p);

    /* Shift out 4 bits 0b1010 (LSB first → 0,1,0,1). TX protocol: count-1, data. */
    pio_sim_sm_put(&pio, 0, 3U);   /* bit_count - 1 */
    pio_sim_sm_put(&pio, 0, 0x0A); /* data */

    /* Sample SWDIO on each rising SWCLK edge (the jmp side 1 cycle), as the
     * target would. Expected sequence LSB-first: 0,1,0,1. */
    bool prev_clk = false;
    uint8_t captured = 0;
    uint8_t nbits = 0;
    for (int i = 0; i < 200 && nbits < 4U; i++) {
        pio_sim_tick(&pio);
        bool clk = pio_sim_get_pin(&pio, PIN_SWCLK);
        if (clk && !prev_clk) { /* rising edge */
            bool dio = pio_sim_get_pin(&pio, PIN_SWDIO);
            captured |= (uint8_t)((dio ? 1U : 0U) << nbits);
            nbits++;
        }
        prev_clk = clk;
    }
    TEST_ASSERT_EQUAL_UINT8(4U, nbits);
    TEST_ASSERT_EQUAL_HEX8(0x0AU, captured);
}

/* ── Behavioural: swd_read samples SWDIO into the RX FIFO ───────────────────── */

static void test_swd_read_captures_bits(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE(pio_asm_assemble(SWD_READ, NULL, &p));
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, PIN_SWDIO);
    sm_config_set_sideset_pins(&c, PIN_SWCLK);
    sm_config_set_in_shift(&c, true, false, 32);
    pio_sim_sm_set_config(&pio, 0, &c);
    TEST_ASSERT_TRUE(pio_asm_load_program(&pio, 0, 0, &p));
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, PIN_SWCLK, 1, true);
    pio_sim_sm_set_consecutive_pindirs(&pio, 0, PIN_SWDIO, 1, false); /* input */
    pio_sim_sm_set_enabled(&pio, 0, true);

    /* Hold SWDIO high; capture 3 bits → ISR collects 0b111, pushed right-shifted
     * so bit 0 lands at position (32 - 3). */
    pio_sim_set_pin(&pio, PIN_SWDIO, true);
    pio_sim_sm_put(&pio, 0, 2U); /* 3 bits, count-1 */

    pio_sim_run_until_rx(&pio, 0, 200);
    TEST_ASSERT_FALSE(pio_sim_sm_is_rx_fifo_empty(&pio, 0));
    uint32_t w = 0;
    TEST_ASSERT_TRUE(pio_sim_sm_get(&pio, 0, &w));
    TEST_ASSERT_EQUAL_HEX32(0x7U << 29, w); /* 3 ones, right-shifted into MSBs */
}

/* ── Error reporting ───────────────────────────────────────────────────────── */

static void test_unknown_mnemonic_reports_line(void)
{
    pio_program_t p;
    const char *bad = ".program t\n"
                      "    set x, 1\n"
                      "    frobnicate y\n";
    TEST_ASSERT_FALSE(pio_asm_assemble(bad, NULL, &p));
    TEST_ASSERT_EQUAL_INT(3, p.error_line);
}

/* ── SM-relative IRQ syntax ─────────────────────────────────────────────────── */

static void test_assemble_irq_rel(void)
{
    pio_program_t p;
    const char *src = ".program rel\n"
                      "    irq set 0 rel\n"
                      "    wait 1 irq 2 rel\n";
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(2U, p.count);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_irq_rel(false, false, 0), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_wait_irq_rel(1, 2), p.insns[1]);
    /* The rel bit (0x10) must be present in the index field of both. */
    TEST_ASSERT_TRUE((p.insns[0] & PIO_IRQ_REL) != 0U);
    TEST_ASSERT_TRUE((p.insns[1] & PIO_IRQ_REL) != 0U);
}

#if PIO_SIM_HAS_RXFIFO_MOV
static void test_assemble_mov_rxfifo(void)
{
    pio_program_t p;
    const char *src = ".program rx\n"
                      "    mov rxfifo[1], isr\n"
                      "    mov osr, rxfifo[2]\n"
                      "    mov rxfifo[y], isr\n"
                      "    mov osr, rxfifo[y]\n";
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(4U, p.count);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov_to_rxfifo(1), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov_from_rxfifo(2), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov_to_rxfifo_y(), p.insns[2]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov_from_rxfifo_y(), p.insns[3]);
}
#endif /* PIO_SIM_HAS_RXFIFO_MOV */

#if PIO_SIM_HAS_WAIT_JMPPIN
static void test_assemble_wait_jmppin(void)
{
    /* pioasm syntax: `jmppin` (index 0) or `jmppin + <n>`. */
    pio_program_t p;
    const char *src = ".program wj\n"
                      "    wait 1 jmppin\n"
                      "    wait 0 jmppin + 3\n";
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(2U, p.count);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_wait_jmppin(1, 0), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_wait_jmppin(0, 3), p.insns[1]);
}
#endif /* PIO_SIM_HAS_WAIT_JMPPIN */

static void test_assemble_rel_rejected_on_wait_pin(void)
{
    pio_program_t p;
    const char *bad = ".program t\n"
                      "    wait 1 pin 0 rel\n";
    TEST_ASSERT_FALSE(pio_asm_assemble(bad, NULL, &p));
    TEST_ASSERT_EQUAL_INT(2, p.error_line);
}

static void test_select_program_by_name(void)
{
    /* A file with both programs; select swd_read explicitly. */
    char combined[1024];
    (void)snprintf(combined, sizeof(combined), "%s%s", SWD_WRITE, SWD_READ);
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(combined, "swd_read", &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(6U, p.count);         /* swd_read is 6 instructions */
    TEST_ASSERT_EQUAL_HEX16(0x8020U, p.insns[5]); /* trailing push */
}

/* ── Mnemonic operand coverage ─────────────────────────────────────────────── */

/* Every jmp condition (and the bare always form) encodes to the right opcode. */
static void test_jmp_conditions(void)
{
    const char *src = ".program j\n"
                      "t:\n"
                      "    jmp t\n"
                      "    jmp !x t\n"
                      "    jmp x-- t\n"
                      "    jmp !y t\n"
                      "    jmp y-- t\n"
                      "    jmp x!=y t\n"
                      "    jmp pin t\n"
                      "    jmp !osre t\n"
                      "    jmp 9\n"; /* numeric target */
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(9U, p.count);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_ALWAYS, 0), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_NOTX, 0), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_XDEC, 0), p.insns[2]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_NOTY, 0), p.insns[3]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_YDEC, 0), p.insns[4]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_XNEY, 0), p.insns[5]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_PIN, 0), p.insns[6]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_NOTOSRE, 0), p.insns[7]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_ALWAYS, 9), p.insns[8]);
}

/* wait gpio / pin / irq sources all encode. */
static void test_wait_sources(void)
{
    const char *src = ".program w\n"
                      "    wait 0 gpio 5\n"
                      "    wait 1 pin 3\n"
                      "    wait 1 irq 2\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_wait(0, PIO_WAIT_GPIO, 5), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_wait(1, PIO_WAIT_PIN, 3), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_wait(1, PIO_WAIT_IRQ, 2), p.insns[2]);
}

/* in/out destination fields beyond the SWD subset. */
static void test_in_out_variants(void)
{
    const char *src = ".program io\n"
                      "    in x, 5\n"
                      "    out pc, 32\n" /* count 32 wraps to 0 */
                      "    out pindirs, 8\n"
                      "    out exec, 16\n"
                      "    out isr, 1\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_in(PIO_SRC_X, 5), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_out(5, 0), p.insns[1]);  /* pc */
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_out(4, 8), p.insns[2]);  /* pindirs */
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_out(7, 16), p.insns[3]); /* exec */
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_out(6, 1), p.insns[4]);  /* isr */
}

/* push/pull qualifiers (iffull/ifempty/block/noblock). */
static void test_push_pull_qualifiers(void)
{
    const char *src = ".program pp\n"
                      "    push iffull noblock\n"
                      "    pull ifempty noblock\n"
                      "    push block\n"
                      "    pull block\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_push(true, false), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_pull(true, false), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_push(false, true), p.insns[2]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_pull(false, true), p.insns[3]);
}

/* mov invert/reverse source ops, extra dest fields, and set destinations. */
static void test_mov_ops_and_set(void)
{
    const char *src = ".program ms\n"
                      "    mov x, !y\n"
                      "    mov osr, ::isr\n"
                      "    mov pins, x\n"
                      "    set pindirs, 3\n"
                      "    set x, 31\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov(1, PIO_MOV_INVERT, PIO_SRC_Y), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov(7, PIO_MOV_REVERSE, PIO_SRC_ISR), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov(0, PIO_MOV_NONE, PIO_SRC_X), p.insns[2]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(4, 3), p.insns[3]); /* pindirs */
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(1, 31), p.insns[4]);
}

/* ── Delay / side-set encoding ─────────────────────────────────────────────── */

/* A required (non-opt) side-set packs the delay and side bits into [12:8]. */
static void test_delay_and_required_sideset(void)
{
    const char *src = ".program d\n"
                      ".side_set 1\n"
                      "    nop side 0 [2]\n"
                      "    nop side 1 [1]\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(2U, p.count);
    uint16_t nop = pio_sim_encode_nop();
    /* 1 side-set bit → 4 delay bits. [2] side 0 → field 0x02; [1] side 1 → 0x11. */
    TEST_ASSERT_EQUAL_HEX16((uint16_t)(nop | (0x02U << 8)), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16((uint16_t)(nop | (0x11U << 8)), p.insns[1]);
}

/* A delay bracket may hold a spaced expression ("[T3 - 1]"), which tokenizes
 * into several tokens — pioasm accepts it, so we must too (the ws2812 corpus
 * program uses exactly this). All spacings encode identically. */
static void test_spaced_delay_expression(void)
{
    const char *src = ".program d\n"
                      ".define T 4\n"
                      "    nop [T - 1]\n"   /* spaced: tokens "[T","-","1]" */
                      "    nop [ T - 1 ]\n" /* brackets split off too        */
                      "    nop [T-1]\n";    /* glued (already worked)        */
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    uint16_t nop_d3 = (uint16_t)(pio_sim_encode_nop() | (3U << 8)); /* delay 3 */
    TEST_ASSERT_EQUAL_HEX16(nop_d3, p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(nop_d3, p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(nop_d3, p.insns[2]);
}

/* A shift by a negative or out-of-range count is a malformed expression: it
 * must be rejected, not evaluated with undefined behaviour (found by fuzzing).
 * The sanitizer CI build turns any residual UB here into a failure. */
static void test_shift_expression_out_of_range_rejected(void)
{
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(".program p\n.define A 1 << -30\n set x, A\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program p\n set x, (1 << 70)\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program p\n set x, (4 >> -1)\n", NULL, &p));
    /* An in-range shift still works. */
    TEST_ASSERT_TRUE(pio_asm_assemble(".program p\n.define A 1 << 4\n set x, A\n", NULL, &p));
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 16), p.insns[0]);
}

/* Arithmetic in a constant expression must not invoke signed-overflow UB: a
 * product of two valid 32-bit operands overflows int64 and wraps in the
 * unsigned domain (found by fuzzing — the sanitizer CI build turns any residual
 * UB here into a failure). The result is masked to the instruction field, so
 * wrapping is observationally fine. */
static void test_expression_overflow_is_defined(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE(
        pio_asm_assemble(".program p\n.define A 2777777777 * 3983219825\n set x, A\n", NULL, &p));
    /* 0xFFFFFFFF * 0xFFFFFFFF ~= 1.8e19 > INT64_MAX — wraps, no UB. */
    TEST_ASSERT_TRUE(pio_asm_assemble(".program p\n set x, (4294967295 * 4294967295)\n", NULL, &p));
}

/* A numeric literal that overflows 32 bits is rejected, not silently wrapped
 * into a bogus operand (hardening; sanitizer build guards the arithmetic). */
static void test_numeric_literal_overflow_rejected(void)
{
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(".program p\n set x, 99999999999999999999\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program p\n set x, 0x1FFFFFFFF\n", NULL, &p));
    TEST_ASSERT_TRUE(pio_asm_assemble(".program p\n.define A 0xFFFFFFFF\n set x, 1\n", NULL, &p));
}

/* Deeply nested parentheses must fail cleanly via the depth guard rather than
 * recursing until the stack is exhausted (found by fuzzing). */
static void test_expression_deep_nesting_bounded(void)
{
    char src[256];
    size_t k = 0;
    const char *pre = ".program p\n set x, ";
    for (const char *q = pre; *q; q++) {
        src[k++] = *q;
    }
    for (int i = 0; i < 100; i++) {
        src[k++] = '(';
    }
    src[k++] = '1';
    src[k++] = '\n';
    src[k] = '\0';
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(src, NULL, &p)); /* no crash, graceful reject */
}

/* Loading a program at an offset where it would not fit in the 32-word
 * instruction memory is rejected rather than corrupting the wrap window / PC. */
static void test_load_past_instruction_memory_rejected(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE(pio_asm_assemble(".program p\n set x, 1\n set y, 2\n set x, 3\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_load_program(&pio, 0, 30, &p)); /* 30 + 3 > 32 */
    TEST_ASSERT_TRUE(pio_asm_load_program(&pio, 0, 29, &p));  /* 29 + 3 == 32, fits */
}

/* An optional side-set sets the enable bit only on instructions that name one. */
static void test_optional_sideset(void)
{
    const char *src = ".program o\n"
                      ".side_set 2 opt pindirs\n"
                      "    nop side 1 [2]\n"
                      "    nop [1]\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_TRUE(p.sideset_opt);
    TEST_ASSERT_TRUE(p.sideset_pindirs);
    TEST_ASSERT_EQUAL_UINT8(3U, p.sideset_bits); /* 2 data + 1 opt */
    /* Enable bit is 0x10 in the [12:8] field → 0x1000 in the word. */
    TEST_ASSERT_TRUE((p.insns[0] & 0x1000U) != 0U);
    TEST_ASSERT_TRUE((p.insns[1] & 0x1000U) == 0U);
}

/* ── Public labels, verbatim blocks, program selection ─────────────────────── */

static void test_public_labels_and_verbatim(void)
{
    const char *src = ".program p\n"
                      "% c-sdk {\n"
                      "  int ignored = 1\n"
                      "%}\n"
                      "public entry:\n"
                      "    nop\n"
                      "    jmp entry\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(2U, p.count);
    TEST_ASSERT_EQUAL_UINT8(1U, p.public_count);
    TEST_ASSERT_EQUAL_STRING("entry", p.public_labels[0].name);
    TEST_ASSERT_EQUAL_UINT8(0U, p.public_labels[0].index);
}

/* Hex literals, ';' comments, and upper-case mnemonics all parse. */
static void test_hex_comments_and_case(void)
{
    const char *src = ".program h\n"
                      "    set x, 0x1F   ; a trailing comment\n"
                      "    JMP 0x2\n"; /* upper-case mnemonic */
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(1, 31), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_ALWAYS, 2), p.insns[1]);
}

/* .define symbols substitute for integer operands (value, count, side, delay). */
static void test_assemble_define(void)
{
    const char *src = ".define COUNT 5\n"
                      ".define DLY 0x02\n"
                      ".program d\n"
                      ".side_set 1\n"
                      "    set x, COUNT     side 0\n"
                      "    out pins, COUNT  side 1 [DLY]\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(2U, p.count);
    /* set x, 5 (side 0); out pins, 5 (side 1, delay 2). */
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(1, 5), p.insns[0]);
    uint16_t out5 = pio_sim_encode_out(0, 5);
    TEST_ASSERT_EQUAL_HEX16((uint16_t)(out5 | (0x12U << 8)), p.insns[1]);
}

/* Slash-slash and slash-star block comments, plus binary literals. */
static void test_assemble_comments_and_binary(void)
{
    const char *src = ".program cb\n"
                      "    set x, 0b00101   // line comment\n"
                      "    set y, 0b11000   ; semicolon comment\n"
                      "    /* a block\n"
                      "       comment spanning lines */\n"
                      "    set x, 7\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(3U, p.count);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 5), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_Y, 24), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 7), p.insns[2]);
}

/* Arithmetic in operands and `.define`s, plus label arithmetic. */
static void test_assemble_expressions(void)
{
    const char *src = ".define BASE 4\n"
                      ".define N (BASE * 2 + 1)\n" /* 9 */
                      ".program ex\n"
                      "top:\n"
                      "    set x, N\n"            /* 9                    */
                      "    set y, (1 << 4) - 1\n" /* 15                   */
                      "    set x, (0x0F & 6)\n"   /* 6                    */
                      "    set y, (BASE | 3)\n"   /* 7                    */
                      "    jmp top + 1\n";        /* target 1 (bare expr) */
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(5U, p.count);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 9), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_Y, 15), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 6), p.insns[2]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_Y, 7), p.insns[3]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_ALWAYS, 1), p.insns[4]);
}

/* Program-config directives are captured into pio_program_t and applied to an SM
 * by pio_asm_apply_program_config. .word emits a raw instruction word. */
static void test_assemble_directives_and_apply(void)
{
    const char *src = ".program cfg\n"
                      ".origin 4\n"
                      ".clock_div 2.5\n"
                      ".fifo tx\n"
                      ".mov_status rxfifo < 2\n"
                      ".out 6 left auto 16\n"
                      ".in 32 right 8\n"
                      ".set 5\n"
                      "    nop\n"
                      ".word 0x1234\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_STRING("cfg", p.name);
    TEST_ASSERT_TRUE(p.has_origin);
    TEST_ASSERT_EQUAL_UINT8(4U, p.origin);
    TEST_ASSERT_TRUE(p.has_clock_div);
    TEST_ASSERT_TRUE(p.has_fifo_join);
    TEST_ASSERT_EQUAL_INT(PIO_FIFO_JOIN_TX, p.fifo_join);
    TEST_ASSERT_TRUE(p.has_mov_status);
    TEST_ASSERT_EQUAL_INT(PIO_STATUS_RX_LEVEL, p.mov_status_sel);
    TEST_ASSERT_EQUAL_UINT8(2U, p.mov_status_n);
    TEST_ASSERT_EQUAL_UINT8(2U, p.count); /* nop + .word */
    TEST_ASSERT_EQUAL_HEX16(0x1234, p.insns[1]);

    pio_sim_t pio;
    pio_sim_init(&pio);
    pio_asm_apply_program_config(&pio, 0, &p);
    TEST_ASSERT_EQUAL_INT(PIO_SHIFT_LEFT, pio.sm[0].out_dir);
    TEST_ASSERT_TRUE(pio.sm[0].autopull);
    TEST_ASSERT_EQUAL_UINT8(16U, pio.sm[0].pull_thresh);
    TEST_ASSERT_EQUAL_INT(PIO_SHIFT_RIGHT, pio.sm[0].in_dir);
    TEST_ASSERT_FALSE(pio.sm[0].autopush);
    TEST_ASSERT_EQUAL_UINT8(8U, pio.sm[0].push_thresh);
    TEST_ASSERT_EQUAL_UINT16(2U, pio.sm[0].clkdiv_int);
    TEST_ASSERT_EQUAL_UINT8(128U, pio.sm[0].clkdiv_frac);
    /* .out / .set carry pin counts (sm_config_set_{out,set}_pin_count). */
    TEST_ASSERT_EQUAL_UINT8(6U, pio.sm[0].out_count);
    TEST_ASSERT_EQUAL_UINT8(5U, pio.sm[0].set_count);
#if PIO_SIM_HAS_IN_PIN_COUNT
    TEST_ASSERT_EQUAL_UINT8(32U, pio.sm[0].in_count); /* .in 32 → unmasked */
#endif
}

#if PIO_SIM_HAS_IRQ_STATUS && PIO_SIM_HAS_IRQ_PREVNEXT
/* `.mov_status irq next|prev set N` selects the neighbouring block's flag. */
static void test_mov_status_irq_next_prev_directive(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(
        pio_asm_assemble(".program s\n.mov_status irq next set 3\n    nop\n", NULL, &p), p.error);
    TEST_ASSERT_TRUE(p.has_mov_status);
    TEST_ASSERT_EQUAL_INT(PIO_STATUS_IRQ_SET_NEXT, p.mov_status_sel);
    TEST_ASSERT_EQUAL_UINT8(3U, p.mov_status_n);

    TEST_ASSERT_TRUE_MESSAGE(
        pio_asm_assemble(".program s\n.mov_status irq prev set 5\n    nop\n", NULL, &p), p.error);
    TEST_ASSERT_EQUAL_INT(PIO_STATUS_IRQ_SET_PREV, p.mov_status_sel);
    TEST_ASSERT_EQUAL_UINT8(5U, p.mov_status_n);

    /* Plain `irq set N` stays local. */
    TEST_ASSERT_TRUE_MESSAGE(
        pio_asm_assemble(".program s\n.mov_status irq set 1\n    nop\n", NULL, &p), p.error);
    TEST_ASSERT_EQUAL_INT(PIO_STATUS_IRQ_SET, p.mov_status_sel);

    /* Applied config lands on the SM. */
    pio_sim_t pio;
    pio_sim_init(&pio);
    TEST_ASSERT_TRUE(
        pio_asm_assemble(".program s\n.mov_status irq next set 3\n    nop\n", NULL, &p));
    pio_asm_apply_program_config(&pio, 0, &p);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)PIO_STATUS_IRQ_SET_NEXT, pio.sm[0].status_sel);
    TEST_ASSERT_EQUAL_UINT8(3U, pio.sm[0].status_n);
}
#endif

/* A line with more tokens than the assembler holds is rejected, not silently
 * truncated (which would drop a trailing side-set / delay and mis-encode). */
static void test_too_many_tokens_errors(void)
{
    /* Far more than MAX_TOKENS comma/space-separated tokens. */
    const char *src = ".program t\n"
                      "    jmp 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 side 1\n";
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(src, NULL, &p));
}

/* An undefined symbol in an operand is still an error (not silently zero). */
static void test_undefined_symbol_errors(void)
{
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    set x, NOPE\n", NULL, &p));
}

/* pio_asm_load_program_at_origin loads at the program's .origin (relocating jmp
 * targets and the PC), or at 0 when no .origin was given. */
static void test_load_program_at_origin(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE(pio_asm_assemble(".program o\n.origin 5\n  set x, 1\n  jmp 0\n", NULL, &p));
    TEST_ASSERT_TRUE(p.has_origin);
    TEST_ASSERT_TRUE(pio_asm_load_program_at_origin(&pio, 0, &p));
    TEST_ASSERT_EQUAL_UINT8(5U, pio_sim_sm_get_pc(&pio, 0)); /* PC at the origin */
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 1), pio.insn[5]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_jmp(PIO_COND_ALWAYS, 5), pio.insn[6]); /* jmp 0 -> 5 */
}

/* `::` (reverse) binds loosest of all, as in pioasm's grammar: `::2 >> 27` is
 * ::(2 >> 27) = 0, not (::2) >> 27 = 8. */
static void test_reverse_binds_loosest(void)
{
    const char *src = ".program r\n"
                      ".define E (::2 >> 27) & 0x1F\n"
                      ".define F (::E + 4) >> 27\n"
                      "    set x, E\n"
                      "    set y, F\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 0), p.insns[0]);
    /* F = ::(0 + 4) >> 27 = 0x20000000 >> 27 = 4 (old precedence gave 0). */
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_Y, 4), p.insns[1]);
}

/* `in status` must be rejected: IN source encoding 101 is reserved on hardware
 * (STATUS is a MOV-only source), and real pioasm rejects it. */
static void test_in_status_rejected(void)
{
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    in status, 8\n", NULL, &p));
    /* …while mov from status stays accepted. */
    TEST_ASSERT_TRUE(pio_asm_assemble(".program t\n    mov x, status\n", NULL, &p));
}

/* pio_asm_load_program starts the SM at the program's first instruction (the
 * SDK's pio_sm_init convention), so a preamble before .wrap_target runs once. */
static void test_load_program_pc_starts_at_offset(void)
{
    const char *src = ".program pre\n"
                      "    set x, 5\n" /* preamble: must execute */
                      ".wrap_target\n"
                      "    set y, 1\n"
                      ".wrap\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_UINT8(1U, p.wrap_bottom);
    TEST_ASSERT_TRUE(pio_asm_load_program(&pio, 0, 2, &p));
    TEST_ASSERT_EQUAL_UINT8(2U, pio_sim_sm_get_pc(&pio, 0)); /* offset, not wrap */
    pio_sim_sm_set_enabled(&pio, 0, true);
    pio_sim_run(&pio, 1);
    TEST_ASSERT_EQUAL_UINT32(5U, pio.sm[0].x); /* preamble ran */
}

#if PIO_SIM_HAS_RXFIFO_MOV
/* `.fifo putget` selects the combined put+get mode (both FJOIN_RX bits), not
 * plain PUT; txput/txget map to their single-direction modes. */
static void test_fifo_putget_maps_to_putget(void)
{
    pio_program_t p;
    TEST_ASSERT_TRUE(pio_asm_assemble(".program f\n.fifo putget\n    nop\n", NULL, &p));
    TEST_ASSERT_EQUAL_INT(PIO_FIFO_JOIN_RX_PUTGET, p.fifo_join);
    TEST_ASSERT_TRUE(pio_asm_assemble(".program f\n.fifo txput\n    nop\n", NULL, &p));
    TEST_ASSERT_EQUAL_INT(PIO_FIFO_JOIN_RX_PUT, p.fifo_join);
    TEST_ASSERT_TRUE(pio_asm_assemble(".program f\n.fifo txget\n    nop\n", NULL, &p));
    TEST_ASSERT_EQUAL_INT(PIO_FIFO_JOIN_RX_GET, p.fifo_join);
}
#endif

/* Non-public defines are local to their program (pioasm scoping): one from a
 * non-selected program must not leak into the selected program's symbols. */
static void test_define_not_leaked_from_other_program(void)
{
    const char *src = ".program a\n.define LOCAL 7\n    set x, LOCAL\n"
                      ".program b\n    set x, LOCAL\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, "a", &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 7), p.insns[0]);
    TEST_ASSERT_FALSE(pio_asm_assemble(src, "b", &p)); /* LOCAL is a's alone */
    /* PUBLIC defines (and top-level ones) stay global. */
    const char *src2 = ".define TOP 3\n"
                       ".program a\n.define public SHARED 5\n    set x, SHARED\n"
                       ".program b\n    set x, (SHARED + TOP)\n";
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src2, "b", &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(PIO_DST_X, 8), p.insns[0]);
}

/* irq in absolute (non-rel) set/wait/clear forms. */
static void test_irq_absolute(void)
{
    const char *src = ".program i\n"
                      "    irq set 3\n"
                      "    irq wait 4\n"
                      "    irq clear 5\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_irq(false, false, 3), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_irq(false, true, 4), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_irq(true, false, 5), p.insns[2]);
}

/* The remaining out/mov/set destination and mov source fields. */
static void test_more_dest_src_fields(void)
{
    const char *src = ".program d2\n"
                      "    out x, 1\n"
                      "    out y, 1\n"
                      "    out null, 1\n"
                      "    mov y, x\n"
                      "    mov pc, x\n"
                      "    mov isr, x\n"
                      "    mov x, null\n"
                      "    mov x, status\n"
                      "    set y, 1\n";
    pio_program_t p;
    TEST_ASSERT_TRUE_MESSAGE(pio_asm_assemble(src, NULL, &p), p.error);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_out(1, 1), p.insns[0]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_out(2, 1), p.insns[1]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_out(3, 1), p.insns[2]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov(2, PIO_MOV_NONE, PIO_SRC_X), p.insns[3]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov(5, PIO_MOV_NONE, PIO_SRC_X), p.insns[4]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov(6, PIO_MOV_NONE, PIO_SRC_X), p.insns[5]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov(1, PIO_MOV_NONE, PIO_SRC_NULL), p.insns[6]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_mov(1, PIO_MOV_NONE, PIO_SRC_STATUS), p.insns[7]);
    TEST_ASSERT_EQUAL_HEX16(pio_sim_encode_set(2, 1), p.insns[8]);
}

/* Each mnemonic rejects a too-short operand list. */
static void test_arity_errors(void)
{
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    jmp\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    wait 1\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    in x\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    out x\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    mov x\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    set x\n", NULL, &p));
    TEST_ASSERT_FALSE(
        pio_asm_assemble(".program t\n    wait 9 gpio z\n", NULL, &p)); /* bad index */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    wait z gpio 0\n", NULL, &p)); /* bad pol */
}

/* ── Error paths ───────────────────────────────────────────────────────────── */

/* Overflowing the 32-instruction program is reported, not silently truncated. */
static void test_program_too_long_reports_error(void)
{
    char src[512];
    int k = snprintf(src, sizeof(src), ".program big\n");
    for (int i = 0; i < 33; i++) {
        k += snprintf(&src[k], sizeof(src) - (size_t)k, "nop\n");
    }
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(src, NULL, &p));
    TEST_ASSERT_TRUE(strstr(p.error, "exceeds") != NULL);
}

/* Selecting a program name that isn't present yields no instructions. */
static void test_no_instructions_error(void)
{
    pio_program_t p;
    const char *src = ".program a\n    nop\n";
    TEST_ASSERT_FALSE(pio_asm_assemble(src, "nonexistent", &p));
    TEST_ASSERT_TRUE(strstr(p.error, "no instructions") != NULL);
}

/* A representative malformed operand for each mnemonic is rejected. */
static void test_operand_errors(void)
{
    pio_program_t p;
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    jmp !z foo\n", NULL, &p));
    TEST_ASSERT_EQUAL_INT(2, p.error_line); /* unknown jmp condition */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    out badd, 1\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    in bad, 1\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    mov x, bogus\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    set badx, 1\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    push bogus\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    pull bogus\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    wait 1 bogus 0\n", NULL, &p));
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    irq\n", NULL, &p)); /* needs index */
}

/* Side-set and delay constraint violations are all reported. */
static void test_sideset_and_delay_errors(void)
{
    pio_program_t p;
    /* side used but no .side_set declared */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    nop side 1\n", NULL, &p));
    /* required side-set missing on an instruction */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n.side_set 1\n    nop\n", NULL, &p));
    /* side value out of range for the declared width */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n.side_set 1\n    nop side 5\n", NULL, &p));
    /* delay too large for the remaining bits (5 side-set bits → 0 delay bits) */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n.side_set 5\n    nop side 0 [1]\n", NULL, &p));
    /* malformed .side_set directive */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n.side_set\n    nop\n", NULL, &p));
    /* non-numeric delay */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    nop [xx]\n", NULL, &p));
    /* non-numeric side value (rejected during side/delay extraction) */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n    nop side z\n", NULL, &p));
    /* optional side value out of range for the data width */
    TEST_ASSERT_FALSE(pio_asm_assemble(".program t\n.side_set 2 opt\n    nop side 9\n", NULL, &p));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_assemble_swd_write_encoding);
    RUN_TEST(test_assemble_swd_read_encoding);
    RUN_TEST(test_jmp_relocated_by_offset);
    RUN_TEST(test_swd_write_clocks_out_lsb_first);
    RUN_TEST(test_swd_read_captures_bits);
    RUN_TEST(test_unknown_mnemonic_reports_line);
    RUN_TEST(test_assemble_irq_rel);
#if PIO_SIM_HAS_RXFIFO_MOV
    RUN_TEST(test_assemble_mov_rxfifo);
    RUN_TEST(test_fifo_putget_maps_to_putget);
#endif
#if PIO_SIM_HAS_WAIT_JMPPIN
    RUN_TEST(test_assemble_wait_jmppin);
#endif
    RUN_TEST(test_assemble_rel_rejected_on_wait_pin);
    RUN_TEST(test_select_program_by_name);
    RUN_TEST(test_jmp_conditions);
    RUN_TEST(test_wait_sources);
    RUN_TEST(test_in_out_variants);
    RUN_TEST(test_push_pull_qualifiers);
    RUN_TEST(test_mov_ops_and_set);
    RUN_TEST(test_delay_and_required_sideset);
    RUN_TEST(test_spaced_delay_expression);
    RUN_TEST(test_shift_expression_out_of_range_rejected);
    RUN_TEST(test_expression_overflow_is_defined);
    RUN_TEST(test_numeric_literal_overflow_rejected);
    RUN_TEST(test_expression_deep_nesting_bounded);
    RUN_TEST(test_load_past_instruction_memory_rejected);
    RUN_TEST(test_optional_sideset);
    RUN_TEST(test_public_labels_and_verbatim);
    RUN_TEST(test_hex_comments_and_case);
    RUN_TEST(test_assemble_define);
    RUN_TEST(test_assemble_comments_and_binary);
    RUN_TEST(test_assemble_expressions);
    RUN_TEST(test_assemble_directives_and_apply);
#if PIO_SIM_HAS_IRQ_STATUS && PIO_SIM_HAS_IRQ_PREVNEXT
    RUN_TEST(test_mov_status_irq_next_prev_directive);
#endif
    RUN_TEST(test_too_many_tokens_errors);
    RUN_TEST(test_undefined_symbol_errors);
    RUN_TEST(test_load_program_at_origin);
    RUN_TEST(test_reverse_binds_loosest);
    RUN_TEST(test_define_not_leaked_from_other_program);
    RUN_TEST(test_in_status_rejected);
    RUN_TEST(test_load_program_pc_starts_at_offset);
    RUN_TEST(test_irq_absolute);
    RUN_TEST(test_more_dest_src_fields);
    RUN_TEST(test_arity_errors);
    RUN_TEST(test_program_too_long_reports_error);
    RUN_TEST(test_no_instructions_error);
    RUN_TEST(test_operand_errors);
    RUN_TEST(test_sideset_and_delay_errors);
    return UNITY_END();
}
