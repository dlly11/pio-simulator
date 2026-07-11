/* fuzz_pio_sim.c — libFuzzer entry point for the PIO simulator execution core.
 *
 * The engine executes arbitrary 16-bit instruction words fed via the public API
 * (pio_sim_load / pio_sim_sm_exec), bypassing the assembler. This harness turns
 * fuzzer bytes into an SM configuration, a program of raw words, and a bounded
 * stream of stimulus actions (tick / FIFO push-pop / pin drive / injected exec /
 * INTE-INTF / RX register-file), then runs a capped number of cycles — looking
 * for crashes, UB (UBSan) and memory errors (ASan). It asserts nothing about the
 * result; a stalled or garbage program is a valid outcome, only a crash is a bug.
 *
 * The library masks/clamps every index and count internally (sm & 3, pin %
 * NUM_PINS, thresholds/wrap clamped, pio_sim_load drops words past addr 31), so
 * raw bytes are safe to feed — EXCEPT the two enum-typed config setters, which
 * this harness masks to their declared range so a garbage value can't trip
 * UBSan's enum check. pio_sim_run advances exactly n ticks (no unbounded loop
 * even on a permanently-stalled SM), so the tick budget is a hard cap.
 *
 * TODO: the DMA engine (pio_dma) is not fuzzed here — it takes raw host pointers
 * + a transfer count, so a sound harness needs buffer-bounded address/count
 * clamping; add a separate fuzz_pio_dma.c for it.
 *
 * Build with clang: -fsanitize=fuzzer,address,undefined (see PIO_SIM_BUILD_FUZZERS).
 */

#include "pio_sim.h"

#include <stdint.h>
#include <string.h>

/* Number of pio_status_sel_t enumerators the current PIO version declares:
 * TX/RX/IRQ_SET always, plus IRQ_SET_PREV/NEXT on RP2350. Masking the fuzzed
 * status selector to this keeps it a valid enumerator. */
#if PIO_SIM_HAS_IRQ_STATUS && PIO_SIM_HAS_IRQ_PREVNEXT
#define FUZZ_STATUS_SEL_COUNT 5U
#else
#define FUZZ_STATUS_SEL_COUNT 3U
#endif

/* Bounds on per-input work (independent of any length byte in the input). */
#define FUZZ_ACTIONS_MAX 64U
#define FUZZ_TICKS_MAX 4096U

/* Fuzzed-data provider: a cursor over the input that returns 0 once exhausted,
 * so even a tiny seed drives a full config+program+action pass. Deterministic —
 * no rand, time, or uninitialised reads. */
typedef struct {
    const uint8_t *p;
    size_t n;
    size_t i;
} fdp_t;

static uint8_t rd8(fdp_t *f) { return (f->i < f->n) ? f->p[f->i++] : 0U; }

static uint16_t rd16(fdp_t *f)
{
    uint16_t lo = rd8(f);
    uint16_t hi = rd8(f);
    return (uint16_t)(lo | (uint16_t)(hi << 8U));
}

static uint32_t rd32(fdp_t *f)
{
    uint32_t lo = rd16(f);
    uint32_t hi = rd16(f);
    return lo | (hi << 16U);
}

/* Translate a run of input bytes into an SM config and init state machine `sm`. */
static void configure_sm(fdp_t *f, pio_sim_t *pio, uint8_t sm)
{
    pio_sm_config c = pio_get_default_sm_config();

    uint8_t out = rd8(f);
    sm_config_set_out_pins(&c, (uint8_t)(out >> 4U), (uint8_t)(out & 0x0FU));
    uint8_t set = rd8(f);
    sm_config_set_set_pins(&c, (uint8_t)(set >> 4U), (uint8_t)(set & 0x0FU));
    sm_config_set_in_pins(&c, rd8(f));

    uint8_t ss = rd8(f);
    sm_config_set_sideset(&c, (uint8_t)(ss & 0x07U), (ss & 0x08U) != 0U, (ss & 0x10U) != 0U);
    sm_config_set_sideset_pins(&c, rd8(f));
    sm_config_set_jmp_pin(&c, rd8(f));

    uint8_t sh = rd8(f);
    sm_config_set_out_shift(&c, (sh & 0x01U) != 0U, (sh & 0x02U) != 0U, rd8(f));
    sm_config_set_in_shift(&c, (sh & 0x04U) != 0U, (sh & 0x08U) != 0U, rd8(f));

    sm_config_set_wrap(&c, rd8(f), rd8(f));

    /* MUST mask: enum-typed parameters. All 6 fifo-join enumerators are declared
     * unconditionally; the status selector has 3 (v0) / 5 (v1). */
    sm_config_set_fifo_join(&c, (pio_fifo_join_t)(rd8(f) % 6U));
    uint8_t st = rd8(f);
    sm_config_set_mov_status(&c, (pio_status_sel_t)(st % FUZZ_STATUS_SEL_COUNT),
                             (uint8_t)(st >> 4U));

    uint8_t osp = rd8(f);
    sm_config_set_out_special(&c, (osp & 0x01U) != 0U, (osp & 0x02U) != 0U,
                              (uint8_t)((osp >> 2U) & 0x1FU));

    /* Clamp the divider to 1..4 for throughput (a huge divisor would make the
     * tick budget advance almost no SM cycles); this is speed, not safety. */
    sm_config_set_clkdiv_int_frac(&c, (uint16_t)(1U + (rd8(f) & 3U)), 0U);

    pio_sim_sm_init(pio, sm, (uint8_t)(rd8(f) % PIO_SIM_INSN_COUNT), &c);
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    fdp_t f = {data, size, 0U};

    pio_sim_t pio;
    pio_sim_init(&pio);

    /* Two SMs on one block: shares the pad model (cross-SM pin priority) and the
     * 8 IRQ flags (IRQ-rel fold, INTR/INTS aggregation) at near-zero extra cost. */
    configure_sm(&f, &pio, 0U);
    configure_sm(&f, &pio, 1U);

    /* Program: up to a full instruction memory of raw words. prog[] is fully
     * zero-initialised so pio_sim_load never reads an uninitialised element. */
    uint16_t prog[PIO_SIM_INSN_COUNT];
    memset(prog, 0, sizeof(prog));
    uint8_t nwords = (uint8_t)(rd8(&f) % (PIO_SIM_INSN_COUNT + 1U));
    for (uint8_t i = 0; i < nwords; i++) {
        prog[i] = rd16(&f);
    }
    pio_sim_load(&pio, (uint8_t)(rd8(&f) % PIO_SIM_INSN_COUNT), prog, nwords);

    pio_sim_sm_set_enabled(&pio, 0U, true);
    pio_sim_sm_set_enabled(&pio, 1U, true);

    uint32_t ticks_used = 0U;
    uint32_t nactions = rd8(&f);
    if (nactions > FUZZ_ACTIONS_MAX) {
        nactions = FUZZ_ACTIONS_MAX;
    }
    for (uint32_t a = 0; a < nactions; a++) {
        uint8_t ab = rd8(&f);
        uint8_t sm = (uint8_t)((ab >> 4U) & 1U);
        uint8_t line = (uint8_t)((ab >> 7U) & 1U);
        bool flag = (ab & 0x40U) != 0U;
        switch (ab & 0x0FU) {
        case 0:
        case 1:
        case 2: {
            uint32_t k = 1U + (uint32_t)((ab >> 5U) & 7U); /* 1..8 ticks */
            if (ticks_used < FUZZ_TICKS_MAX) {
                uint32_t room = FUZZ_TICKS_MAX - ticks_used;
                if (k > room) {
                    k = room;
                }
                pio_sim_run(&pio, k);
                ticks_used += k;
            }
            break;
        }
        case 3:
            (void)pio_sim_sm_put(&pio, sm, rd32(&f));
            break;
        case 4: {
            uint32_t word = 0U;
            (void)pio_sim_sm_get(&pio, sm, &word);
            (void)word;
            break;
        }
        case 5:
            pio_sim_set_pin(&pio, rd8(&f), (ab & 0x80U) != 0U);
            break;
        case 6:
            pio_sim_release_pin(&pio, rd8(&f));
            break;
        case 7:
        case 8:
            pio_sim_sm_exec(&pio, sm, rd16(&f)); /* arbitrary 16-bit opcode */
            break;
        case 9:
            pio_sim_sm_set_enabled(&pio, sm, (ab & 0x80U) != 0U);
            break;
        case 10:
            pio_sim_sm_set_pc(&pio, sm, rd8(&f));
            break;
        case 11:
#if PIO_SIM_HAS_GPIO_BASE
            pio_sim_set_gpio_base(&pio, rd8(&f));
#else
            (void)rd8(&f);
#endif
            break;
        case 12:
            pio_sim_set_irqn_source_mask_enabled(&pio, line, rd32(&f), flag);
            break;
        case 13:
            pio_sim_set_intf(&pio, line, rd32(&f), flag);
            break;
        case 14:
#if PIO_SIM_HAS_RXFIFO_MOV
            pio_sim_sm_rxfifo_put(&pio, sm, rd8(&f), rd32(&f));
#else
            (void)rd8(&f);
            (void)rd32(&f);
#endif
            break;
        case 15:
        default:
            if ((ab & 0x80U) != 0U) {
                pio_sim_sm_restart(&pio, sm);
            } else {
                pio_sim_sm_clear_fifos(&pio, sm);
            }
            break;
        }
    }

    return 0;
}
