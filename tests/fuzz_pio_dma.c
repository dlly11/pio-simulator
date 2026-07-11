/* fuzz_pio_dma.c — libFuzzer entry point for the DMA transfer engine.
 *
 * The DMA model moves bytes between raw host addresses, applies bswap, feeds the
 * sniffer (CRC/sum), advances addresses (linear or ring-wrapped), chains, and
 * paces off DREQs — all driven by fuzzer bytes here, under ASan/UBSan. It asserts
 * nothing; only a crash/UB is a bug.
 *
 * Soundness: unlike the rest of the library the DMA does NOT bounds-check the mem
 * addresses or trans_count — it writes where told, like silicon. So the harness
 * keeps every transfer inside its own buffers: each MEM endpoint points at the
 * base of a dedicated 64 KB buffer and trans_count is clamped so the touched span
 * (and any ring window, capped below) stays in bounds. An ASan report is then a real
 * library bug, not a harness overrun. pio_dma_tick moves at most one element, so
 * a hard tick cap bounds runtime even under a chain loop.
 *
 * Build with clang: -fsanitize=fuzzer,address,undefined (see PIO_SIM_BUILD_FUZZERS).
 */

#include "pio_dma.h"
#include "pio_sim.h"

#include <stdint.h>
#include <string.h>

#define FUZZ_DMA_BUF 65536U   /* per-endpoint buffer (ASan-tracked) */
#define FUZZ_DMA_MAX_TC 1024U /* trans_count cap: span <= 1024*4 = 4 KB << 64 KB */
#define FUZZ_DMA_TICKS 4096U  /* hard cap on transfers moved */

/* Non-contiguous enum: mask a fuzzed byte to a valid sniffer mode (a raw cast
 * would be a UBSan enum-load abort under -fno-sanitize-recover). */
static const pio_dma_sniff_calc_t SNIFF_MODES[6] = {PIO_DMA_SNIFF_CRC32, PIO_DMA_SNIFF_CRC32R,
                                                    PIO_DMA_SNIFF_CRC16, PIO_DMA_SNIFF_CRC16R,
                                                    PIO_DMA_SNIFF_EVEN,  PIO_DMA_SNIFF_SUM};

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

/* Dedicated in-bounds buffers for MEM endpoints (static → ASan instruments the
 * redzones, so a library OOB is still caught). Aligned to the max ring window we
 * allow (FUZZ_DMA_RING_MAX bits): DMA ring-wrap rounds the *absolute* address
 * down to a 2^ring_size boundary, so an unaligned buffer would let the window
 * start below it. With the base at a 2^N-aligned offset 0 and ring_size <= N, the
 * whole window stays inside the buffer. */
#define FUZZ_DMA_RING_MAX 12U /* window <= 4096 bytes */
_Alignas(1U << FUZZ_DMA_RING_MAX) static uint8_t g_rbuf[FUZZ_DMA_BUF];
_Alignas(1U << FUZZ_DMA_RING_MAX) static uint8_t g_wbuf[FUZZ_DMA_BUF];

/* Pick a DMA endpoint from a fuzzed selector: MEM (bounded buffer base), or a
 * PIO TX/RX FIFO on the one attached block. */
static pio_dma_addr_t pick_endpoint(uint8_t sel, uint8_t *membuf)
{
    switch (sel % 3U) {
    case 1:
        return pio_dma_addr_txf(0U, (uint8_t)((sel >> 2U) & 3U));
    case 2:
        return pio_dma_addr_rxf(0U, (uint8_t)((sel >> 2U) & 3U));
    case 0:
    default:
        return pio_dma_addr_mem(membuf); /* base of a 64 KB buffer */
    }
}

/* Translate a fuzzed DREQ selector into a valid TREQ (force / timer / PIO). */
static uint8_t pick_dreq(uint8_t sel)
{
    switch (sel & 3U) {
    case 1:
        return PIO_DMA_TREQ_TIMER((sel >> 2U) & 3U);
    case 2:
        return PIO_DMA_DREQ_PIO_TX(0U, (uint8_t)((sel >> 2U) & 3U));
    case 3:
        return PIO_DMA_DREQ_PIO_RX(0U, (uint8_t)((sel >> 2U) & 3U));
    case 0:
    default:
        return PIO_DMA_TREQ_FORCE; /* unpaced: always ready */
    }
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    fdp_t f = {data, size, 0U};

    /* One PIO block so TXF/RXF endpoints + PIO DREQs are reachable; SM0 gets a
     * default config so its FIFOs have capacity. */
    pio_sim_t sim;
    pio_sim_init(&sim);
    pio_sm_config smc = pio_get_default_sm_config();
    pio_sim_sm_init(&sim, 0U, 0U, &smc);
    pio_sim_t *pios[1] = {&sim};

    pio_dma_t dma;
    pio_dma_init(&dma, pios, 1U);

    /* Configure 1..2 channels from the input. */
    uint8_t nch = (uint8_t)(1U + (rd8(&f) & 1U));
    bool sniff_any = false;
    uint8_t sniff_ch = 0U;
    for (uint8_t ch = 0; ch < nch; ch++) {
        dma_channel_config c = pio_dma_channel_get_default_config(ch);
        uint8_t szb = rd8(&f);
        channel_config_set_transfer_data_size(&c, (pio_dma_size_t)(szb % 3U)); /* mask enum */
        uint8_t flags = rd8(&f);
        channel_config_set_read_increment(&c, (flags & 0x01U) != 0U);
        channel_config_set_write_increment(&c, (flags & 0x02U) != 0U);
        /* Clamp ring size to the window our buffers are aligned for (the lib
         * caps at 15, but our buffers only guarantee 2^FUZZ_DMA_RING_MAX). */
        channel_config_set_ring(&c, (flags & 0x04U) != 0U,
                                (uint8_t)(rd8(&f) % (FUZZ_DMA_RING_MAX + 1U)));
        channel_config_set_bswap(&c, (flags & 0x08U) != 0U);
        channel_config_set_sniff_enable(&c, (flags & 0x10U) != 0U);
        channel_config_set_chain_to(&c, rd8(&f)); /* clamped mod channel count */
        channel_config_set_dreq(&c, pick_dreq(rd8(&f)));
        if ((flags & 0x10U) != 0U) {
            sniff_any = true;
            sniff_ch = ch;
        }

        uint8_t kinds = rd8(&f);
        pio_dma_addr_t rd = pick_endpoint((uint8_t)(kinds & 0x0FU), g_rbuf);
        pio_dma_addr_t wr = pick_endpoint((uint8_t)((kinds >> 4U) & 0x0FU), g_wbuf);
        uint32_t tc = 1U + (rd16(&f) % FUZZ_DMA_MAX_TC); /* span <= 4 KB, in-bounds */
        pio_dma_channel_configure(&dma, ch, &c, wr, rd, tc, (flags & 0x20U) != 0U);
    }

    if (sniff_any) {
        pio_dma_sniffer_enable(&dma, sniff_ch, SNIFF_MODES[rd8(&f) % 6U], (rd8(&f) & 1U) != 0U);
        pio_dma_sniffer_set_data_accumulator(&dma, rd32(&f));
    }

    /* Bounded stimulus loop: tick the DMA, and occasionally feed/drain FIFOs,
     * repace timers, (re)start/abort channels, and reseed the sniffer. */
    uint32_t ticks = 0U;
    uint8_t nact = rd8(&f);
    for (uint8_t a = 0; a < nact; a++) {
        uint8_t ab = rd8(&f);
        switch (ab & 0x07U) {
        case 0:
        case 1:
        case 2:
        case 3:
            if (ticks < FUZZ_DMA_TICKS) {
                (void)pio_dma_tick(&dma);
                ticks++;
            }
            break;
        case 4:
            (void)pio_sim_sm_put(&sim, (uint8_t)((ab >> 4U) & 3U), rd32(&f));
            break;
        case 5: {
            uint32_t w = 0U;
            (void)pio_sim_sm_get(&sim, (uint8_t)((ab >> 4U) & 3U), &w);
            (void)w;
            break;
        }
        case 6:
            pio_dma_timer_set_fraction(&dma, (uint8_t)((ab >> 4U) & 3U), rd16(&f), rd16(&f));
            break;
        case 7:
        default:
            if ((ab & 0x80U) != 0U) {
                pio_dma_start_channel_mask(&dma, rd32(&f));
            } else {
                pio_dma_channel_abort(&dma, (uint8_t)((ab >> 4U) & 7U));
            }
            break;
        }
    }
    /* Drain any remaining scheduled work, still under the tick cap. */
    while ((ticks < FUZZ_DMA_TICKS) && pio_dma_tick(&dma)) {
        ticks++;
    }

    return 0;
}
