/*
 * SPDX-License-Identifier: MIT
 * pio_dma.c — RP2040/RP2350-style DMA controller model. See pio_dma.h for the
 * behavioural contract and the documented simplifications.
 *
 * A pure client of the simulator's public FIFO API: FIFO endpoints move data
 * through pio_sim_sm_put / pio_sim_sm_get, so the engine composes with any
 * mixture of standalone blocks and shared-pad groups.
 */

#include "pio_dma.h"

#include "pio_sim_internal.h"

#include <stddef.h>
#include <string.h>

/* PIO-side DREQ levels (kept in pio_sim.h for standalone use). */
bool pio_sim_dreq_tx(const pio_sim_t *pio, uint8_t sm)
{
    return !pio_sim_sm_is_tx_fifo_full(pio, sm);
}
bool pio_sim_dreq_rx(const pio_sim_t *pio, uint8_t sm)
{
    return !pio_sim_sm_is_rx_fifo_empty(pio, sm);
}

#define CH_IDX(ch) ((uint8_t)((ch) % PIO_SIM_DMA_NUM_CHANNELS))
#define LINE_IDX(l) ((uint8_t)((l) % PIO_SIM_DMA_NUM_IRQS))

/* ── Endpoints ─────────────────────────────────────────────────────────────── */

pio_dma_addr_t pio_dma_addr_mem(void *p)
{
    pio_dma_addr_t a = {PIO_DMA_ADDR_MEM, (unsigned char *)p, 0, 0};
    return a;
}

pio_dma_addr_t pio_dma_addr_txf(uint8_t pio_index, uint8_t sm)
{
    pio_dma_addr_t a = {PIO_DMA_ADDR_PIO_TXF, NULL, pio_index, sm};
    return a;
}

pio_dma_addr_t pio_dma_addr_rxf(uint8_t pio_index, uint8_t sm)
{
    pio_dma_addr_t a = {PIO_DMA_ADDR_PIO_RXF, NULL, pio_index, sm};
    return a;
}

/* ── Lifecycle / configuration ─────────────────────────────────────────────── */

void pio_dma_init(pio_dma_t *d, pio_sim_t *const *pios, uint8_t pio_count)
{
    (void)memset(d, 0, sizeof(*d));
    if (pio_count > PIO_SIM_NUM_PIO) {
        pio_count = PIO_SIM_NUM_PIO;
    }
    for (uint8_t i = 0; i < pio_count; i++) {
        d->pio[i] = pios[i];
    }
    d->pio_count = pio_count;
    for (uint8_t c = 0; c < PIO_SIM_DMA_NUM_CHANNELS; c++) {
        d->ch[c].ctrl = pio_dma_channel_get_default_config(c);
    }
}

dma_channel_config pio_dma_channel_get_default_config(uint8_t ch)
{
    dma_channel_config c;
    (void)memset(&c, 0, sizeof(c));
    c.data_size = DMA_SIZE_32;
    c.incr_read = true;
    c.incr_write = false;
    c.chain_to = CH_IDX(ch); /* self: no chain */
    c.treq_sel = PIO_DMA_TREQ_FORCE;
    return c;
}

/* ── SDK-named config mutators ─────────────────────────────────────────────── */

void channel_config_set_read_increment(dma_channel_config *c, bool incr) { c->incr_read = incr; }
void channel_config_set_write_increment(dma_channel_config *c, bool incr) { c->incr_write = incr; }
void channel_config_set_dreq(dma_channel_config *c, uint8_t dreq) { c->treq_sel = dreq; }
void channel_config_set_chain_to(dma_channel_config *c, uint8_t chain_to)
{
    /* Only real channels can be chained to; keep the field in range so the
     * `1u << chain_to` trigger in dma_advance is always well-defined. */
    c->chain_to = (uint8_t)(chain_to % PIO_SIM_DMA_NUM_CHANNELS);
}
void channel_config_set_transfer_data_size(dma_channel_config *c, pio_dma_size_t size)
{
    /* Only 8/16/32-bit elements exist; clamp so elem_bytes (1u << size) can never
     * exceed 4 and overrun the 4-byte transfer word. */
    if ((unsigned)size > (unsigned)DMA_SIZE_32) {
        size = DMA_SIZE_32;
    }
    c->data_size = size;
}
void channel_config_set_ring(dma_channel_config *c, bool write, uint8_t size_bits)
{
    c->ring_sel = write;
    c->ring_size = size_bits;
}
void channel_config_set_bswap(dma_channel_config *c, bool bswap) { c->bswap = bswap; }
void channel_config_set_irq_quiet(dma_channel_config *c, bool irq_quiet)
{
    c->irq_quiet = irq_quiet;
}
void channel_config_set_high_priority(dma_channel_config *c, bool high_priority)
{
    c->high_priority = high_priority;
}
void channel_config_set_sniff_enable(dma_channel_config *c, bool sniff_enable)
{
    c->sniff_en = sniff_enable;
}

void pio_dma_channel_configure(pio_dma_t *d, uint8_t ch, const dma_channel_config *c,
                               pio_dma_addr_t write_addr, pio_dma_addr_t read_addr,
                               uint32_t trans_count, bool trigger)
{
    pio_dma_channel_t *chan = &d->ch[CH_IDX(ch)];
    chan->ctrl = *c;
    chan->write_addr = write_addr;
    chan->read_addr = read_addr;
    chan->trans_count_reload = trans_count;
    chan->en = true;
    if (trigger) {
        pio_dma_start_channel_mask(d, (uint32_t)1U << CH_IDX(ch));
    }
}

void pio_dma_channel_set_enabled(pio_dma_t *d, uint8_t ch, bool en) { d->ch[CH_IDX(ch)].en = en; }

/* Raise the completion flag for `ch` and run its callback. */
static void dma_raise_irq(pio_dma_t *d, uint8_t ch)
{
    d->intr |= (uint32_t)1U << ch;
    if (d->ch[ch].on_complete != NULL) {
        d->ch[ch].on_complete(d, ch, d->ch[ch].cb_ctx);
    }
}

void pio_dma_start_channel_mask(pio_dma_t *d, uint32_t mask)
{
    for (uint8_t c = 0; c < PIO_SIM_DMA_NUM_CHANNELS; c++) {
        if ((mask & ((uint32_t)1U << c)) == 0U) {
            continue;
        }
        pio_dma_channel_t *chan = &d->ch[c];
        if (chan->trans_count_reload == 0U) {
            /* NULL trigger: the channel does not start. In IRQ_QUIET mode
             * this is the moment the deferred completion IRQ fires (the
             * control-block chain termination idiom). */
            if (chan->ctrl.irq_quiet) {
                dma_raise_irq(d, c);
            }
            continue;
        }
        chan->trans_count = chan->trans_count_reload;
        chan->busy = true;
    }
}

void pio_dma_channel_abort_mask(pio_dma_t *d, uint32_t mask)
{
    for (uint8_t c = 0; c < PIO_SIM_DMA_NUM_CHANNELS; c++) {
        if ((mask & ((uint32_t)1U << c)) != 0U) {
            d->ch[c].busy = false; /* atomic transfers: nothing in flight */
        }
    }
}

void pio_dma_channel_abort(pio_dma_t *d, uint8_t ch)
{
    pio_dma_channel_abort_mask(d, (uint32_t)1U << CH_IDX(ch));
}

bool pio_dma_channel_is_busy(const pio_dma_t *d, uint8_t ch) { return d->ch[CH_IDX(ch)].busy; }

uint32_t pio_dma_channel_transfer_count(const pio_dma_t *d, uint8_t ch)
{
    return d->ch[CH_IDX(ch)].trans_count;
}

void pio_dma_channel_set_callback(pio_dma_t *d, uint8_t ch, pio_dma_callback_t fn, void *ctx)
{
    d->ch[CH_IDX(ch)].on_complete = fn;
    d->ch[CH_IDX(ch)].cb_ctx = ctx;
}

/* ── Pacing timers ─────────────────────────────────────────────────────────── */

void pio_dma_timer_set_fraction(pio_dma_t *d, uint8_t timer, uint16_t numerator,
                                uint16_t denominator)
{
    timer = (uint8_t)(timer % PIO_SIM_DMA_NUM_TIMERS);
    d->timer[timer].num = numerator;
    d->timer[timer].den = denominator;
    d->timer[timer].accum = 0;
    d->timer[timer].credit = false;
}

static void dma_timers_tick(pio_dma_t *d)
{
    for (uint8_t t = 0; t < PIO_SIM_DMA_NUM_TIMERS; t++) {
        if (d->timer[t].den == 0U) {
            continue;
        }
        d->timer[t].accum += d->timer[t].num;
        if (d->timer[t].accum >= d->timer[t].den) {
            d->timer[t].accum -= d->timer[t].den;
            d->timer[t].credit = true; /* level request; not accumulated */
        }
    }
}

/* ── Sniffer ───────────────────────────────────────────────────────────────── */

void pio_dma_sniffer_enable(pio_dma_t *d, uint8_t ch, pio_dma_sniff_calc_t mode,
                            bool force_channel_enable)
{
    d->sniff.en = true;
    d->sniff.chan = CH_IDX(ch);
    d->sniff.calc = (uint8_t)mode;
    if (force_channel_enable) {
        d->ch[CH_IDX(ch)].ctrl.sniff_en = true;
    }
}

void pio_dma_sniffer_disable(pio_dma_t *d) { d->sniff.en = false; }

void pio_dma_sniffer_set_output_invert_enabled(pio_dma_t *d, bool invert)
{
    d->sniff.out_inv = invert;
}

void pio_dma_sniffer_set_output_reverse_enabled(pio_dma_t *d, bool reverse)
{
    d->sniff.out_rev = reverse;
}

void pio_dma_sniffer_set_data_accumulator(pio_dma_t *d, uint32_t seed_value)
{
    d->sniff.data = seed_value;
}

uint32_t pio_dma_sniffer_get_data_accumulator(const pio_dma_t *d)
{
    uint32_t v = d->sniff.data;
    if (d->sniff.out_rev) {
        v = pio_reverse32(v);
    }
    if (d->sniff.out_inv) {
        v = ~v;
    }
    return v;
}

static uint32_t crc_update_msb(uint32_t crc, uint8_t byte, uint32_t poly, uint8_t width)
{
    uint32_t top = (uint32_t)1U << (width - 1U);
    crc ^= (uint32_t)byte << (width - 8U);
    for (uint8_t b = 0; b < 8U; b++) {
        crc = ((crc & top) != 0U) ? ((crc << 1U) ^ poly) : (crc << 1U);
    }
    return (width == 32U) ? crc : (crc & (((uint32_t)1U << width) - 1U));
}

static uint32_t crc_update_lsb(uint32_t crc, uint8_t byte, uint32_t poly)
{
    crc ^= byte;
    for (uint8_t b = 0; b < 8U; b++) {
        crc = ((crc & 1U) != 0U) ? ((crc >> 1U) ^ poly) : (crc >> 1U);
    }
    return crc;
}

/* Feed one transferred element (its low `bytes` bytes) into SNIFF_DATA. */
static void dma_sniff(pio_dma_t *d, uint32_t value, uint8_t bytes)
{
    uint32_t crc = d->sniff.data;
    switch (d->sniff.calc) {
    case (uint8_t)PIO_DMA_SNIFF_CRC32:
        for (uint8_t i = 0; i < bytes; i++) {
            crc = crc_update_msb(crc, (uint8_t)(value >> (8U * i)), 0x04C11DB7U, 32);
        }
        break;
    case (uint8_t)PIO_DMA_SNIFF_CRC32R:
        for (uint8_t i = 0; i < bytes; i++) {
            crc = crc_update_lsb(crc, (uint8_t)(value >> (8U * i)), 0xEDB88320U);
        }
        break;
    case (uint8_t)PIO_DMA_SNIFF_CRC16:
        for (uint8_t i = 0; i < bytes; i++) {
            crc = crc_update_msb(crc, (uint8_t)(value >> (8U * i)), 0x1021U, 16);
        }
        break;
    case (uint8_t)PIO_DMA_SNIFF_CRC16R:
        for (uint8_t i = 0; i < bytes; i++) {
            crc = crc_update_lsb(crc & 0xFFFFU, (uint8_t)(value >> (8U * i)), 0x8408U);
        }
        break;
    case (uint8_t)PIO_DMA_SNIFF_EVEN:
        crc ^= value; /* per-bit-lane even parity accumulate */
        break;
    case (uint8_t)PIO_DMA_SNIFF_SUM:
    default:
        crc += value;
        break;
    }
    d->sniff.data = crc;
}

/* ── Transfer engine ───────────────────────────────────────────────────────── */

static uint8_t elem_bytes(pio_dma_size_t size) { return (uint8_t)(1U << (unsigned)size); }

/* Is the channel's TREQ asserted this tick? (Timer credits are consumed on
 * transfer, in dma_transfer_one.) */
static bool dma_treq_ready(const pio_dma_t *d, const pio_dma_channel_t *chan)
{
    uint8_t treq = chan->ctrl.treq_sel;
    if (treq == PIO_DMA_TREQ_FORCE) {
        return true;
    }
    if ((treq >= PIO_DMA_TREQ_TIMER(0)) && (treq <= PIO_DMA_TREQ_TIMER(3))) {
        return d->timer[treq - PIO_DMA_TREQ_TIMER(0)].credit;
    }
    uint8_t pio_index = treq / 8U;
    if ((pio_index >= d->pio_count) || (d->pio[pio_index] == NULL)) {
        return false;
    }
    uint8_t sm = treq % 4U;
    bool is_rx = ((treq % 8U) >= 4U);
    return is_rx ? pio_sim_dreq_rx(d->pio[pio_index], sm) : pio_sim_dreq_tx(d->pio[pio_index], sm);
}

/* FIFO endpoints must also be able to move data this tick, whatever the TREQ
 * says (a mis-paced channel must not overrun/underrun the model). */
static bool dma_endpoints_ready(const pio_dma_t *d, const pio_dma_channel_t *chan)
{
    const pio_dma_addr_t *r = &chan->read_addr;
    const pio_dma_addr_t *w = &chan->write_addr;
    if (r->kind == PIO_DMA_ADDR_PIO_RXF) {
        if ((r->pio_index >= d->pio_count) || (d->pio[r->pio_index] == NULL) ||
            !pio_sim_dreq_rx(d->pio[r->pio_index], r->sm)) {
            return false;
        }
    }
    if (w->kind == PIO_DMA_ADDR_PIO_TXF) {
        if ((w->pio_index >= d->pio_count) || (d->pio[w->pio_index] == NULL) ||
            !pio_sim_dreq_tx(d->pio[w->pio_index], w->sm)) {
            return false;
        }
    }
    /* Reading a TXF or writing an RXF is a configuration error: never ready. */
    if ((r->kind == PIO_DMA_ADDR_PIO_TXF) || (w->kind == PIO_DMA_ADDR_PIO_RXF)) {
        return false;
    }
    /* A NULL memory endpoint (e.g. pio_dma_addr_mem(NULL)) would fault the
     * transfer's memcpy — treat it as never ready instead. */
    if (((r->kind == PIO_DMA_ADDR_MEM) && (r->mem == NULL)) ||
        ((w->kind == PIO_DMA_ADDR_MEM) && (w->mem == NULL))) {
        return false;
    }
    return true;
}

/* Advance a MEM cursor by one element, honouring the ring if it applies to
 * this side. The wrap is address-aligned (the low ring_size bits wrap within
 * the naturally aligned window), exactly like CHx_CTRL RING — so the buffer
 * must be 2^ring_size aligned, as on hardware. */
static void dma_advance(pio_dma_addr_t *a, bool incr, uint8_t ring_size, bool ring_applies,
                        uint8_t bytes)
{
    if ((a->kind != PIO_DMA_ADDR_MEM) || !incr) {
        return;
    }
    /* Hardware RING_SIZE is a 4-bit field (0..15); guard the shift so a bogus
     * value can't invoke UB (shift >= the pointer width). Out-of-range means
     * "no wrap" rather than crashing. */
    if (ring_applies && (ring_size != 0U) && (ring_size < (uint8_t)(sizeof(uintptr_t) * 8U))) {
        /* Address-aligned wrap, expressed as a signed pointer adjustment so
         * there is no integer-to-pointer cast: move the cursor by the delta of
         * its low bits within the window (negative when it wraps to the base). */
        uintptr_t mask = ((uintptr_t)1U << ring_size) - 1U;
        uintptr_t low = (uintptr_t)a->mem & mask;
        uintptr_t next = (low + bytes) & mask;
        a->mem += (ptrdiff_t)next - (ptrdiff_t)low;
    } else {
        a->mem += bytes;
    }
}

static uint32_t dma_bswap(uint32_t v, uint8_t bytes)
{
    if (bytes == 2U) {
        return ((v & 0xFFU) << 8U) | ((v >> 8U) & 0xFFU);
    }
    if (bytes == 4U) {
        return (v << 24U) | ((v & 0xFF00U) << 8U) | ((v >> 8U) & 0xFF00U) | (v >> 24U);
    }
    return v;
}

/* Move one element on channel `c`; assumes readiness was checked. */
static void dma_transfer_one(pio_dma_t *d, uint8_t c)
{
    pio_dma_channel_t *chan = &d->ch[c];
    uint8_t bytes = elem_bytes(chan->ctrl.data_size);

    /* Read. */
    uint32_t v = 0;
    if (chan->read_addr.kind == PIO_DMA_ADDR_MEM) {
        (void)memcpy(&v, chan->read_addr.mem, bytes); /* low bytes, host-endian */
    } else {
        (void)pio_sim_sm_get(d->pio[chan->read_addr.pio_index], chan->read_addr.sm, &v);
    }

    if (chan->ctrl.bswap) {
        v = dma_bswap(v, bytes);
    }
    if (d->sniff.en && chan->ctrl.sniff_en && (d->sniff.chan == c)) {
        dma_sniff(d, v, bytes);
    }

    /* Write. */
    if (chan->write_addr.kind == PIO_DMA_ADDR_MEM) {
        (void)memcpy(chan->write_addr.mem, &v, bytes);
    } else {
        (void)pio_sim_sm_put(d->pio[chan->write_addr.pio_index], chan->write_addr.sm, v);
    }

    dma_advance(&chan->read_addr, chan->ctrl.incr_read, chan->ctrl.ring_size, !chan->ctrl.ring_sel,
                bytes);
    dma_advance(&chan->write_addr, chan->ctrl.incr_write, chan->ctrl.ring_size, chan->ctrl.ring_sel,
                bytes);

    /* Consume a timer credit if that is what paced us. */
    uint8_t treq = chan->ctrl.treq_sel;
    if ((treq >= PIO_DMA_TREQ_TIMER(0)) && (treq <= PIO_DMA_TREQ_TIMER(3))) {
        d->timer[treq - PIO_DMA_TREQ_TIMER(0)].credit = false;
    }

    chan->trans_count--;
    if (chan->trans_count == 0U) {
        chan->busy = false;
        if (!chan->ctrl.irq_quiet) {
            dma_raise_irq(d, c);
        }
        if ((chan->ctrl.chain_to != c) && (chan->ctrl.chain_to < PIO_SIM_DMA_NUM_CHANNELS)) {
            pio_dma_start_channel_mask(d, (uint32_t)1U << chan->ctrl.chain_to);
        }
    }
}

static bool dma_channel_ready(const pio_dma_t *d, uint8_t c)
{
    const pio_dma_channel_t *chan = &d->ch[c];
    return chan->en && chan->busy && dma_treq_ready(d, chan) && dma_endpoints_ready(d, chan);
}

bool pio_dma_tick(pio_dma_t *d)
{
    dma_timers_tick(d);
    /* One transfer per tick, whole controller: scan high-priority channels
     * round-robin first, then normal priority (see pio_dma.h). */
    for (uint8_t pass = 0; pass < 2U; pass++) {
        bool want_hp = (pass == 0U);
        for (uint8_t i = 0; i < PIO_SIM_DMA_NUM_CHANNELS; i++) {
            uint8_t c = (uint8_t)((d->rr_next + i) % PIO_SIM_DMA_NUM_CHANNELS);
            if ((d->ch[c].ctrl.high_priority == want_hp) && dma_channel_ready(d, c)) {
                dma_transfer_one(d, c);
                d->rr_next = (uint8_t)((c + 1U) % PIO_SIM_DMA_NUM_CHANNELS);
                return true;
            }
        }
    }
    return false;
}

/* ── Interrupts (SDK dma_irqn_*) ───────────────────────────────────────────── */

void pio_dma_irqn_set_channel_mask_enabled(pio_dma_t *d, uint8_t irq_index, uint32_t channel_mask,
                                           bool enabled)
{
    if (enabled) {
        d->inte[LINE_IDX(irq_index)] |= channel_mask;
    } else {
        d->inte[LINE_IDX(irq_index)] &= ~channel_mask;
    }
}

void pio_dma_irqn_set_channel_enabled(pio_dma_t *d, uint8_t irq_index, uint8_t ch, bool enabled)
{
    pio_dma_irqn_set_channel_mask_enabled(d, irq_index, (uint32_t)1U << CH_IDX(ch), enabled);
}

bool pio_dma_irqn_get_channel_status(const pio_dma_t *d, uint8_t irq_index, uint8_t ch)
{
    return (pio_dma_get_ints(d, irq_index) & ((uint32_t)1U << CH_IDX(ch))) != 0U;
}

void pio_dma_irqn_acknowledge_channel(pio_dma_t *d, uint8_t irq_index, uint8_t ch)
{
    (void)irq_index; /* INTR is shared across lines; clearing it acks both */
    d->intr &= ~((uint32_t)1U << CH_IDX(ch));
}

uint32_t pio_dma_get_intr(const pio_dma_t *d) { return d->intr; }

uint32_t pio_dma_get_ints(const pio_dma_t *d, uint8_t line)
{
    return (d->intr & d->inte[LINE_IDX(line)]) | d->intf[LINE_IDX(line)];
}

uint32_t pio_dma_get_inte(const pio_dma_t *d, uint8_t line) { return d->inte[LINE_IDX(line)]; }
uint32_t pio_dma_get_intf(const pio_dma_t *d, uint8_t line) { return d->intf[LINE_IDX(line)]; }

void pio_dma_irqn_force_channel(pio_dma_t *d, uint8_t irq_index, uint8_t ch, bool on)
{
    uint32_t bit = (uint32_t)1U << CH_IDX(ch);
    if (on) {
        d->intf[LINE_IDX(irq_index)] |= bit;
    } else {
        d->intf[LINE_IDX(irq_index)] &= ~bit;
    }
}
