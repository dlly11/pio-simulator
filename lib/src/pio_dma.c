/*
 * SPDX-License-Identifier: MIT
 * pio_dma — the DREQ-paced DMA channel model for pio_sim. See pio_sim.h.
 *
 * A pure client of the simulator's public FIFO API: it moves elements between
 * a host buffer and a state machine's FIFOs when the corresponding DREQ is
 * asserted, one element per step, as a real DMA channel does.
 */

#include "pio_sim.h"

#include <string.h>

bool pio_sim_dreq_tx(const pio_sim_t *pio, uint8_t sm) { return !pio_sim_tx_full(pio, sm); }
bool pio_sim_dreq_rx(const pio_sim_t *pio, uint8_t sm) { return !pio_sim_rx_empty(pio, sm); }

void pio_sim_dma_init(pio_sim_dma_t *dma, pio_sim_t *pio, uint8_t sm, pio_dma_dir_t dir,
                      uint32_t *buf, uint32_t count)
{
    pio_sim_dma_init_ex(dma, pio, sm, dir, buf, count, PIO_DMA_SIZE_32, true, 0, NULL);
}

void pio_sim_dma_init_ex(pio_sim_dma_t *dma, pio_sim_t *pio, uint8_t sm, pio_dma_dir_t dir,
                         void *buf, uint32_t count, pio_dma_size_t size, bool incr, uint32_t ring,
                         pio_sim_dma_t *chain)
{
    dma->pio = pio;
    dma->sm = sm;
    dma->dir = dir;
    dma->buf = buf;
    dma->count = count;
    dma->pos = 0;
    dma->size = size;
    dma->incr = incr;
    dma->ring = ring;
    dma->chain = chain;
}

bool pio_sim_dma_done(const pio_sim_dma_t *dma) { return dma->pos >= dma->count; }

static size_t dma_elem_bytes(pio_dma_size_t size) { return (size_t)1U << (unsigned)size; }

/* Element index this transfer addresses, honouring increment and ring wrap. */
static uint32_t dma_index(const pio_sim_dma_t *dma)
{
    uint32_t idx = dma->incr ? dma->pos : 0U;
    if (dma->ring != 0U) {
        idx %= dma->ring;
    }
    return idx;
}

static uint32_t dma_read_elem(const pio_sim_dma_t *dma, uint32_t idx)
{
    const unsigned char *b = (const unsigned char *)dma->buf;
    size_t es = dma_elem_bytes(dma->size);
    uint32_t v = 0;
    (void)memcpy(&v, &b[(size_t)idx * es], es); /* low `es` bytes (host-endian) */
    return v;
}

static void dma_write_elem(pio_sim_dma_t *dma, uint32_t idx, uint32_t v)
{
    unsigned char *b = (unsigned char *)dma->buf;
    size_t es = dma_elem_bytes(dma->size);
    (void)memcpy(&b[(size_t)idx * es], &v, es);
}

bool pio_sim_dma_step(pio_sim_dma_t *dma)
{
    if (pio_sim_dma_done(dma)) {
        return false;
    }
    if (dma->dir == PIO_DMA_TO_SM) {
        if (!pio_sim_dreq_tx(dma->pio, dma->sm)) {
            return false; /* TX DREQ not asserted: FIFO full */
        }
        (void)pio_sim_tx_push(dma->pio, dma->sm, dma_read_elem(dma, dma_index(dma)));
    } else {
        uint32_t word;
        if (!pio_sim_dreq_rx(dma->pio, dma->sm)) {
            return false; /* RX DREQ not asserted: FIFO empty */
        }
        (void)pio_sim_rx_pop(dma->pio, dma->sm, &word);
        dma_write_elem(dma, dma_index(dma), word);
    }
    dma->pos++;
    return true;
}

void pio_sim_dma_step_many(pio_sim_dma_t *const *chans, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        (void)pio_sim_dma_step(chans[i]);
    }
}

uint64_t pio_sim_dma_run(pio_sim_dma_t *dma, uint64_t max_ticks)
{
    uint64_t t = 0;
    pio_sim_dma_t *cur = dma;
    while ((cur != NULL) && (t < max_ticks)) {
        (void)pio_sim_dma_step(cur); /* service the FIFO, then advance the SM */
        pio_sim_tick(cur->pio);
        t++;
        if (pio_sim_dma_done(cur)) {
            cur = cur->chain; /* follow the chain, if any */
        }
    }
    return t;
}
