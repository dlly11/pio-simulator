/* Smoke consumer: include the umbrella header and touch one symbol from each
 * module, so linking exercises the whole installed library surface. */
#include "pio.h"

#include <stdio.h>

int main(void)
{
    pio_sim_t pio;
    pio_sim_init(&pio);

    /* Assemble and run a one-instruction program. */
    pio_program_t prog;
    if (!pio_asm_assemble(".program t\n set x, 5\n", NULL, &prog)) {
        return 1;
    }
    (void)pio_asm_load_program(&pio, 0, 0, &prog);
    pio_sim_sm_set_enabled(&pio, 0, true);
    pio_sim_run(&pio, 1);

    /* Touch the pad/mux, clock, DMA and chip modules so each links. */
    pio_sim_pad_set_input_enable(&pio, 0, true);
    pio_clk_tree_t clk;
    pio_clk_init_default(&clk);
    pio_dma_t dma;
    pio_sim_t *blocks[] = {&pio};
    pio_dma_init(&dma, blocks, 1);
    pio_chip_t chip;
    pio_chip_init(&chip);

    printf("pio_sim consumer OK: x=%u, clk=%llu Hz\n", (unsigned)pio_sim_sm_get_x(&pio, 0),
           (unsigned long long)pio_clk_sys_hz(&clk));
    return 0;
}
