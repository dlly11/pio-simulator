# Examples

Six standalone reference programs, from a first blink to a full multi-block bus.
Each is a single `.c` file with a `main()` that sets something up, runs the
simulation, prints observable output, and returns non-zero if a built-in sanity
check fails — so they double as runnable smoke tests. They are built and run in
CI on both RP2040 and RP2350, so they always match the current API.

Read them in order: each introduces a little more of the library.

| # | File | Tier | Shows |
|---|------|------|-------|
| 1 | [`01_blink.c`](01_blink.c) | simple | assemble → configure → tick → read a pin |
| 2 | [`02_fifo_echo.c`](02_fifo_echo.c) | simple | TX/RX FIFO put/get, `pio_sim_run_until_rx` |
| 3 | [`03_uart_tx.c`](03_uart_tx.c) | intermediate | side-set, clock divider, pin sampling, a trace hook |
| 4 | [`04_ws2812.c`](04_ws2812.c) | intermediate | per-instruction delays, autopull, waveform decode |
| 5 | [`05_spi_dma.c`](05_spi_dma.c) | full | DMA → TX FIFO pacing (DREQ), PIO+DMA co-stepping |
| 6 | [`06_chip_bus.c`](06_chip_bus.c) | full | two PIO blocks on shared GPIO, DMA both ways, clock tree |

## What each one does

- **01 blink** — a two-instruction program toggles GPIO 0 into a square wave.
  The smallest complete use of the library and the canonical setup path.
- **02 fifo echo** — a program with no pins that echoes each TX word back to RX;
  the host drives it purely through the FIFO API.
- **03 uart tx** — the canonical pico-examples 8n1 UART transmitter. Sends a
  byte, samples the line at each bit centre, and decodes the frame back.
- **04 ws2812** — the canonical NeoPixel driver. Streams two 24-bit pixels and
  decodes the one-wire waveform (bit value is the HIGH-time) back to RGB.
- **05 spi dma** — a DMA channel streams a byte buffer into an SPI-style TX
  program's FIFO, paced by the PIO TX DREQ; the clocked-out bits are recovered.
- **06 chip bus** — the capstone. PIO0 clocks bytes onto shared GPIOs; PIO1,
  on the same pins, samples them back. One DMA channel feeds the producer, a
  second drains the consumer, and the whole chip is driven by `pio_chip_tick`
  with the exchange time reported in real nanoseconds from the clock tree.

## Building and running

### With CMake (also how CI builds them)

From the repository root:

```sh
cmake -B build -DPIO_SIM_BUILD_EXAMPLES=ON        # add -DPIO_SIM_PLATFORM=RP2040 for RP2040
cmake --build build
./build/examples/01_blink                          # ...02_fifo_echo, 03_uart_tx, etc.
```

### With bare gcc (no CMake)

Each example is one translation unit plus the library sources. Pick the target
chip with `-DPIO_SIM_PIO_VERSION` (`0` = RP2040, `1` = RP2350):

```sh
gcc -std=c11 -Wall -Wextra -Ilib/include -Ilib/config -DPIO_SIM_PIO_VERSION=1 \
    lib/src/*.c examples/01_blink.c -o /tmp/blink && /tmp/blink
```

## Example output

```
$ ./build/examples/06_chip_bus
PIO0 -> shared GPIO -> PIO1, 4 bytes, clk_sys = 150000000 Hz
exchange took 511 cycles (~3407 ns), producer ran 64 instructions
     sent  ->  received
     0xB3  ->  0xB3
     0x5A  ->  0x5A
     0xC7  ->  0xC7
     0x2D  ->  0x2D
OK: two PIO blocks exchanged 4 bytes over shared GPIO via DMA
```
