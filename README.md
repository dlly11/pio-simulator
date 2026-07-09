# pio_sim

![ci](https://github.com/dlly11/pio-simulator/actions/workflows/ci.yml/badge.svg)

A pure-C functional simulator and assembler for the **RP2040 / RP2350 PIO**
block. It runs real PIO programs off-hardware so you can unit-test PIO logic
(protocols, timing, FIFO behaviour) in a normal host build — no silicon, no
Pico SDK, no RTOS.

- **`pio_sim`** — the engine. Implements the full PIO instruction set (JMP, WAIT,
  IN, OUT, PUSH, PULL, MOV, IRQ, SET) with side-set, delay, OSR/ISR shift
  registers (autopush/autopull), joinable 4-deep FIFOs, scratch X/Y, per-SM clock
  dividers, and a shared pad model so multiple state machines — and an external
  device model — interact on the same wires. `pio_sim_tick()` advances one system
  clock. Non-blocking FIFO ops match the hardware: `push noblock` on a full RX
  FIFO discards the word and clears the ISR; `pull noblock` on an empty TX FIFO
  loads the OSR from scratch X. On RP2350 it also models the 48-GPIO window
  (GPIOBASE), inter-PIO IRQ signalling, and multi-PIO groups. Companion modules
  cover the chip around the block: `pio_gpio` (PADS_BANK0 pad registers and the
  IO_BANK0 FUNCSEL mux with output/input overrides), `pio_dma` (the full 12/16-
  channel DMA controller with chaining, IRQs, pacing timers and the CRC
  sniffer), `pio_clock` (XOSC→PLL→clk_sys tree with tick↔time conversion), and
  `pio_chip` (all blocks + shared pads + DMA + clock behind one tick call) — see
  Scope below.
- **`pio_asm`** — a pioasm-compatible assembler: all mnemonics and operand forms
  (incl. RP2350's `mov rxfifo[]`, `wait jmppin`, `irq/wait prev|next`, `mov
  pindirs`), every directive (`.program`, `.define`, `.side_set`, `.wrap_target`,
  `.wrap`, `.origin`, `.pio_version`, `.clock_div`, `.fifo`, `.mov_status`,
  `.in`/`.out`/`.set`, `.word`, `.lang_opt`), labels, `;`/`//`/`/* */` comments, hex/binary
  literals, and full integer expressions (with pioasm's precedence) over defines
  and labels. Tests spell programs inline instead of shipping a `pioasm`
  toolchain; a build-time differential checks the encodings stay bit-exact.

Pure C (only `<stdint.h>`/`<stdbool.h>`/`<stdio.h>`/`<stdlib.h>`/`<string.h>`),
MIT-licensed.

## Layout

```
lib/include/   public headers (pio_sim.h, pio_asm.h)
lib/src/       implementation (and any private headers)
lib/config/    library configuration header (pio_sim_config.h: target-chip select)
third_party/   Unity test framework (git submodule)
tests/         unit tests + unity_config.h
```

## Target chip (RP2040 vs RP2350)

The RP2350 PIO ("version 1") is a superset of the RP2040 PIO ("version 0"): it
adds the indexed RX-FIFO `mov rxfifo[]` instructions, `MOV STATUS` from an IRQ
flag, `wait ... jmppin`, inter-PIO (`irq/wait ... prev|next`) signalling, and the
48-GPIO window (GPIOBASE). `PIO_SIM_PIO_VERSION` selects which set the library
exposes — `1` (RP2350, the default) or `0` (RP2040). The v1-only API is
**removed** from a v0 build, so using it on an RP2040 target is a compile error
rather than a silent surprise.

Set it either way:

```sh
# CMake (maps to PIO_SIM_PIO_VERSION and to `pioasm -v <0|1>` for the test):
cmake -B build -G Ninja -DPIO_SIM_PLATFORM=RP2040   # or RP2350 (default)
```

```c
/* …or as a project/compiler macro, which a CMake -D also sets: */
-DPIO_SIM_PIO_VERSION=0
```

Embedders can instead vendor their own `pio_sim_config.h` earlier on the include
path (`lib/config` is on the public include path) to pin the default. The macro
is exported as a **PUBLIC** compile definition, so a consumer always compiles
against the same feature surface the library was built with.

## System modelling (pads, interrupts, DMA, multi-PIO)

- **GPIOBASE** — `pio_sim_set_gpio_base(&pio, 16)` shifts the state machines'
  32-pin window to GPIO 16-47 (default 0 → GPIO 0-31), as on RP2350B. Every
  SM-side pin access (IN/OUT/SET/side-set/MOV pins, `WAIT GPIO/PIN`, `JMP PIN`)
  is offset; host access (`pio_sim_get_pin`/`set_pin`) stays absolute.
- **Pin drive & arbitration** — each state machine has its own pin output and
  pin-direction (output-enable) registers; the pad's level is *resolved* from the
  pin **writes** of each cycle, as hardware collates them (RP2040 §3.5.6.1). When
  several SMs write one pin on the **same cycle** the **highest-numbered** wins;
  a pin nobody writes holds its latched level, so a later, uncontested write from
  any SM lands regardless of number (`OUT_STICKY` re-asserts an SM's pins every
  cycle, which is what gives it continuous priority). A state machine drives a
  pad only where its **pindirs** mark the pin an output (as on hardware —
  `SET/OUT PINS` to a pin left as input has no pad effect). A pin with no PIO
  output, no external driver, and no pull is **high-impedance** and reads 0
  (true tri-state, not its last driven value).
- **Output controls** — `pio_sim_sm_set_out_special` mirrors the SDK's
  `sm_config_set_out_special`: `OUT_STICKY` plus inline OUT-enable use a chosen
  OUT-data bit to gate the pin write — and under sticky, an enable of 0 *releases*
  the pins (clears the SM's output enable) so a lower-priority SM, an external
  level, or a pull shows through (the documented override pattern).
- **Pads, pulls & external drive** — `pio_sim_set_pull_level` gives a pin a
  pull-up/down (open-drain buses like I²C); `pio_sim_set_pin` externally drives a
  pin and `pio_sim_release_pin` stops (so it floats/pulls); `pio_sim_set_input_sync_bypass`
  skips the two-cycle input synchroniser per pin. Priority is PIO > external > pull
  > float.
- **Multiple PIO blocks** — a device has more than one PIO (`PIO_SIM_NUM_PIO`:
  2 on RP2040, 3 on RP2350). `pio_sim_group_init` ties several `pio_sim_t`
  blocks to one clock; `pio_sim_group_tick`/`_run` advance them in lockstep, and
  on RP2350 their `irq ... prev|next` links are auto-wired into a ring
  (PIO0→PIO1→PIO2→PIO0). `pio_sim_group_init_shared` additionally points the
  blocks at one shared pad set so they drive and sample the same GPIOs. (Link two
  blocks directly with `pio_sim_set_irq_neighbors` for the IRQ ring alone.)
- **FIFO status** — `pio_sim_tx_level`/`rx_level` report occupancy (FSTAT) and
  `pio_sim_get_fdebug`/`clear_fdebug` expose the sticky TXSTALL/TXOVER/RXUNDER/
  RXSTALL debug flags. In the RP2350 random-access RX modes (`.fifo txput`/`txget`)
  the four entries are reached by index with `pio_sim_rxfifo_get`/`_put`, not the
  FIFO pop/push.
- **System interrupts** — the two PIO interrupt lines are modelled:
  `pio_sim_set_irq_enable`/`set_irq_force` (INTE/INTF) over the INTR sources
  (per-SM RX-not-empty, TX-not-full, and the low four SM IRQ flags), read back
  with `pio_sim_get_ints` / `pio_sim_interrupt_line`.
- **Synchronised start** — `pio_sim_set_sm_mask_enabled` enables several SMs at
  once and phase-aligns their dividers (like `pio_enable_sm_mask_in_sync`).
- **DMA pacing** — `pio_sim_dma_t` models a DMA channel servicing a FIFO, paced
  by the PIO's DREQ (`pio_sim_dreq_tx`/`_rx`). `pio_sim_dma_init` is the simple
  word-at-a-time form; `pio_sim_dma_init_ex` adds 8/16/32-bit transfer sizes,
  fixed-address mode, an element ring, and chaining to a next channel.
  `pio_sim_dma_step`/`step_many` move one element when the DREQ is asserted, and
  `pio_sim_dma_run` ticks the PIO and services the channel (and its chain) until
  done.

The in-tree assembler also tracks the program-config directives in
`pio_program_t`. `pio_asm_apply_program_config` applies the ones that map to SM
state: `.clock_div`, `.fifo`, `.mov_status`, and the `.in`/`.out`/`.set` shift
config **and pin counts** (`.out`/`.set` set the OUT/SET pin counts; `.in` sets
the RP2350 IN pin count, which has no RP2040 equivalent and is ignored there).
`.origin` and `.pio_version` are recorded as metadata only — `.origin` is advisory
since the load address is passed explicitly to `pio_asm_load_program`.

## Scope

This is a **functional** simulator: one `pio_sim_tick()` is one PIO system clock,
and the per-SM clock divider is rate-exact (the 16.8 fractional accumulator
averages to `int + frac/256` cycles per SM step). The chip-level surroundings of
the PIO block are modelled too:

- **Clock tree** (`pio_clock.h`): XOSC → PLL_SYS → clk_sys divider with
  datasheet-validated parameters and exact integer tick↔ns/µs conversion. It
  does not change stepping — a tick is still one clk_sys cycle; the tree gives
  it a duration.
- **GPIO pads** (`pio_gpio.h`): PADS_BANK0 with IE, OD, pulls (both enabled =
  bus keeper) and RP2350 pad isolation simulated exactly; DRIVE strength,
  SLEWFAST and SCHMITT are accepted/stored but are analog and do not affect the
  digital simulation.
- **GPIO function mux** (`pio_gpio.h`): per-pin FUNCSEL routing with
  OUTOVER/OEOVER/INOVER; inputs always visible to the PIO, as on silicon.
- **DMA** (`pio_dma.h`): the full 12/16-channel controller — address
  generation, rings, chaining/null triggers, DREQ + pacing timers, byte swap,
  the CRC sniffer, IRQ lines and abort.
- **Chip umbrella** (`pio_chip.h`): all PIO blocks + shared pads + DMA + clock
  in one `pio_chip_tick()`.

Documented simulator-only deviations (each noted at its API):

- Pins reset to a sim-only `LEGACY_ANY_PIO` function (every block may drive
  every pin) and pads reset to friendly defaults (no pulls). Hardware-accurate
  routing/reset are opt-in via `pio_sim_gpio_set_function` and
  `pio_sim_pads_reset_hw`.
- The DMA moves one atomic element per tick, whole-controller, with
  level-sensitive DREQ — deterministic, and equivalent for PIO-FIFO workloads,
  but not the pipelined bus of real silicon.
- Autopull is a background OSR refill at one-tick granularity, following the
  datasheet rules (refills during stalls/delays; an OUT cannot fill and shift
  the OSR in one cycle). The datasheet notes the exact refill point is
  pipeline-dependent and not to be relied upon.

Still intentionally out of scope: analog pad behaviour (drive-strength
contention, slew, hysteresis), real-time execution, bus-cycle-exact DMA
pipelining, and the RP2350 DMA MPU/security attributes.

## Build & test

The unit tests use [Unity](https://github.com/ThrowTheSwitch/Unity), pulled in as a
git submodule. Clone with `--recurse-submodules`, or initialise it after cloning:

```sh
git submodule update --init
cmake -B build        # -G Ninja optional; any generator works
cmake --build build
ctest --test-dir build --output-on-failure
```

No CMake handy? The library is dependency-free, so a bare compiler run works too:

```sh
gcc -std=c11 -Wall -Wextra -I lib/include -I lib/config -I third_party/unity/src \
    lib/src/pio_sim.c lib/src/pio_dma.c lib/src/pio_asm.c \
    tests/test_pio_sim.c third_party/unity/src/unity.c -o test_sim && ./test_sim
```

CI builds gcc + clang across both platforms (`-DPIO_SIM_PLATFORM=RP2040|RP2350`),
runs the suites under ASan/UBSan, lints with clang-tidy/clang-format, and
cross-checks the assembler's encodings against the real pioasm (see below).

### Lint & format

CI also runs `clang-format` (style in `.clang-format`) and `clang-tidy` (checks in
`.clang-tidy`) over the library. To reproduce locally:

```sh
# formatting (no changes => clean)
clang-format --dry-run --Werror lib/src/*.c lib/include/*.h tests/test_*.c

# static analysis: needs a compile database for the include paths / feature defines
cmake -B build -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
clang-tidy -p build lib/src/pio_sim.c lib/src/pio_dma.c lib/src/pio_asm.c
```

## Embedding

`pio_sim` is a CMake library target with public headers. Vendor it (submodule,
`FetchContent`, or a subtree) and:

```cmake
add_subdirectory(third_party/pio_sim)   # defines the `pio_sim` target; tests off by default
target_link_libraries(your_tests PRIVATE pio_sim)
```

…or install it and consume the package:

```cmake
find_package(pio_sim CONFIG REQUIRED)
target_link_libraries(your_tests PRIVATE pio_sim::pio_sim)
```

Consumers then `#include "pio_sim.h"` / `"pio_asm.h"` (or `#include "pio.h"` for
the whole API in one include). Unit tests build only for a standalone (top-level)
configure; set `-DPIO_SIM_BUILD_TESTS=OFF` to force them off. (Embedders that *do*
build the tests must also init the `third_party/unity` submodule.)

See [CONTRIBUTING.md](CONTRIBUTING.md) for the dev environment (a Nix flake
pinning the clang-21 toolchain) and how to reproduce the CI gate locally.

## API sketch

```c
#include "pio_sim.h"
#include "pio_asm.h"
#include <stddef.h> /* NULL */

pio_sim_t pio;
pio_sim_init(&pio);

pio_program_t prog;
pio_asm_assemble(".program blink\n set pins, 1 [1]\n set pins, 0 [1]\n", NULL, &prog);
pio_asm_load_program(&pio, /*sm=*/0, /*offset=*/0, &prog);
/* Loading only writes the instruction words. Directives that map to SM config
 * (.clock_div, .fifo, .mov_status, .out/.in/.set, side-set) take effect only
 * via pio_asm_apply_program_config: */
pio_asm_apply_program_config(&pio, /*sm=*/0, &prog);
pio_sim_sm_set_enabled(&pio, 0, true);

for (int i = 0; i < 8; i++) pio_sim_tick(&pio);
bool pin0 = pio_sim_get_pin(&pio, 0);
```

See `pio_sim.h` and `pio_asm.h` for the full surface, and `tests/` for worked
examples (instruction-level and assembler round-trips).
