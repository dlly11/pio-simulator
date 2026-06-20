# pio_sim

A pure-C functional simulator and assembler for the **RP2040 / RP2350 PIO**
block. It runs real PIO programs off-hardware so you can unit-test PIO logic
(protocols, timing, FIFO behaviour) in a normal host build — no silicon, no
Pico SDK, no RTOS.

- **`pio_sim`** — the engine. Implements the full PIO instruction set (JMP, WAIT,
  IN, OUT, PUSH, PULL, MOV, IRQ, SET) with side-set, delay, OSR/ISR shift
  registers (autopush/autopull), joinable 4-deep FIFOs, scratch X/Y, per-SM clock
  dividers, and a shared 32-pin pad model so multiple state machines — and an
  external device model — interact on the same wires. `pio_sim_tick()` advances
  one system clock.
- **`pio_asm`** — a minimal assembler for the subset of PIO assembly real programs
  use (`.program`, `.side_set`, `.wrap`, labels, all mnemonics), so tests can spell
  programs inline instead of shipping a `pioasm` toolchain.

Pure C (only `<stdint.h>`/`<stdbool.h>`/`<stdio.h>`/`<string.h>`), MIT-licensed.

## Build & test

```sh
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

## Embedding

`pio_sim` is a CMake library target with public headers. Vendor it (submodule,
`FetchContent`, or a subtree) and:

```cmake
add_subdirectory(third_party/pio_sim)   # defines the `pio_sim` target; tests off by default
target_link_libraries(your_tests PRIVATE pio_sim)
```

Consumers then `#include "pio_sim.h"` / `"pio_asm.h"`. Unit tests build only for a
standalone (top-level) configure; set `-DPIO_SIM_BUILD_TESTS=OFF` to force them off.

## API sketch

```c
#include "pio_sim.h"
#include "pio_asm.h"

pio_sim_t pio;
pio_sim_init(&pio);

pio_program_t prog;
pio_asm_assemble(".program blink\n set pins, 1 [1]\n set pins, 0 [1]\n", NULL, &prog);
pio_asm_load_program(&pio, /*sm=*/0, /*offset=*/0, &prog);
pio_sim_sm_set_enabled(&pio, 0, true);

for (int i = 0; i < 8; i++) pio_sim_tick(&pio);
bool pin0 = pio_sim_get_pin(&pio, 0);
```

See `pio_sim.h` and `pio_asm.h` for the full surface, and `tests/` for worked
examples (instruction-level and assembler round-trips).
