# Contributing to pio-simulator

Thanks for helping out. This is a pure-C library with a strict CI gate; the
fastest path to a green PR is to reproduce that gate locally, which the pinned
toolchain below makes exact.

## Dev environment (recommended: Nix)

CI pins **clang 21** for building, formatting, static analysis, sanitizers,
fuzzing and coverage. The repo ships a Nix flake that provides the identical
toolchain, so local results match CI (no clang-format/clang-tidy version skew):

```sh
nix develop           # clang, clang-format, clang-tidy, llvm-cov — all v21, on PATH
```

With [direnv](https://direnv.net/): `direnv allow` once, and the shell loads
automatically (`.envrc` is `use flake`).

Prefer your own toolchain? Anything with a C11 compiler works, but use
**clang-format 21 / clang-tidy 21** for the lint jobs or you may see version
differences. `nix run nixpkgs#pioasm` gives the real assembler for the encoding
differential.

## Build & test

CMake + Ninja, selecting the target PIO version with `-DPIO_SIM_PLATFORM`:

```sh
cmake -B build -G Ninja -DPIO_SIM_PLATFORM=RP2350   # or RP2040
cmake --build build
ctest --test-dir build --output-on-failure
```

Both platforms must pass — RP2350 is the superset, RP2040 exercises the
v0-gated paths. The encoding differential (`test_pio_asm_vs_pioasm`) runs only
when a `pioasm` is available:

```sh
cmake -B build -G Ninja -DPIOASM_EXECUTABLE="$(nix build --no-link --print-out-paths nixpkgs#pioasm)/bin/pioasm"
```

### Bare gcc (no CMake)

Handy for a quick check without a build tree:

```sh
gcc -std=c11 -Wall -Wextra -Wpedantic -Ilib/include -Ilib/config -Ilib/src \
    -Ithird_party/unity/src -Itests -DUNITY_INCLUDE_CONFIG_H -DPIO_SIM_PIO_VERSION=1 \
    lib/src/*.c third_party/unity/src/unity.c tests/test_pio_sim.c -o /tmp/t && /tmp/t
```

Set `-DPIO_SIM_PIO_VERSION=0` for RP2040, `=1` for RP2350. (Note: the CMake-only
`PIO_SIM_PLATFORM` name does nothing as a bare `-D` macro — the config header
reads `PIO_SIM_PIO_VERSION`.)

## Before you push — reproduce the CI gate

```sh
# Formatting (must be clean) — the same file set CI checks
clang-format --dry-run --Werror lib/src/*.c lib/src/pio_sim_internal.h \
    lib/include/*.h lib/config/*.h \
    tests/test_*.c tests/fuzz_pio_asm.c tests/consumer/main.c

# Static analysis (clean, WarningsAsErrors)
cmake -B build -G Ninja -DPIO_SIM_PLATFORM=RP2350 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
clang-tidy -p build lib/src/*.c

# Sanitizers
cmake -B build-san -G Ninja -DCMAKE_C_COMPILER=clang \
  -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all"
cmake --build build-san && ctest --test-dir build-san

# Fuzz the assembler (finds parser crashes/UB)
cmake -B build-fuzz -G Ninja -DCMAKE_C_COMPILER=clang -DPIO_SIM_BUILD_FUZZERS=ON
cmake --build build-fuzz --target fuzz_pio_asm
mkdir -p /tmp/finds && ./build-fuzz/tests/fuzz_pio_asm -max_total_time=30 /tmp/finds tests/fuzz_corpus

# Coverage (CI gates at 80% total lines)
cmake -B build-cov -G Ninja -DCMAKE_C_COMPILER=clang -DPIO_SIM_COVERAGE=ON
cmake --build build-cov
( cd build-cov && LLVM_PROFILE_FILE="cov-%p.profraw" ctest )
llvm-profdata merge -sparse build-cov/cov-*.profraw -o cov.profdata
llvm-cov report $(find build-cov -name 'test_pio_*' -type f ! -name '*.profraw' | sed 's/^/-object /') \
  -instr-profile=cov.profdata -ignore-filename-regex='(tests|third_party|unity)'
```

## Conventions

- **Fidelity first.** This models real RP2040/RP2350 silicon; cite the datasheet
  section for any behavioural change, and add a `.pio` corpus program or unit
  test that pins it. New assembler syntax must match real `pioasm` (the
  differential enforces this).
- Match the surrounding style; `clang-format` (config in `.clang-format`) is
  authoritative. Keep the `-Werror` warning set clean.
- RP2040 vs RP2350 differences go through the `PIO_SIM_HAS_*` gates in
  `lib/config/pio_sim_config.h`, never a bare version check.
- The library is single-threaded and dependency-free; keep it that way.

## Pull requests

Branch off `main`, keep commits focused, and make sure the full CI matrix is
green — build (gcc + clang-21 × RP2040/RP2350), sanitizers, clang-tidy, format,
the pioasm differential, fuzz, the 80% coverage gate, and the install-consumer
smoke.
