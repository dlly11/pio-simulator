# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

_Nothing yet._

## [0.1.1] - 2026-07-11

### Fixed

- **DMA** — `pio_dma_channel_configure()` now clamps an out-of-range `CHx_CTRL`
  `DATA_SIZE` on the register-write path, not only in the
  `channel_config_set_transfer_data_size()` setter. A caller that hand-built the
  config struct could set `data_size` past 32-bit, making the transfer engine
  shift a word by up to 56 bits (undefined behaviour) and read/write past the
  4-byte transfer word. `DATA_SIZE` is a 2-bit field on silicon, so the model now
  truncates it at the boundary, matching the sibling `RING_SIZE`/`CHAIN_TO` guards.
- **Assembler** — `in status, <n>` is now accepted (encodes `0x40a8`), matching
  real `pioasm`; it was previously rejected in error. The simulator already reads
  `STATUS` as an `IN` source, and the encoding is now pinned bit-exact against
  `pioasm` in the differential corpus.

### Changed

- **CI** — the fuzz job now points crash/leak/oom/timeout reproducers at a
  dedicated directory (`-artifact_prefix`) and uploads them as a build artifact
  when a run fails, so a red fuzz leg yields a downloadable minimized input
  instead of discarding it with the runner.

## [0.1.0] - 2026-07-11

First tagged release: a complete, pure-C11 functional simulator and
pioasm-compatible assembler for the RP2040 / RP2350 PIO block, with the chip
around it (pads, DMA, clock tree) modelled too.

### Added

- **PIO simulator** — the full instruction set (JMP, WAIT, IN, OUT, PUSH, PULL,
  MOV, IRQ, SET) with side-set, per-instruction delay, OSR/ISR shift registers
  (autopush/autopull with configurable thresholds), joinable 4-deep TX/RX FIFOs,
  scratch registers, and rate-exact per-SM fractional clock dividers — stepped one
  system clock per `pio_sim_tick()`.
- **pioasm-compatible assembler** — every mnemonic, operand form and directive,
  full integer expressions over defines and labels, cross-checked **bit-exact**
  against the real `pioasm` in CI.
- **Shared pad model** — PADS_BANK0 pad registers and the IO_BANK0 FUNCSEL mux
  (output/OE/input overrides), with several state machines and an external device
  model driving and sampling the same wires under hardware-exact arbitration.
- **DMA controller** — 12 channels (RP2040) / 16 (RP2350): address generation with
  ring wrap, 8/16/32-bit transfers, byte swap, DREQ pacing (PIO TX/RX, fractional
  timers, unpaced), chaining, per-channel IRQs, and the CRC sniffer.
- **Clock tree** — XOSC → PLL_SYS → clk_sys with datasheet-validated limits and
  exact 64-bit tick⇄time conversion.
- **Chip umbrella** — every PIO block + shared pads + DMA + clock tree behind one
  `pio_chip_tick()`.
- **RP2040 / RP2350 targets** — selected with `PIO_SIM_PIO_VERSION` (or the CMake
  `PIO_SIM_PLATFORM`); the v1-only surface is compiled out of a v0 build, so using
  it on an RP2040 target is a compile error rather than a silent surprise.
- **Reentrancy** — no mutable global state and no allocation; all state lives in
  caller-owned structs, so independent instances don't interfere.
- **Packaging** — CMake `install` + `find_package(pio_sim)` linking
  `pio_sim::pio_sim`, with the platform surface pinned via a PUBLIC compile
  definition.
- **Docs & examples** — six standalone, CI-run reference programs and a generated
  Doxygen API reference (published to GitHub Pages).
- **CI** — build (gcc + clang-21) × RP2040/RP2350, ASan/UBSan, clang-tidy,
  cppcheck, clang-format, the pioasm differential, libFuzzer harnesses
  (assembler, execution core, DMA), and an 80% line-coverage gate.

[Unreleased]: https://github.com/dlly11/pio-simulator/compare/v0.1.1...HEAD
[0.1.1]: https://github.com/dlly11/pio-simulator/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/dlly11/pio-simulator/releases/tag/v0.1.0
