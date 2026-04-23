# STM32N6 MX66UW1G45G Flash Algorithm

A [probe-rs](https://probe.rs/) flash algorithm for the Macronix MX66UW1G45G 1Gbit SPI NOR flash on the **STM32N6570-DK** discovery board.

The flash is connected via **XSPI2** (NCS1 / port PN1). This algorithm should also work on other boards using the same flash chip on the same XSPI2 port.

## Building

```
cargo build
```

## Testing

Requires a connected STM32N6570-DK with an ST-Link probe:

```
cargo run
```

This builds the algorithm and runs the probe-rs flash test suite (sector erase, chip erase, page program, verify).

### probe-rs version requirement

Needs **probe-rs ≥ 0.32** (unreleased at the time of writing). The STM32N6 reset-sequence fix (probe-rs PR #3843) landed after the 0.31.0 release; without it `target-gen test` hardfaults during flash-algorithm init with `The core entered an unexpected status: LockedUp`.

Until 0.32 ships, install `probe-rs` and `target-gen` from master:

```
cargo install --git https://github.com/probe-rs/probe-rs probe-rs-tools target-gen --locked
```

## Usage

After building, flash a binary to the external NOR flash with probe-rs:

```
probe-rs download --chip-description-path target/definition.yaml --chip STM32N657 STM32N6570-DK_OoB.hex --binary-format ihex
```

## How it works

The algorithm initialises the STM32N6 peripherals (clocks, power, GPIO, XSPIM, RISAF) and drives XSPI2 in indirect SPI mode for erase/program/read operations. On teardown (`Drop`) it switches XSPI2 back to memory-mapped mode so the debugger and application code can read flash at `0x70000000`.

Built with the [`flash-algorithm`](https://github.com/probe-rs/flash-algorithm) crate and [`stm32-metapac`](https://github.com/embassy-rs/stm32-data) (PAC-only, no HAL runtime).
