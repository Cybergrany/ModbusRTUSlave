# ModbusRTUSlave compatibility gates

Run the complete software gate from the repository root:

```sh
OGM_SLAVE_CORE_REFERENCE=/path/to/OGM_slave_core \
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable \
scripts/run_compatibility_gates.sh
```

The optional reference must be checked out at
`73925642c29a0f419b2b3cb160647dee71f4c078`. With no reference argument, the
gate still checks the three embedded-source SHA-256 values and the public
include-forwarder hash frozen in
`scripts/check_ogm_source_parity.sh` and `ogm-fork-lock.json`.

## What is frozen

The 29-case production characterization suite compiles the carried
`ModbusRTUSlave.cpp` directly against a deterministic fake Arduino clock and
serial port. Its bridge-mode limits match production
(`MAX_MULTI_COILS=64`, `MAX_MULTI_HRS=32`). It pins:

- exact FC 1/2/3/4 response bytes and FC 5/6/15/16 mutation/ACK bytes;
- CRC rejection, foreign-unit rejection, exception responses, malformed
  targeted broadcasts and resynchronization;
- silent broadcast reads, ordinary broadcast mutation without response, and
  targeted FC `0x45` filtering/mutation without response;
- admission -> table mutation -> immutable journal publication -> ACK order;
- local-write notification, rejected-write nonmutation, queue saturation,
  chunking, source/overflow tokens, cross-table order and exact retirement;
- T3.5 completion at the exact boundary, partial-frame deadlines,
  `micros()` rollover, cooperative TX drain, flush and DE transition order;
- exact serial/clock/mutex/flush call counts, diagnostic event order, fixed
  host object sizes and generous normal-run performance tripwires.

When `OGM_SLAVE_CORE_REFERENCE` is supplied, the gate also compiles the
embedded and packaged production translation units independently with the same
`-Os` bridge/mutex/diagnostic profile and requires their object files to be
byte-identical. This catches include-resolution or package-layout side effects
that a source-only comparison could miss.

The journal oracle performs 655,560 deterministic checks against a frozen
executable model of the pre-extraction two-ring implementation. It also proves
all-or-nothing reservation, publication invisibility, immutable chunk
snapshots, custom compatibility records, stale/foreign ticket rejection,
token rollover, fixed footprint and zero allocation. The same instantiated
journal is compiled with strict host C++11 and AVR `g++`. Those two strict
compile-only results apply to the standalone journal template, not the entire
Arduino package.

Run only the production characterization suite with:

```sh
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable \
pio test -e native_modbus_tests
```

## Same-host strict performance mode

The ordinary suite uses deterministic operation counts and broad wall-time
tripwires so normal CI load does not create false failures. For a quiet-host
release check, opt into the baseline-derived absolute ceilings:

```sh
OGM_STRICT_MODBUS_PERF=1 \
OGM_SLAVE_CORE_REFERENCE=/path/to/OGM_slave_core \
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable \
scripts/run_compatibility_gates.sh
```

The strict lane uses medians and, where available, CPU affinity. Current
ceilings inherited unchanged from the source oracle are 20 ns/op for idle
poll, 780 ns/op for a complete local FC6 request/TX cycle, 900 ns/op for an
admitted forwarded FC6, 3,600 ns/op at the 32-register FC16 chunk boundary,
and 6,400 ns/op for a 65-register/three-entry FC16. The bridge-mode host ABI is
also fixed at 16,144 bytes for `ModbusRTUSlave` and 76 bytes for
`BridgePending`.

`scripts/compare_modbus_native_perf.sh` compares built characterization
executables. `scripts/compare_modbus_forwarded_pio_perf.sh` builds two
worktrees, alternates seven paired samples and rejects a median regression over
5%. Both provide `--self-test` parser/statistic checks. Because this candidate
carries the executable OGM source byte-for-byte, source parity is the primary
migration proof; timing comparisons guard compiler/package side effects.

## Embedded compile and package checks

The complete package and maintained example compile under the Nano/AVR Arduino
toolchain's C++11 settings:

```sh
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable SCONSFLAGS=-j8 \
pio ci examples/ModbusRTUSlaveExample --lib . --board nanoatmega328
```

Validate the PlatformIO package surface with:

```sh
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable \
pio pkg pack . --output /tmp/modbusrtuslave-ogm.tar.gz
```

## Physical boundary

These gates prove carried source identity, deterministic software behavior,
calculated timing and representative compilation. They cannot prove UART ISR
latency, actual drain duration, DE electrical edges, cross-core mutex behavior,
RS485 contention or application scheduling. This branch makes no hardware
acceptance claim and does not authorize a consumer or deployed-slave cutover.
