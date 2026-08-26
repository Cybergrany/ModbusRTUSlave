# ModbusRTUSlave tests

Run the complete standalone software gate from the repository root:

```sh
PLATFORMIO_CORE_DIR=/absolute/path/to/a/writable/platformio-core \
scripts/run_validation_gates.sh
```

The gate verifies normal and advanced public-header profiles, native protocol
fixtures, strict host and AVR C++11 journal builds, performance-script parsing,
and package behavior.

## Public-header compile gates

`ModbusRTUSlavePublicHeaderCompileGate.cpp` includes only
`<ModbusRTUSlave.h>`. It is compiled in the standalone profile and in the full
mutex, journal, work-state, and diagnostics profile. The implementation is
also compiled in both profiles.

Compile the Arduino example for Nano/AVR with:

```sh
PLATFORMIO_CORE_DIR=/absolute/path/to/a/writable/platformio-core \
SCONSFLAGS=-j8 \
pio ci examples/ModbusRTUSlaveExample --lib . --board nanoatmega328
```

## Protocol characterization

Run only the native protocol suite with:

```sh
PLATFORMIO_CORE_DIR=/absolute/path/to/a/writable/platformio-core \
pio test -e native_modbus_tests
```

The 30 cases cover:

- exact FC 1/2/3/4 responses and FC 5/6/15/16 mutations and acknowledgements;
- CRC, foreign-unit, exception, malformed targeted-broadcast, and receive
  resynchronization behavior;
- silent broadcast reads and no-response broadcast writes;
- admission, mutation, immutable publication, reply, and retirement order;
- policy-free bridge defaults with no reserved coil or register addresses;
- local callbacks, rejected writes, queue saturation, chunking, and tokens;
- T3.5 boundaries, partial-frame deadlines, clock rollover, cooperative
  transmit drain, UART flush, and DE transitions; and
- clock, serial, mutex, flush, diagnostic, object-size, and operation counts.

## Ingress journal

The journal oracle performs more than 655,000 deterministic checks. It covers
all-or-nothing reservation, publication visibility, immutable chunk snapshots,
custom entry traits, invalid tickets, token rollover, fixed footprint, and zero
allocation. The instantiated journal also passes strict host and AVR C++11
compile gates.

## Performance checks

The suite checks deterministic operation counts and representative host timing
for idle polling, local single writes, forwarded single writes, and multiple
register chunk boundaries. Paired comparisons use medians and CPU affinity
when available. Host scheduling can still perturb wall time, so operation
counts and fixed object-size checks remain the stronger evidence.

For an interleaved comparison of two already-built native test programs:

```sh
scripts/compare_modbus_native_perf.sh BASELINE_PROGRAM CANDIDATE_PROGRAM
```

The comparator requires both current raw timing lines and checks all five
lanes with identical operation counts. Set
`MBUS_RTU_SLAVE_MAX_REGRESSION_PCT` to change the default 5% limit.

## Packaging

```sh
PLATFORMIO_CORE_DIR=/absolute/path/to/a/writable/platformio-core \
pio pkg pack . --output /tmp/modbusrtuslave.tar.gz
```

## Hardware boundary

These gates cover protocol bytes, calculated timing, ordering, object layout,
fixed capacity, and representative compilation. They cannot prove UART ISR
latency, real drain duration, DE electrical edges, cross-core mutex behavior,
RS485 contention, or application scheduling. Those require target and bus
validation.

The queued-two-frames parser limitation is documented, not characterized as
passing. Any repair needs a dedicated buffered-frame fixture and physical
timing validation.
