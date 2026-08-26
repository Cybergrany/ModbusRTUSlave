# ModbusRTUSlave validation gates

Run the complete software gate from the repository root:

```sh
OGM_STRICT_MODBUS_PERF=1 \
OGM_SLAVE_CORE_REFERENCE=/path/to/OGM_slave_core \
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable \
scripts/run_validation_gates.sh
```

The optional reference must be checked out exactly at
`73925642c29a0f419b2b3cb160647dee71f4c078`. Without it, the gate still checks
the reviewed canonical source hashes, the absence of `src/Comms` forwarders,
the RX-framing TODO, both public-header compile profiles, all native fixtures,
strict C++11 journal compiles, performance comparators, and package behavior.

With the reference, `scripts/check_ogm_layout_migration.sh` normalizes only the
three approved nonfunctional changes before comparing source:

1. `Comms/ModbusRTUSlave.h` becomes the real `ModbusRTUSlave.h`;
2. the journal include moves from `Comms/` to `detail/`;
3. two inherited trailing spaces are removed from the moved public header;
4. the comment-only RX-framing TODO from `0ad325f3` / `02b93e28` is present.

The gate then independently compiles the embedded and canonical package
translation units under the same optimized bridge/mutex/diagnostic profile and
requires byte-identical object files. This catches macro resolution, inline
definition, symbol, layout, or code-generation drift that path normalization
alone could miss.

## Public-layout compile gates

`ModbusRTUSlavePublicHeaderCompileGate.cpp` includes only
`<ModbusRTUSlave.h>`. It is compiled once in the ordinary standalone profile
and once with all OGM bridge-facing switches. The full implementation is also
compiled in both profiles. A negative compile proves that
`<Comms/ModbusRTUSlave.h>` no longer resolves.

The maintained Arduino example supplies a complete Nano/AVR C++11 package
compile:

```sh
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable SCONSFLAGS=-j8 \
pio ci examples/ModbusRTUSlaveExample --lib . --board nanoatmega328
```

## Frozen behavior

The 29-case production characterization suite compiles the canonical source
against a deterministic fake Arduino clock and serial port. Its bridge limits
match production (`MAX_MULTI_COILS=64`, `MAX_MULTI_HRS=32`). It pins:

- exact FC 1/2/3/4 response bytes and FC 5/6/15/16 mutation/ACK bytes;
- CRC, foreign-unit, exception, malformed FC `0x45`, and resynchronization
  behavior;
- silent broadcast reads, broadcast mutation without reply, and targeted
  broadcast filtering/mutation without reply;
- admission -> mutation -> immutable journal publication -> ACK order;
- local callbacks, rejected writes, queue saturation, chunking, tokens,
  cross-table ordering, and exact retirement;
- exact T3.5 boundaries, partial-frame deadlines, `micros()` rollover,
  cooperative TX drain, UART flush, and DE transitions;
- serial/clock/mutex/flush call counts, diagnostic ordering, fixed host object
  sizes, and deterministic operation counts.

Run only this suite with:

```sh
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable \
pio test -e native_modbus_tests
```

The standalone journal oracle performs at least 655,560 deterministic checks
against a frozen executable model of the pre-extraction rings. It proves
all-or-nothing reservation, publication invisibility, immutable chunk
snapshots, custom records, stale/foreign ticket rejection, token rollover,
fixed footprint, and zero allocation. The instantiated journal separately
passes strict host and AVR C++11 compile gates.

## Strict performance lane

The opt-in lane uses medians and, where available, CPU affinity. The inherited
ceilings are 20 ns/op for idle poll, 780 ns/op for a local FC6 request/TX cycle,
900 ns/op for an admitted forwarded FC6, 3,600 ns/op at the 32-register FC16
chunk boundary, and 6,400 ns/op for a 65-register/three-entry FC16. Bridge-mode
host sizes remain 16,144 bytes for `ModbusRTUSlave` and 76 bytes for
`BridgePending`.

`scripts/compare_modbus_native_perf.sh` compares built executables.
`scripts/compare_modbus_forwarded_pio_perf.sh` alternates paired samples from
two worktrees and rejects a median regression above 5%. Both scripts include
parser/statistic self-tests.

Wall-time samples can still be perturbed by host scheduling. A single marginal
failure with identical operations is rerun on the quiet pinned lane; thresholds
must not be relaxed merely to make a noisy run green.

## Packaging

Validate the exported library surface with:

```sh
PLATFORMIO_CORE_DIR=/path/to/.platformio_core_portable \
pio pkg pack . --output /tmp/modbusrtuslave-ogm.tar.gz
```

## Physical boundary

These gates prove the reviewed source delta, deterministic protocol behavior,
calculated timing, optimized object identity, and representative compilation.
They cannot prove UART ISR latency, actual drain duration, DE electrical edges,
cross-core mutex behavior, RS485 contention, or application scheduling. A
consumer cutover and physical slave/bridge validation remain separate release
gates.

The inherited queued-two-frames RX limitation is documented, not fixed or
characterized as passing here. Any repair needs a dedicated buffered-FC69 plus
unicast fixture and physical timing validation.
