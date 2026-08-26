# ModbusRTUSlave

`ModbusRTUSlave` is an Arduino Modbus RTU slave/server library. It exposes
caller-owned coil and register arrays through a cooperative, allocation-free
service loop.

```cpp
#include <ModbusRTUSlave.h>
```

## When to use it

Use this library to:

- expose sensor values and actuator controls to a Modbus master;
- implement a small remote-I/O node with fixed memory use;
- keep receive parsing and transmit completion nonblocking;
- accept standard broadcast writes without replying;
- accept targeted no-response writes from a compatible master; or
- capture accepted writes for a separate store-and-forward worker.

## What this fork adds

- **Cooperative transmit completion.** `poll()` parses requests and starts a
  reply; `tx_pump()` finishes drain timing and releases driver enable. This is
  useful when the application loop must continue servicing other work. See
  [the service-loop contract](src/README.md#required-service-loop).
- **Broadcast behavior.** Standard unit-zero writes update the configured
  table without a response, while broadcast reads remain silent.
- **Targeted broadcast.** FC `0x45` carries FC 5, 6, 15, or 16 for one unit
  without a reply. This can update one device on a multidrop bus while keeping
  fire-and-forget ordering. See [wire behavior](src/README.md#wire-behavior).
- **Robust timing edges.** RTU idle and transmit deadlines remain safe across
  the wrapping Arduino microsecond clock, with target-specific drain handling
  where required.
- **Optional integration hooks.** Compile-time mutexes, work-state accessors,
  statistics, diagnostics, and a durable ingress journal are available while
  the default build stays small. See
  [optional compile-time features](src/README.md#optional-compile-time-features).
- **Transactional bridge ingress.** Accepted non-local writes can be reserved,
  copied into immutable snapshots, and retired in source order by a separate
  worker. See [ingress journal](src/README.md#ingress-journal).
- **Behavior and performance tests.** Wire bytes, mutation order, timing
  boundaries, rollover, journal behavior, fixed capacity, object size, and
  operation counts are characterized. See [the test guide](test/README.md).

## Quick start

Configure each table before `begin()`. The arrays must remain alive while the
slave uses them.

```cpp
#include <Arduino.h>
#include <ModbusRTUSlave.h>

constexpr uint8_t kUnitId = 7;
constexpr int8_t kDriverEnablePin = 2;
constexpr unsigned long kBaud = 115200UL;

bool coils[16] = {};
bool discreteInputs[16] = {};
uint16_t holdingRegisters[32] = {};
uint16_t inputRegisters[32] = {};

ModbusRTUSlave slave(Serial1, kDriverEnablePin);

void setup() {
  slave.configureCoils(coils, 16);
  slave.configureDiscreteInputs(discreteInputs, 16);
  slave.configureHoldingRegisters(holdingRegisters, 32);
  slave.configureInputRegisters(inputRegisters, 32);
  slave.begin(kUnitId, kBaud, SERIAL_8N1);
}

void loop() {
  slave.poll();
  slave.tx_pump();

  inputRegisters[0] = analogRead(A0);
}
```

Call both service methods frequently. Long blocking work in `loop()` can delay
frame recognition or driver-enable release.

## Supported operations

| Function | Table/action |
| --- | --- |
| FC 1 | Read Coils |
| FC 2 | Read Discrete Inputs |
| FC 3 | Read Holding Registers |
| FC 4 | Read Input Registers |
| FC 5 | Write Single Coil |
| FC 6 | Write Single Holding Register |
| FC 15 | Write Multiple Coils |
| FC 16 | Write Multiple Holding Registers |

Valid unit IDs are 1 through 247. Unit zero is reserved for broadcast.
FC `0x45` is an extension, not a standard Modbus function; both ends must
support it.

## Advanced integration

The default API needs only Arduino and the four caller-owned tables. Optional
features can add table mutexes, parser work-state accessors, counters, and
durable write snapshots for a downstream worker. These features are selected
at compile time and may require application-provided adapter headers. Their
exact contracts are documented in [the API guide](src/README.md).

The journal under `src/detail/` is an implementation detail. Integrations
should use the public `ModbusRTUSlave` methods rather than include that header
directly.

## Known framing limitation

Arduino `Stream` does not retain the physical arrival time of bytes that are
already queued. If two complete RTU frames are buffered before one `poll()`,
their real inter-frame gap may be lost and the current parser can reject the
combined bytes as one CRC-invalid frame. Applications should service `poll()`
frequently. Any parser change needs both buffered-frame tests and target UART
validation.

## Examples and reference

- [Complete Arduino example](examples/ModbusRTUSlaveExample/ModbusRTUSlaveExample.ino)
- [Detailed API and integration guide](src/README.md)
- [Public header](src/ModbusRTUSlave.h)
- [Validation guide](test/README.md)

## Testing

Run the software validation from the repository root:

```sh
PLATFORMIO_CORE_DIR=/absolute/path/to/a/writable/platformio-core \
scripts/run_validation_gates.sh
```

The tests cover protocol behavior and calculated timing. They cannot prove
electrical RS485 behavior or UART scheduling on every board.

## License

MIT. See [LICENSE](LICENSE). Third-party notices are in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
