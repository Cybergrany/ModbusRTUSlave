# Public API and integration contract

The supported consumer include is:

```cpp
#include <ModbusRTUSlave.h>
```

`ModbusRTUSlave.h` declares the public class and `ModbusRTUSlave.cpp` provides
the implementation. The journal under `src/detail/` is an implementation
detail rather than a separate public include.

## Required service loop

Receive parsing and transmit completion are cooperative. Call both methods
frequently. `poll()` may parse a request, mutate a table, and start a reply.
`tx_pump()` waits for the calculated drain deadline, flushes the UART, applies
target compensation where configured, and returns driver enable low.

```cpp
#include <Arduino.h>
#include <ModbusRTUSlave.h>

bool coils[16] = {};
bool discreteInputs[16] = {};
uint16_t holdingRegisters[32] = {};
uint16_t inputRegisters[32] = {};
ModbusRTUSlave slave(Serial1, 2);

void setup() {
  slave.configureCoils(coils, 16);
  slave.configureDiscreteInputs(discreteInputs, 16);
  slave.configureHoldingRegisters(holdingRegisters, 32);
  slave.configureInputRegisters(inputRegisters, 32);
  slave.begin(7, 115200UL, SERIAL_8N1);
}

void loop() {
  slave.poll();
  slave.tx_pump();
}
```

`begin()` accepts IDs 1 through 247, calculates T1.5/T3.5, starts the selected
UART, initializes DE low, and discards bytes already buffered. `HardwareSerial`
is supported. `Serial_` is also supported when the Arduino core defines
`HAVE_CDCSERIAL`. `SoftwareSerial` is not supported.

## Wire behavior

The implementation handles FC 1, 2, 3, 4, 5, 6, 15, and 16. Unit-zero reads
are ignored. Unit-zero writes mutate the configured table without a reply.

FC `0x45` is a targeted-broadcast extension. It wraps FC 5, 6, 15, or 16 with
a target unit. Only that unit mutates, and no unit replies. A unicast FC
`0x45` receives an illegal-function exception.

Frames complete after at least the calculated T3.5 idle interval. The
deadline calculations are safe across the wrapping Arduino microsecond clock.

## Optional compile-time features

The default build adds no platform mutex, journal, statistics adapter, or
diagnostics state. Guarded declarations in
[`ModbusRTUSlave.h`](ModbusRTUSlave.h) provide these optional features:

- one caller-provided mutex per Modbus table;
- `workState()` and `hasWorkPending()` for cooperative schedulers;
- request, response, timing, and transmit diagnostics;
- application-owned statistics adapters;
- optional receive purging after transmit; and
- durable admitted-write snapshots for a store-and-forward worker.

Build flags that change declarations, object layout, or behavior must be
defined consistently for the application and the library source.

## Bridge callbacks and ordering

An advanced integration can provide static callbacks for write admission,
local-range classification, and local-write notification:

```cpp
bool admit(uint16_t start, uint16_t count, bool isCoil,
           bool fireAndForget, uint16_t& generation);
bool isLocal(uint16_t start, uint16_t count, bool isCoil);
void localWrite(uint16_t start, uint16_t count, bool isCoil);

ModbusRTUSlave::setBridgeAdmissionFn(&admit);
ModbusRTUSlave::setBridgeLocalRangeFn(&isLocal);
ModbusRTUSlave::setBridgeLocalWriteFn(&localWrite);
```

For an admitted non-local write, the order is:

1. reserve journal capacity;
2. mutate the table under its configured lock;
3. publish an immutable snapshot; and
4. queue the Modbus reply when the request expects one.

`bridgePeekNext()` preserves acceptance order across coil and holding-register
rings. Retire an item with `bridgeCommitNext()` only after the destination has
accepted it, using the exact source token returned by the peek. If the
destination is full, leave the source item uncommitted.

## Ingress journal

`detail/ModbusRTUIngressJournal.h` is allocation-free and C++11-compatible. It
stores reservations, immutable value chunks, and overflow records in fixed
compile-time capacity. Consumers should access it through
`ModbusRTUSlave` so journal storage remains an internal detail.

## Known parser limitation

If two complete RTU frames are already buffered before one `poll()`, Arduino
`Stream` does not preserve the physical T3.5 gap between them. `_readRequest()`
can drain both into one candidate and reject the combined bytes on CRC.

The source retains a detailed TODO beside `_readRequest()`. A fix must preserve
the trailing frame, characterize two buffered valid ADUs including targeted
broadcast followed by unicast, and be validated with physical UART timing.
Ignoring the CRC statistic alone would not fix the parser.
