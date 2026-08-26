# Public API and integration contract

The entire supported consumer include surface is:

```cpp
#include <ModbusRTUSlave.h>
```

`src/ModbusRTUSlave.h` is the real declaration and
`src/ModbusRTUSlave.cpp` is its implementation. There is deliberately no
`Comms/ModbusRTUSlave.h` compatibility forwarder. The allocation-free ingress
journal lives under `src/detail/`; its file path is an implementation detail,
not an independently versioned consumer API.

## Required service loop

Receive parsing and transmit completion are cooperative and nonblocking. Call
both methods frequently. `poll()` may queue a response and assert DE;
`tx_pump()` waits for the calculated drain deadline, flushes the UART, applies
target compensation where configured, and deasserts DE.

```cpp
#include <Arduino.h>
#include <ModbusRTUSlave.h>

namespace {
bool coils[16] = {};
bool discreteInputs[16] = {};
uint16_t holdingRegisters[32] = {};
uint16_t inputRegisters[32] = {};
ModbusRTUSlave slave(Serial1, 2);
}

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

`begin()` accepts IDs 1 through 247, calculates T1.5/T3.5 from baud and serial
format, starts the selected UART, initializes DE low, and discards bytes already
buffered. `HardwareSerial` is supported and, when the core defines
`HAVE_CDCSERIAL`, so is `Serial_`. `SoftwareSerial` is not supported on this
fork.

## Wire behavior

The implementation handles FC 1, 2, 3, 4, 5, 6, 15 and 16. Address-zero read
requests are ignored. Address-zero writes mutate the configured table without
transmitting a reply. OGM FC `0x45` wraps FC 5, 6, 15 or 16 with a target unit;
only that unit mutates and no unit replies. A unicast FC `0x45` receives the
ordinary illegal-function exception.

Frames complete after at least the calculated T3.5 idle interval. CRC, length,
address, quantity and table-range failures retain the frozen OGM handling.
`tx_pump()` uses unsigned receive-time arithmetic and signed due-time comparison
so the characterized `micros()` rollover behavior is preserved.

## Optional compile-time surfaces

These switches are off for an ordinary standalone consumer. They add no
default runtime work or object state.

| Flag | Contract |
| --- | --- |
| `OGM_USE_MUTEX` | Requires `platform/PlatformMutex.h`. Pass four optional table mutex pointers to `configurePlatformMutex()`; `nullptr` leaves that table unlocked. Bridge journal locking uses `SafePlatformMutex` from the same provider. |
| `OGM_BRIDGE_MODE` | Enables durable admitted-write snapshots and overflow records. The build provides `PinIndexDefines::MAX_MULTI_COILS` and `MAX_MULTI_HRS` through `IO/ExternalPins/PinIndexDefines.h`. |
| `OGM_MODBUS_MT_ACCESSORS` | Adds `workState()` and `hasWorkPending()` for cooperative scheduling without changing parser progress. |
| `USB_DEBUG` | Adds per-instance counters and timestamps returned by `debugInfo()`. |
| `BRIDGE_UPSTREAM_TX_DIAG=1` | With bridge mode, records accepted -> queued -> pump -> done counts and latency buckets. `copyAndResetBridgeUpstreamTxDiag()` snapshots and clears them. |
| `USING_STATS` | Requires the application-owned `Pins/SlaveStats.h` adapter. Protocol and bridge rejection counters remain outside this library. |
| `MODBUS_PURGE_RX_AFTER_TX` | Discards receive bytes after TX completion. It is opt-in because it changes recovery behavior. |
| `MODBUS_RTU_SLAVE_BUF_SIZE` | Overrides the default 256-byte request/response buffer before the public header is included. |

The OGM bridge callbacks remain static because that is the established ABI:

```cpp
bool admit(uint16_t start, uint16_t count, bool isCoil,
           bool fireForget, uint16_t& generation);
bool isLocal(uint16_t start, uint16_t count, bool isCoil);
void localWrite(uint16_t start, uint16_t count, bool isCoil);

ModbusRTUSlave::setBridgeAdmissionFn(&admit);
ModbusRTUSlave::setBridgeLocalRangeFn(&isLocal);
ModbusRTUSlave::setBridgeLocalWriteFn(&localWrite);
```

For an admitted non-local write, ordering is reserve capacity -> mutate under
the table mutex -> publish an immutable snapshot -> queue the Modbus reply.
`bridgePeekNext()` preserves acceptance order across coil and holding rings.
Retire only with `bridgeCommitNext()` and the exact source token after the
destination accepts the record. A full destination must leave the source entry
uncommitted.

`BridgePending` and the static callback names are retained OGM integration
surfaces. Neutralizing them or introducing a different platform/serial API is
future behavior work and must not be folded into this path migration.

## Ingress journal

`detail/ModbusRTUIngressJournal.h` is allocation-free, C++11, and independent
of Arduino, RTOS, pins, and game concepts. Its reservation/publish/retire
contract is documented inline. It is intentionally nested under `detail`
because consumers should integrate through `ModbusRTUSlave`, not bind to its
storage file layout.

## Known parser limitation

If two complete RTU ADUs are already buffered before one `poll()`, Arduino
`Stream` does not preserve the physical T3.5 gap between them. `_readRequest()`
can therefore drain both into one candidate frame and reject the concatenation
on CRC. The full investigation TODO from commits `0ad325f3` / `02b93e28` is
retained at that method. A future parser change needs an explicit two-frame
fixture and physical UART/RS485 validation; this layout migration does not try
to solve it.
