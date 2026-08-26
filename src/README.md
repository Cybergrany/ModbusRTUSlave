# Compatibility API and integration contract

The executable implementation in `Comms/ModbusRTUSlave.h` and
`Comms/ModbusRTUSlave.cpp` is a byte-for-byte copy of the implementation in
`OGM_slave_core` commit
`73925642c29a0f419b2b3cb160647dee71f4c078`. Keeping those files unchanged is
intentional: this first package proves a repository move, not a protocol or
timing rewrite. `ModbusRTUSlave.h` is only a public include forwarder, so both
of these forms name the same class:

```cpp
#include <ModbusRTUSlave.h>          // normal Arduino-library use
#include <Comms/ModbusRTUSlave.h>    // temporary OGM migration compatibility
```

## Required service loop

Receive parsing and transmit completion are cooperative and nonblocking. Call
both methods frequently; `poll()` may queue a response and assert DE, while
`tx_pump()` waits for the calculated drain deadline, flushes the UART, applies
the target compensation delay where configured, and deasserts DE.

```cpp
#include <Arduino.h>
#include <ModbusRTUSlave.h>

namespace {
constexpr uint8_t kUnitId = 7;
constexpr uint8_t kDriverEnablePin = 2;
constexpr unsigned long kBaud = 115200UL;

bool coils[16] = {};
bool discreteInputs[16] = {};
uint16_t holdingRegisters[32] = {};
uint16_t inputRegisters[32] = {};
ModbusRTUSlave slave(Serial1, kDriverEnablePin);
}

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
}
```

`begin()` accepts IDs 1 through 247, calculates T1.5/T3.5 from the baud and
serial format, starts the selected UART, initializes DE low, and discards any
bytes already buffered. The compatibility implementation supports
`HardwareSerial` and, when the core defines `HAVE_CDCSERIAL`, `Serial_`.
`SoftwareSerial` was disabled at OGM's first import and is deliberately not
restored by this replay.

## Wire behavior

The implementation handles FC 1, 2, 3, 4, 5, 6, 15 and 16. Address-zero read
requests are ignored. Address-zero writes mutate the configured table without
transmitting a reply. OGM FC `0x45` wraps FC 5, 6, 15 or 16 with a target unit;
only the named unit mutates and no unit replies. A unicast FC `0x45` receives
the ordinary illegal-function exception.

Frames complete only after at least the calculated T3.5 idle interval. CRC,
length, address, quantity and table-range failures retain the exact frozen OGM
handling. `tx_pump()` uses unsigned receive-time arithmetic and signed due-time
comparison so the characterized `micros()` rollover behavior is preserved.

## Optional OGM compile-time surfaces

These flags reproduce current OGM integration. They are off for a standalone
slave and deliberately add no default object state or runtime work.

| Flag | Contract |
| --- | --- |
| `OGM_USE_MUTEX` | Requires `platform/PlatformMutex.h`. Four optional table mutex pointers may be passed to `configurePlatformMutex()`; `nullptr` leaves that table unlocked. The bridge journal uses `SafePlatformMutex` from the same header. |
| `OGM_BRIDGE_MODE` | Enables durable admitted-write snapshots and overflow records. The build must provide `PinIndexDefines::MAX_MULTI_COILS` and `MAX_MULTI_HRS` through `IO/ExternalPins/PinIndexDefines.h`. Current production values are 64 and 32. |
| `OGM_MODBUS_MT_ACCESSORS` | Adds `workState()` and `hasWorkPending()` for a scheduler without changing parser progress. |
| `USB_DEBUG` | Adds per-instance counters and timestamps returned by `debugInfo()`. |
| `BRIDGE_UPSTREAM_TX_DIAG=1` | With bridge mode, records accepted -> queued -> pump -> done counts and latency buckets. `copyAndResetBridgeUpstreamTxDiag()` is the only read/reset operation. |
| `USING_STATS` | Requires the OGM-owned `Pins/SlaveStats.h` adapter. Protocol and bridge rejections are reported there; the statistics store remains outside this library. |
| `MODBUS_PURGE_RX_AFTER_TX` | Discards receive bytes after TX completion. It is intentionally opt-in because it changes receive recovery behavior. |
| `MODBUS_RTU_SLAVE_BUF_SIZE` | Overrides the default 256-byte request/response buffer before including the header. |

The OGM bridge callbacks are static because that is the established embedded
contract. Configure them before admitting upstream writes:

```cpp
bool admit(uint16_t start, uint16_t count, bool isCoil,
           bool fireForget, uint16_t& generation);
bool isLocal(uint16_t start, uint16_t count, bool isCoil);
void localWrite(uint16_t start, uint16_t count, bool isCoil);

ModbusRTUSlave::setBridgeAdmissionFn(&admit);
ModbusRTUSlave::setBridgeLocalRangeFn(&isLocal);
ModbusRTUSlave::setBridgeLocalWriteFn(&localWrite);
```

For an admitted non-local write, ordering is reserve capacity -> mutate the
table under its mutex -> publish an immutable snapshot -> queue the Modbus
reply. `bridgePeekNext()` preserves acceptance order across coil and holding
rings. Retire only with `bridgeCommitNext()` and the exact source token after
the destination accepted the record; a full destination must leave the source
entry uncommitted.

`BridgePending`, the callback names, and the OGM header paths are compatibility
surfaces rather than a redesigned generic API. Removing them or introducing a
new platform abstraction is intentionally deferred until after the exact
source package has passed consumer and physical validation.

## Supporting journal

`Comms/ModbusRTUIngressJournal.h` is allocation-free, C++11, and independent of
Arduino, RTOS, pins, and game concepts. Its reservation/publish/retire contract
is documented inline. The fork retains its exact OGM source bytes and tests it
against the frozen pre-extraction ring oracle on both host and AVR compilers.
