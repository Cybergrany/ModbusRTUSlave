# ModbusRTUSlave

An Arduino Modbus RTU slave/server library maintained by OpenGameMaster and
derived from [CMB27/ModbusRTUSlave](https://github.com/CMB27/ModbusRTUSlave).
The default build is self-contained apart from Arduino and exposes one portable
public header:

```cpp
#include <ModbusRTUSlave.h>
```

> [!IMPORTANT]
> Version `2.1.0-ogm.1` is the clean-layout software candidate. Its behavior is
> frozen by differential tests and an exact optimized-object comparison, but
> no OGM consumer or physical RS485 topology has yet been cut over to it. Pin
> the final reviewed tag or full commit, not a moving branch.

## What this fork adds

The fork keeps the familiar CMB27 2.x array-backed API while carrying the
behavior developed and deployed by OpenGameMaster:

- cooperative, nonblocking request parsing and transmit completion through
  `poll()` plus `tx_pump()`;
- ordinary Modbus broadcast writes with no response and silent broadcast
  reads;
- OGM targeted broadcast function `0x45`, wrapping FC 5, 6, 15 or 16 for one
  selected unit without a reply;
- rollover-safe RTU idle and transmit deadlines, target-specific UART flush
  compensation, and an AVR-safe byte-to-word conversion;
- optional platform-provided table mutexes, work-state accessors, statistics,
  USB diagnostics, and bridge transmit diagnostics;
- an allocation-free transactional ingress journal for durable bridge
  forwarding, including admission-before-mutation and immutable post-write
  snapshots;
- frozen wire, ordering, timing, footprint, C++11, packaging, and performance
  validation.

The optional OGM features are compile-gated. A normal Arduino consumer does
not need OGM_Portable, external-board concepts, an RTOS, or dynamic allocation.
See [src/README.md](src/README.md) for their exact adapter contracts.

## Lineage and branch policy

This repository retains the complete CMB27 history. The maintained OGM line
branches from CMB27 commit
`65ae4dd4cf121f42a3a9daa917034e319ebed65e`, the self-contained 2.x revision
from which OGM evolved. `main` is intended to be the public OGM-maintained
fork; current CMB27 work is tracked through the `upstream` remote for review.

CMB27 3.x decomposes the implementation across ModbusADU and ModbusSlaveLogic
and changes the API. It must not be mechanically merged into this line. Any
future reconciliation is a separately designed and validated migration.

The pre-layout `2.0.6-ogm.1` candidate remains frozen at
`05c70b32625bba97691a3902e21f4316cf4689d6` on the historical `ogm/compat`
line. Exact anchors and carried changes are recorded in
[OGM_FORK_PROVENANCE.md](OGM_FORK_PROVENANCE.md) and
[`ogm-fork-lock.json`](ogm-fork-lock.json).

## Installation

Pin an immutable reviewed tag or full 40-character commit:

```ini
lib_deps =
  https://github.com/Cybergrany/ModbusRTUSlave.git#<validated-tag-or-commit>
```

For a coordinated local migration, use a path dependency and test the package
and consumer changes together:

```ini
lib_deps =
  symlink:///absolute/path/to/ModbusRTUSlave
```

Do not add ModbusADU or ModbusSlaveLogic to this 2.x-derived package. The
standalone implementation is intentionally self-contained.

## Migrating from the pre-layout candidate

`2.1.0-ogm.1` deliberately removes the temporary `Comms/` compatibility
layout. This is a source-path cleanup, not a protocol or runtime change.

| Before (`2.0.6-ogm.1`) | After (`2.1.0-ogm.1`) | Action |
| --- | --- | --- |
| `#include <ModbusRTUSlave.h>` | `#include <ModbusRTUSlave.h>` | No change for ordinary library users. |
| `#include "Comms/ModbusRTUSlave.h"` | `#include <ModbusRTUSlave.h>` | Update every include. The old path is intentionally absent. |
| Embedded `src/Comms/ModbusRTUSlave.cpp` | Package-owned `src/ModbusRTUSlave.cpp` | Remove or compile-exclude the embedded implementation when adding the package. |
| `Comms/ModbusRTUIngressJournal.h` | Library-internal `detail/ModbusRTUIngressJournal.h` | Do not treat the journal path as a stable consumer API. |

An OGM ownership cutover must be atomic: adding this package while leaving an
embedded `ModbusRTUSlave.cpp` active creates duplicate definitions. Include
guards cannot protect separate translation units.

## Basic usage

Configure each backing table before `begin()`. Call both service methods
frequently: `poll()` receives and queues responses, while `tx_pump()` completes
the calculated UART drain/flush sequence and returns driver-enable low.

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
ModbusRTUSlave modbus(Serial1, kDriverEnablePin);
}

void setup() {
  modbus.configureCoils(coils, 16);
  modbus.configureDiscreteInputs(discreteInputs, 16);
  modbus.configureHoldingRegisters(holdingRegisters, 32);
  modbus.configureInputRegisters(inputRegisters, 32);
  modbus.begin(kUnitId, kBaud, SERIAL_8N1);
}

void loop() {
  modbus.poll();
  modbus.tx_pump();
}
```

Supported standard functions are FC 1, 2, 3, 4, 5, 6, 15 and 16. Valid
configured unit IDs are 1 through 247; address zero is reserved for broadcast.
`HardwareSerial` is supported, plus `Serial_` when the Arduino core defines
`HAVE_CDCSERIAL`. OGM disabled the earlier AVR `SoftwareSerial` surface at its
first import, and this fork does not restore it.

## Behavioral contract

The clean-layout migration must preserve:

- request acceptance, CRC and range validation, exception codes, mutations,
  response bytes, and broadcast silence;
- admission, mutation, callback, journal publication, reply, and forwarding
  order;
- T1.5/T3.5 calculations, receive cleanup, UART write/flush and driver-enable
  transitions;
- mutex spans, diagnostics, fixed-capacity behavior, object layout, stack,
  flash, RAM, and hot-path performance.

The canonical package object is byte-identical to the optimized embedded
bridge-profile object after the expected include-path and comment-only source
normalization. The characterization suite passes 29 wire/order/timing cases;
the journal oracle performs at least 655,560 deterministic checks. See
[test/README.md](test/README.md) for reproducible commands and the exact
software-versus-hardware boundary.

## Known RX framing limitation

The underlying Arduino `Stream` API reports queued bytes without physical
arrival timestamps. If application scheduling delays one `poll()` until two
complete RTU frames are already buffered, their real T3.5 gap is no longer
observable; the current drain can concatenate them and reject the combined
bytes as one CRC-invalid frame. This is inherited behavior, not introduced by
the public-layout migration.

The exact investigation note from OGM slave commit
`0ad325f333204225468d1f7c4ae65a408a1bf87b` (merged by
`02b93e2898a5456486d4ee8fbcacde9973526b6d`) is retained beside
`_readRequest()`. A future fix must characterize two pre-buffered valid ADUs,
including FC `0x45` followed by unicast, and preserve trailing bytes using
length-aware extraction or UART timing metadata. Suppressing the CRC statistic
alone is not a fix.

## License

This fork is MIT licensed. CMB27-derived source and attribution are documented
in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and
[LICENSES/CMB27-ModbusRTUSlave-MIT.txt](LICENSES/CMB27-ModbusRTUSlave-MIT.txt).
