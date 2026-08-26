# OpenGameMaster fork provenance

This repository retains CMB27's complete Git ancestry while presenting the
additional OpenGameMaster work as a maintained, portable library. The intended
long-lived branch model is:

- `main`: the reviewed OGM-maintained implementation and public API;
- `upstream/main`: current CMB27 development, fetched for comparison;
- historical `ogm/compat`: frozen pre-layout replay retained until the clean
  migration has completed final review.

The OGM line is based on the self-contained CMB27 2.x implementation. Current
CMB27 3.x uses ModbusADU and ModbusSlaveLogic and is a separate architecture;
it must not be mechanically merged into this fork.

## Immutable anchors

| Purpose | Commit |
| --- | --- |
| Current CMB27 head reviewed while seeding the fork | `83c2b50e32c0d10ea8e300761f1bbd058eb9d1bd` |
| Historical CMB27 branch point used by OGM | `65ae4dd4cf121f42a3a9daa917034e319ebed65e` |
| First source import into `OGM_slave_core` | `6ce585ddfcaba7e4517700f858c53410b935caae` |
| Frozen executable source replay | `73925642c29a0f419b2b3cb160647dee71f4c078` |
| Frozen pre-layout candidate | `05c70b32625bba97691a3902e21f4316cf4689d6` |
| RX-framing investigation note | `0ad325f333204225468d1f7c4ae65a408a1bf87b` |
| RX-framing note merge into `OGM_slave_core` | `02b93e2898a5456486d4ee8fbcacde9973526b6d` |

At first import, OGM disabled AVR `SoftwareSerial`; the remaining source
matched the CMB27 branch point. The exact initial/current hashes and optional
adapter providers are machine-readable in [`ogm-fork-lock.json`](ogm-fork-lock.json).

## What OGM added

The following is established behavior carried from OGM, not new logic written
for the clean-layout migration:

| Theme | OGM history |
| --- | --- |
| Self-contained 2.x import and disabled AVR `SoftwareSerial` | `6ce585dd` |
| FC `0x45` targeted broadcast, standard broadcast silence, nonblocking RX/T3.5, cooperative TX | `d506e894` through `bdade566`, including `e15ffc1f` |
| Platform-selected table mutexes and per-instance RX/TX state | `7954a7b4`, `d6bfa4b8`, `1d6ea3db` |
| Optional statistics and USB diagnostics | `a35aee02`; `a7784845` through `3b100642` |
| Bridge admission, local notifications, source/overflow queues and write metadata | `6eded705` through `ba958619` |
| Exact-token durable handoff and immutable passthrough snapshots | `79363fd2`, `cf04bd1f` |
| GIGA bridge 8N1 workaround, AVR byte conversion, scheduler state, TX diagnostics | `0cbbb65b`, `d93a383a`, `c3a90468`, `ecf83b66` |
| Allocation-free transactional ingress journal | `19c3af6a` |

Against the CMB27 branch point, the carried OGM header measures 405
insertions/10 deletions and the implementation 1,503 insertions/115 deletions.
The fine-grained OGM history remains authoritative; this table is only a map.

## Clean public-layout migration

The pre-layout package `2.0.6-ogm.1` intentionally copied three embedded files
byte-for-byte under `src/Comms/` and added a root include forwarder. That was a
useful source-identity checkpoint, but it obscured the public fork behind OGM's
old internal directory structure.

Version `2.1.0-ogm.1` removes that temporary compatibility layout:

| Canonical file | Role | SHA-256 |
| --- | --- | --- |
| `src/ModbusRTUSlave.h` | Real portable public declaration | `c53562fe91a353b4bbbbf7febd65f826d700d3900f6e6431cc5ec57fa054e687` |
| `src/ModbusRTUSlave.cpp` | Package-owned implementation | `506257c11f891e5a839f177dda96b92ffd439232336e082ced3a444486d4c30f` |
| `src/detail/ModbusRTUIngressJournal.h` | Internal neutral transactional journal | `f9d08a82db5f349610db43115089c9e916e3a59024d0d189ebcbbe9e8eec381e` |

The allowed delta from `OGM_slave_core` `73925642` is deliberately narrow:

1. move the declaration from `Comms/ModbusRTUSlave.h` to
   `ModbusRTUSlave.h` and update its journal include;
2. move the implementation to `src/ModbusRTUSlave.cpp` and include the public
   header by its canonical name;
3. move the unchanged neutral journal under `src/detail/`;
4. remove two inherited trailing spaces while moving the real public header;
5. carry the exact comment-only RX-framing TODO from `0ad325f3` beside
   `_readRequest()`.

No function body, branch, constant, declaration, register shape, object
layout, call order, platform operation, or wire rule changed. The normalized
source comparator fails closed if anything outside those transformations
moves. Independently compiled optimized bridge-profile objects are
byte-identical before and after the layout migration.

The old `Comms/` public path is intentionally absent and a negative compile
gate enforces its removal. Ordinary consumers already using
`<ModbusRTUSlave.h>` require no source edit; embedded OGM consumers must update
their includes and atomically transfer `.cpp` ownership to this package.

## Optional OGM adapter boundary

The standalone library depends only on Arduino. Optional OGM features retain
small compile-time provider contracts rather than importing product concepts:

- `OGM_USE_MUTEX` obtains `PlatformMutex` and `SafePlatformMutex` from
  `platform/PlatformMutex.h`;
- `OGM_BRIDGE_MODE` obtains snapshot limits from
  `IO/ExternalPins/PinIndexDefines.h`;
- `USING_STATS` reports through application-owned `Pins/SlaveStats.h`.

The current `BridgePending` record and static bridge callbacks remain because
changing them would be an API, timing, and footprint migration. External-board,
child-routing, clock, GPIO, serial, and scheduling abstractions do not belong in
this mechanical path pass.

The neutral ingress journal is already independent of Arduino, RTOS, pin maps,
and game concepts. Its `detail/` placement states that consumers should use it
through `ModbusRTUSlave`; it does not alter its compiled semantics.

## Known RX framing debt

The TODO introduced by `0ad325f3` and merged at `02b93e28` is retained verbatim
beside `_readRequest()`. Arduino `Stream` exposes queued bytes but not their
physical arrival times. When scheduling delays service until two valid ADUs are
buffered, the parser can lose their T3.5 boundary, concatenate them, and reject
both as one CRC-invalid frame even though another slave serviced promptly and
accepted the same traffic.

This migration does not claim to fix that behavior. A future change must first
queue two CRC-valid ADUs before one service pass—including FC `0x45` followed
by unicast—and require both to be extracted. Candidate mechanisms are
function-length-aware ADU peeling with retained trailing bytes or UART
idle/per-byte timing metadata. Hiding the CRC counter is explicitly
insufficient. Because this changes parser behavior and real timing, it needs a
dedicated software characterization and physical RS485 gate.

## Validation state

The clean-layout candidate passes:

- reviewed canonical hashes and normalized source comparison;
- standalone and full OGM bridge-profile root-header/implementation compiles;
- an enforced failure for the removed `Comms/ModbusRTUSlave.h` path;
- exact optimized bridge-profile object parity with the embedded source;
- 29 production wire, CRC, broadcast, exception, ordering, T3.5, TX and
  footprint scenarios;
- at least 655,560 deterministic journal checks plus host and AVR C++11
  compile gates;
- strict same-host performance ceilings, comparator self-tests, and the paired
  forwarded-write comparison against the pre-layout candidate;
- the maintained Nano/AVR C++11 example and PlatformIO package export.

The coordinated software ownership cutover is complete in `OGM_slave_core`
`344d4b05dcf08cf6098f26c4150436a1722b6c79`, published as `OGM_Slave` `2.0.0`.
That consumer pins this library at
`776b7e0147f495585fd368e10fcd81f81534ba8f` and `OGM_Portable` at
`a9e98e849b4227159be6b3c527b1a32be14394d2`. The reviewed consumer evidence is:

- 10/10 OGM pin/ordering policy tests;
- 29/29 package-linked Modbus characterization cases;
- exactly one packaged `ModbusRTUSlave.cpp` object and one `poll()` definition;
- strict performance and comparator gates; and
- a clean `OGM_Slave` archive with no Git/worktree/build metadata or deleted
  embedded Modbus implementation.

This establishes a singular package owner and a software-valid consumer tuple.
It is not firmware artifact or hardware acceptance. A release still requires:

1. rebuild the exact dependent OGM slave and bridge trees and compare artifacts, symbols,
   stack, flash and RAM;
2. validate the resulting consumer on the physical slave/bridge RS485
   topology, including compatibility with deployed original slave firmware.

## Upstream review policy

Fetch upstream without rewriting the maintained fork:

```sh
git fetch upstream --prune --tags
git log --left-right --graph main...upstream/main
```

Review and port upstream changes individually when they apply. Do not merge or
rebase the 3.x line into the 2.x-derived OGM implementation. Releases are
consumed through immutable tags or full commits, never a moving branch.
