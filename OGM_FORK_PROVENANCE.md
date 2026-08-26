# OpenGameMaster fork provenance

This repository retains the complete CMB27 Git history. Its long-lived lines
have deliberately different purposes:

- `main` tracks `upstream/main` without OGM changes.
- `ogm/compat` starts at the historical CMB27 revision used as the OGM slave
  source base. Reviewed OGM compatibility changes are replayed only here.

Do not merge or rebase current upstream into `ogm/compat`. CMB27 3.x split the
slave across ModbusADU and ModbusSlaveLogic and changed the API. Reconciling
that redesign is a separate migration, not part of this repository move.

## Immutable anchors

| Purpose | Commit |
| --- | --- |
| CMB27 revision reviewed before fork seeding | `83c2b50e32c0d10ea8e300761f1bbd058eb9d1bd` |
| `ogm/compat` branch point | `65ae4dd4cf121f42a3a9daa917034e319ebed65e` |
| First source import in `OGM_slave_core` | `6ce585ddfcaba7e4517700f858c53410b935caae` |
| Functional replay source | `73925642c29a0f419b2b3cb160647dee71f4c078` |

At first import OGM disabled AVR `SoftwareSerial`; the remaining imported
source matched the CMB27 branch point. The initial and current hashes, later
upstream lineage references, optional OGM adapter hashes, and validation state
are machine-readable in [`ogm-fork-lock.json`](ogm-fork-lock.json).

## Current compatibility status

The functional replay is implemented as package version `2.0.6-ogm.1`. The
three production files are byte-for-byte copies of the source anchor:

| Candidate file | SHA-256 |
| --- | --- |
| `src/Comms/ModbusRTUSlave.h` | `cc6145185e889a344bfabd7dbb3a591fb5e67cc4f0490b02091e2006ff576c3d` |
| `src/Comms/ModbusRTUSlave.cpp` | `7844e6f6a1f8a8818f3044af5db07847dc244aa5ae4f2cfa46b73538b68c18aa` |
| `src/Comms/ModbusRTUIngressJournal.h` | `f9d08a82db5f349610db43115089c9e916e3a59024d0d189ebcbbe9e8eec381e` |

Commit `0e63f6f7bd1e51974a6dbccf18585213d6c57cbe` performs the source transfer.
The only production packaging addition is `src/ModbusRTUSlave.h`, a 13-line
include forwarder. It preserves both `<ModbusRTUSlave.h>` for normal Arduino
users and `"Comms/ModbusRTUSlave.h"` for a later OGM ownership cutover. No
function body, conditional branch, constant, declaration, register shape, data
layout, call order, or platform call changed.

Against the historical 2.x branch point, the carried OGM header measures 405
insertions/10 deletions and the implementation 1,503 insertions/115 deletions.
Against the validated embedded OGM source, executable drift is 0 lines and all
three files compare byte-for-byte. The large historical divergence is therefore
visible and reviewable without being reinterpreted during packaging.

The candidate passes the frozen 29-scenario production characterization, the
655,560-check ingress-journal oracle, strict host and AVR C++11 compiles, fixed
object-size checks, an exact independently compiled `-Os` bridge-mode object
comparison, strict same-host performance ceilings, a Nano/ATmega328 example
build, and PlatformIO package validation. That makes it a software
candidate for an isolated consumer cutover. It is not a compatibility release:
no consumer has switched to it and no slave/bridge hardware validation is
claimed.

## Functional divergence ledger

This ledger describes behavior already present in the exact source anchor. It
does not describe new edits made by this repository transfer.

| Established OGM theme | Source history |
| --- | --- |
| Initial self-contained 2.x import; AVR `SoftwareSerial` disabled | `6ce585dd` |
| FC `0x45` targeted broadcast, silent ordinary broadcasts, nonblocking RX framing/T3.5 and cooperative TX completion | `d506e894` through `bdade566`, including `e15ffc1f` timing trim |
| Platform-selected table mutexes and per-instance RX/TX state | `7954a7b4`, `d6bfa4b8`, `1d6ea3db` |
| Optional slave statistics and USB diagnostics | `a35aee02`; `a7784845` through `3b100642` |
| Bridge admission, local-range notifications, source/overflow queues, operation accounting and fire-forget/public-debt metadata | `6eded705` through `ba958619` |
| Exact-token durable source handoff and immutable passthrough snapshots | `79363fd2`, `cf04bd1f` |
| GIGA bridge 8N1 one-argument `begin()` workaround | `0cbbb65b` |
| AVR-safe 16-bit byte-to-word conversion | `d93a383a` |
| Optional work-state and upstream TX diagnostics | `c3a90468`, `ecf83b66` |
| Allocation-free transactional ingress journal | `19c3af6a` |

The full OGM history remains the authoritative fine-grained ledger. The source
anchor and hashes prevent this thematic summary from concealing a code delta.

## Intentional package-only divergences

1. Production files live under `src/Comms/` so their established include paths
   and bytes remain intact; a top-level public header forwards to them.
2. Package metadata uses the unambiguous prerelease identity
   `2.0.6-ogm.1` and the Cybergrany repository URL. It does not reuse CMB27's
   materially different `2.0.6` release identity.
3. The maintained example removes the unavailable `SoftwareSerial` option,
   calls `tx_pump()` after `poll()`, and documents cooperative TX completion.
4. Documentation, frozen host fixtures, C++11/AVR compile gates, package
   checks and performance comparators are added. They do not compile into
   consumers.

No other transfer delta is intentional. Run
`scripts/check_ogm_source_parity.sh` to fail closed if a production byte moves.

## Optional OGM adapter boundary

The standalone build needs only Arduino. Existing compile-time OGM features
retain their exact include contracts rather than copying product code into this
fork:

- `OGM_USE_MUTEX` obtains `PlatformMutex` and `SafePlatformMutex` from
  `platform/PlatformMutex.h` in OGM_Portable.
- `OGM_BRIDGE_MODE` obtains only the two snapshot bounds from
  `IO/ExternalPins/PinIndexDefines.h` in OGM_Portable.
- `USING_STATS` reports through the OGM-owned `Pins/SlaveStats.h` adapter.

The exact providing commits and hashes are pinned in the lock file. This means
the first compatibility candidate still exposes established `BridgePending`
and static bridge callbacks behind `OGM_BRIDGE_MODE`. That is deliberate
compatibility debt: replacing those names or introducing a new clock/GPIO/
serial abstraction now would create the very semantic, timing and footprint
drift this transfer is meant to avoid. Such cleanup should follow a successful
source-ownership cutover as a separately gated change.

The neutral `ModbusRTUIngressJournal.h` is the exception: it is already free of
Arduino, RTOS, pin-map and game dependencies, and belongs with the slave logic
that uses it.

## Remote layout

```text
origin    git@github.com:Cybergrany/ModbusRTUSlave.git
upstream  https://github.com/CMB27/ModbusRTUSlave.git
```

Refresh the upstream tracking line and publish compatibility work without
rewriting either history:

```bash
git fetch upstream --prune --tags
git push origin upstream/main:main
git push -u origin ogm/compat
```

The `upstream/main:main` push must remain fast-forward-only in practice. If it
is rejected, inspect the fork divergence; never force-push either long-lived
line.

## Replay and release policy

1. Preserve the exact source anchor for the repository move. Any subsequent
   logic, API, platform, formatting or optimization change is a new reviewed
   commit with its own differential gates.
2. Keep optional OGM product dependencies compile-gated. Do not make
   external-board, child, route or game concepts part of standalone behavior.
3. Preserve request/response bytes, broadcast silence, admission/mutation/
   publication/ACK order, T3.5 and TX timing, lock spans, diagnostics, object
   layout and hot-path performance.
4. Pin releases by immutable tag or full commit. Never consume `main` or
   `ogm/compat` by moving branch name.
5. Reconcile CMB27 3.x or neutralize the retained compatibility adapters only
   as separate migrations after this exact-source package is proven.

## Compatibility release gates

A revision on `ogm/compat` is suitable for an OGM release only after all of the
following evidence is recorded against one exact dependency tuple:

1. Source provenance and package hashes remain exact.
2. Valid, invalid, exception, CRC, truncated, overlength and broadcast fixtures
   retain their frozen mutations, responses and statuses.
3. Admission, mutation, callback, journal publication, reply and forwarding
   order remain exact, including rejected and no-response requests.
4. Frame boundaries, TX drain, flush, DE transitions and cleanup retain their
   trace and hardware timing.
5. Hot paths, stack, RAM, flash and fixed-capacity behavior are measured on the
   relevant AVR and GIGA consumers.
6. The exact OGM slave and bridge trees build from clean immutable pins, and
   their artifacts are compared with the embedded-source baseline.
7. The candidate runs on the physical slave/bridge RS485 topology. Existing
   deployed slave compatibility remains part of that test.

Passing the current software gates supports an isolated consumer migration
because no behavior delta is expected. It does not replace the consumer-build
or physical gates, and this branch deliberately records both as pending.
