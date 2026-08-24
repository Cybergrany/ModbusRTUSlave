# OpenGameMaster fork provenance

This repository retains the complete CMB27 Git history. Its long-lived lines
have deliberately different purposes:

- `main` tracks `upstream/main` without OGM changes.
- `ogm/compat` starts at the historical CMB27 revision used as the OGM slave
  source base. Reviewed OGM compatibility changes are replayed only here.

Do not recreate `ogm/compat` from the latest upstream API. The upstream 3.x
line decomposes the slave across additional libraries, whereas OGM forked the
self-contained 2.x implementation. Mixing that redesign into the repository
move would invalidate the no-behaviour-change migration gate.

## Immutable anchors

| Purpose | Commit |
| --- | --- |
| CMB27 revision reviewed before fork seeding | `83c2b50e32c0d10ea8e300761f1bbd058eb9d1bd` |
| `ogm/compat` branch point | `65ae4dd4cf121f42a3a9daa917034e319ebed65e` |
| First source import in `OGM_slave_core` | `6ce585ddfcaba7e4517700f858c53410b935caae` |

At import, OGM disabled the AVR `SoftwareSerial` declarations and calls by
commenting them out. All other slave source lines match the CMB27 branch point.
The exact upstream and imported file hashes are recorded in
[`ogm-fork-lock.json`](ogm-fork-lock.json).

## Current compatibility status

The initial `ogm/compat` seed adds provenance and package metadata only. Its
slave source files remain identical to the historical CMB27 branch point, and
`ogm_functional_replay` in `ogm-fork-lock.json` remains `not_started`. The AVR
`SoftwareSerial` import delta and all later OGM changes still have to be
replayed as separately reviewed commits.

The compatibility seed is self-contained. The pinned ModbusADU and
ModbusSlaveLogic revisions in the lock file document the later upstream 3.x
decomposition; they are lineage references, not runtime dependencies of this
2.x seed and must not be silently introduced during the compatibility replay.

The reviewed reference pins are:

| Repository | Commit | Upstream tag | Role in this seed |
| --- | --- | --- | --- |
| CMB27/ModbusADU | `7cb0e24f0abe86bc83e114325d75fe7a7d878562` | `1.0.2` | Reference for the later 3.x ADU extraction; not linked. |
| CMB27/ModbusSlaveLogic | `85b579741c7772588b532f277ded569f9d8fcbeb` | `1.0.1` | Reference for the later 3.1 logic extraction; not linked. |

## Remote layout

```text
origin    git@github.com:Cybergrany/ModbusRTUSlave.git
upstream  https://github.com/CMB27/ModbusRTUSlave.git
```

The Cybergrany repository is maintained as a GitHub fork of CMB27. Refresh the
upstream tracking line and publish compatibility work without rewriting either
history:

```bash
git fetch upstream --prune --tags
git push origin upstream/main:main
git push -u origin ogm/compat
```

The `upstream/main:main` push must remain fast-forward-only in practice. If Git
rejects it, inspect the fork divergence; do not force-push either long-lived
line.

## Replay and release policy

1. Replay current OGM slave behavior as small, reviewable themes. Record the
   exact source OGM commit or commit range in each replay commit message.
2. Keep GPIO, clocks, serial drain, locks, diagnostics and threading behind
   neutral platform contracts. Keep external-board, child, bridge-route and
   game concepts out of the public Modbus slave API.
3. Keep the OGM ingress journal and product forwarding policy in adapters unless
   a neutral callback contract is required; preserve admission, mutation and
   durable-record ordering at that boundary.
4. Pin releases by immutable tag or commit. Never consume `main` or
   `ogm/compat` by moving branch name.
5. Reconcile the newer CMB27 3.x decomposition only as a separate, explicitly
   test-gated migration after the compatibility package is proven.

## Compatibility release gates

A revision on `ogm/compat` is suitable for an OGM consumer only after all of
the following evidence is recorded against the exact dependency tuple:

1. Source provenance: every replay commit names its source OGM commit(s), the
   package lock is current, and consumer manifests resolve immutable commits.
2. Protocol parity: exhaustive valid, invalid, exception, CRC, truncated,
   overlength and broadcast fixtures produce the frozen OGM mutations,
   responses and statuses for all supported function codes.
3. Ordering parity: admission, backing-store mutation, observer callback,
   ingress-journal record/commit and bridge forwarding retain their established
   order, including rejected and no-response requests.
4. Timing parity: inter-character/frame boundaries, response delay, serial
   write/drain, DE/RE transitions, post-delay and cleanup match trace oracles.
5. Resource/performance parity: paired hot-path gates pass and stack, RAM and
   flash changes are measured and accepted on the relevant AVR and GIGA builds.
6. Consumer validation: native suites plus exact OGM slave, bridge and master
   firmware builds pass from clean dependency caches. Legacy slave firmware
   remains build- and protocol-compatible where it is part of the baseline.
7. Hardware validation: the release candidate is exercised over the physical
   GIGA/RS485 topology with unchanged deployed slave firmware.

Passing gates 1 through 6 supports a hardware checkpoint because the migration
should not alter on-wire behavior; it does not replace gate 7. Likewise,
hardware that merely appears playable does not replace exhaustive trace,
ordering, timing and performance gates.
