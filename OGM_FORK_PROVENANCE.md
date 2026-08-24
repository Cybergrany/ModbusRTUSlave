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

## Initial release rule

The seed is for provenance inspection and packaging checks. It is not an OGM
consumer candidate until the functional replay, software parity gates and
physical RS485 validation have been completed against immutable revisions.

