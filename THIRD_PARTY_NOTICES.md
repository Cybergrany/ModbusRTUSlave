# Third-party notices

This compatibility line contains source derived from CMB27's
ModbusRTUSlave Arduino library. The original work is licensed under the MIT
License; the complete notice is retained in
`LICENSES/CMB27-ModbusRTUSlave-MIT.txt` and is identical to the repository's
root `LICENSE`.

## ModbusRTUSlave

- Upstream: https://github.com/CMB27/ModbusRTUSlave
- Original author: Christopher Bulliner
- Historical basis: `65ae4dd4cf121f42a3a9daa917034e319ebed65e`
- Initial OGM import: `6ce585ddfcaba7e4517700f858c53410b935caae`
- Functional replay source: OGM_slave_core
  `73925642c29a0f419b2b3cb160647dee71f4c078`
- Candidate paths: `src/Comms/ModbusRTUSlave.h` and
  `src/Comms/ModbusRTUSlave.cpp`
- License: `LICENSES/CMB27-ModbusRTUSlave-MIT.txt`

The initial OGM copy differed only by disabling SoftwareSerial sections.
Subsequent OGM changes include targeted broadcast FC `0x45`, nonblocking
receive/transmit service, optional platform synchronization and diagnostics,
and bridge durability queues. The exact history and source hashes are recorded
in `OGM_FORK_PROVENANCE.md` and `ogm-fork-lock.json`. This notice establishes
lineage; it does not imply that current OGM behavior is identical to an
upstream release.
