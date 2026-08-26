# Third-party notices

This repository includes source derived from the ModbusRTUSlave Arduino
library by Christopher Bulliner. The original work is licensed under the MIT
License. Its complete notice is retained in
[`LICENSES/CMB27-ModbusRTUSlave-MIT.txt`](LICENSES/CMB27-ModbusRTUSlave-MIT.txt).

## ModbusRTUSlave

- Source project: https://github.com/CMB27/ModbusRTUSlave
- Original author: Christopher Bulliner
- Historical source basis: `65ae4dd4cf121f42a3a9daa917034e319ebed65e`
- Maintained source: `src/ModbusRTUSlave.h` and `src/ModbusRTUSlave.cpp`
- License notice: `LICENSES/CMB27-ModbusRTUSlave-MIT.txt`

Maintained additions include targeted broadcast FC `0x45`, cooperative
receive and transmit service, optional platform synchronization and
diagnostics, and durable fixed-capacity ingress queues.
