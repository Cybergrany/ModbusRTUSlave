# ModbusRTUSlave

> [!IMPORTANT]
> This branch is the OpenGameMaster compatibility line, rooted at the exact
> CMB27 2.x revision used as OGM's slave source base. The functional source
> replay is implemented and software-gated, but no OGM consumer or hardware
> has been cut over. Do not treat it as a release until those remaining gates
> are recorded against an immutable commit or tag. See
> [OGM_FORK_PROVENANCE.md](OGM_FORK_PROVENANCE.md).

This repository preserves two intentionally different histories:

| Ref | Purpose | Consumer guidance |
| --- | --- | --- |
| `main` | Mirrors the current `CMB27/ModbusRTUSlave` line without OGM changes. | Use to review upstream work, not as an automatic OGM upgrade. |
| `ogm/compat` | Starts at CMB27 `65ae4dd4cf121f42a3a9daa917034e319ebed65e`, the self-contained 2.x source used as OGM's base. | Carries the exact OGM functional replay. Consume only an immutable validated tag or commit. |

The exact anchors, source hashes and replay policy are recorded in
[OGM_FORK_PROVENANCE.md](OGM_FORK_PROVENANCE.md) and
[`ogm-fork-lock.json`](ogm-fork-lock.json). Do not merge or rebase `main` into
`ogm/compat`: upstream 3.x decomposition is a separate behavior and API change.

## Compatibility-line status

The implementation, public declaration, and neutral ingress journal are
byte-for-byte copies of `OGM_slave_core`
`73925642c29a0f419b2b3cb160647dee71f4c078`. They remain under `src/Comms/`
so the transfer itself changes no executable source. A small top-level header
preserves the normal `<ModbusRTUSlave.h>` include. Both package manifests use
the distinct prerelease identity `2.0.6-ogm.1`.

The 29-case production characterization and 655,560-check journal oracle pin
wire bytes, CRC/error handling, broadcast silence, admission/mutation/snapshot/
ACK order, T3.5 and TX boundaries, rollover, fixed object sizes and strict
same-host performance. The journal passes strict host and AVR C++11 compile
gates; the complete package and maintained example pass the Nano/AVR Arduino
C++11 build. PlatformIO packaging also passes. See
[test/README.md](test/README.md).

No consumer manifest has changed and no hardware acceptance is claimed. Check
`ogm_functional_replay` in `ogm-fork-lock.json` for the exact evidence and
remaining gates.

## Installing a validated compatibility release

PlatformIO consumers should pin an immutable compatibility tag or full commit,
never a moving branch:

```ini
lib_deps =
  https://github.com/Cybergrany/ModbusRTUSlave.git#<validated-tag-or-40-char-commit>
```

The compatibility package is intentionally self-contained. Do not add the
newer upstream ModbusADU, ModbusRTUComm or ModbusSlaveLogic decomposition as
implicit dependencies: that would be an API and implementation migration, not
a repository-only move. The ADU and SlaveLogic commits in
`ogm-fork-lock.json` are lineage references only.

For local extraction work, use a path dependency so the consumer and library
changes are tested together:

```ini
lib_deps =
  symlink:///absolute/path/to/ModbusRTUSlave
```

> [!WARNING]
> A consumer cutover must transfer source ownership atomically. In the same
> consumer change that adds this package, remove or compile-exclude the
> embedded `include/Comms/ModbusRTUSlave.h`,
> `src/Comms/ModbusRTUSlave.cpp`, and
> `include/Comms/ModbusRTUIngressJournal.h`. Leaving the embedded `.cpp` active
> alongside the package creates competing definitions; include guards do not
> protect separate translation units. Do not remove OGM-owned adapter headers
> such as `Pins/SlaveStats.h` or the optional OGM_Portable platform/pin
> providers.

Before publishing a release, replace the local path with the immutable remote
revision and rebuild the exact OGM slave, bridge and master dependency trees
from clean dependency caches.

## Compatibility usage example

This implementation owns UART startup. Configure the backing arrays before
`begin()`, then call both `poll()` and `tx_pump()` frequently. `poll()` queues
responses without blocking; `tx_pump()` completes drain/flush and returns the
driver-enable pin low. A unit ID of zero is reserved for broadcast requests;
configure a normal slave with an ID from 1 through 247.

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

See [src/README.md](src/README.md) for targeted broadcasts, optional mutexes,
bridge ingress callbacks, diagnostics, compile flags, and their ordering
contracts.

## OGM compatibility contract

Moving slave protocol ownership into this repository must not change:

- accepted and rejected request bytes, CRC checks, address/range validation,
  response bytes or exception codes for function codes 1, 2, 3, 4, 5, 6, 15
  and 16;
- broadcast mutation-without-response behavior, malformed-frame handling, or
  when backing arrays become visible to OGM observers;
- inter-character/frame timeout boundaries, UART write/flush,
  driver-enable transitions, compensation delay or receive-buffer cleanup;
- request, mutation, callback, ingress-journal and bridge-forwarding order;
- lock spans, caller-visible status/diagnostics, fixed-capacity behavior, stack,
  RAM, flash or hot-path performance beyond the accepted gates;
- compatibility with the existing OGM master/bridge and deployed original
  slave firmware.

Native fixtures and byte comparisons can strongly constrain these properties,
but they cannot prove UART interrupt latency, RS485 electrical timing,
scheduler behavior, drain duration or bus contention. The compatibility tag
therefore still requires physical validation after its software gates pass.

## Historical upstream API documentation

The reference below is retained from the CMB27 2.x branch point. It contains
known API drift: notably, the OGM line has no `SoftwareSerial` constructor or
`setResponseDelay()`, and responses require cooperative `tx_pump()` service.
The pinned source, local example, and compatibility contract above are
authoritative.

Modbus is an industrial communication protocol. The RTU variant communicates over serial lines such as UART, RS-232, or RS-485. The full details of the Modbus protocol can be found at [modbus.org](https://modbus.org). A good summary can also be found on [Wikipedia](https://en.wikipedia.org/wiki/Modbus).

This is an Arduino library that implements the slave/server logic of the Modbus RTU protocol. This library implements function codes 1 (Read Coils), 2 (Read Discrete Inputs), 3 (Read Holding Registers), 4 (Read Input Registers), 5 (Write Single Coil), 6 (Write Single Holding Register), 15 (Write Multiple Coils), and 16 (Write Multiple Holding Registers).

This library will work with HardwareSerial, SoftwareSerial, or Serial_ (USB Serial on ATmega32u4 based boards). A driver enable pin can be set, enabling an RS-485 transceiver to be used. This library requires arrays for coils, discrete inputs, holding registers, and input registers to be passed to it. 


## Version Note
Version 2.x.x of this library is not backward compatible with version 1.x.x. Any sketches that were written to use a 1.x.x version of this library will not work with later versions, at least not without modification.


## Compatibility
This library has been succsessfully tested with the following boards:
- Arduino Leonardo
- Arduino Make Your UNO (USB had to be unplugged to work with HardwareSerial)
- Arduino Mega 2560
- Arduino Nano
- Arduino Nano 33 BLE
- Arduino Nano 33 IoT
- Arduino Nano ESP32
- Arduino Nano Every
- Arduino Nano RP2040 Connect - Using Earle F. Philhower's arduino-pico core
- Arduino UNO R3 SMD
- Arduino UNO R4

Problems were encountered with the following board:
- Arduino Nano RP2040 Connect - Using Arduino's ArduinoCore-mbed (Reliable communication could not be established with the master/client board)


## Example
- [ModbusRTUSlaveExample](examples/ModbusRTUSlaveExample/ModbusRTUSlaveExample.ino)


## Methods

### ModbusRTUSlave()

#### Description
Creates a ModbusRTUSlave object and sets the serial port to use for data transmission.
Optionally sets a driver enable pin. This pin will go `HIGH` when the library is transmitting. This is primarily intended for use with an RS-485 transceiver, but it can also be a handy diagnostic when connected to an LED.

#### Syntax
``` C++
ModbusRTUSlave(serial)
ModbusRTUSlave(serial, dePin)
```

#### Parameters
- `serial`: the serial port object to use for Modbus communication.
- `dePin`: the driver enable pin. This pin is set HIGH when transmitting. If this parameter is set to `NO_DE_PIN`, this feature will be disabled. Default value is `NO_DE_PIN`. Allowed data types: `uint8_t` or `byte`.

#### Example
``` C++
# include <ModbusRTUSlave.h>

const uint8_t dePin = 13;

ModbusRTUSlave modbus(Serial, dePin);
```

---


### configureCoils()

#### Description
Tells the library where coil data is stored and the number of coils.
If this function is not run, the library will assume there are no coils.

#### Syntax
``` C++
modbus.configureCoils(coils, numCoils)
```

#### Parameters
- `coils`: an array of coil values. Allowed data types: array of `bool`.
- `numCoils`: the number of coils. This value must not be larger than the size of the array. Allowed data types: `uint16_t`.

---


### configureDiscreteInputs()

#### Description
Tells the library where to read discrete input data and the number of discrete inputs.
If this function is not run, the library will assume there are no discrete inputs.

#### Syntax
``` C++
modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs)
```

#### Parameters
- `discreteInputs`: an array of discrete input values. Allowed data types: array of `bool`.
- `numDiscreteInputs`: the number of discrete inputs. This value must not be larger than the size of the array. Allowed data types: `uint16_t`.

---



### configureHoldingRegisters()

#### Description
Tells the library where holding register data is stored and the number of holding registers.
If this function is not run, the library will assume there are no holding registers.

#### Syntax
``` C++
modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters)
```

#### Parameters
- `holdingRegisters`: an array of holding register values. Allowed data types: array of `uint16_t`.
- `numHoldingRegisters`: the number of holding registers. This value must not be larger than the size of the array. Allowed data types: `uint16_t`.

---


### configureInputRegisters()

#### Description
Tells the library where to read input register data and the number of input registers.
If this function is not run, the library will assume there are no input registers.

#### Syntax
``` C++
modbus.configureInputRegisters(inputRegisters, numInputRegisters)
```

#### Parameters
- `inputRegisters`: an array of input register values. Allowed data types: array of `uint16_t`.
- `numInputRegisters`: the number of input registers. This value must not be larger than the size of the array. Allowed data types: `uint16_t`.

---


### begin()

#### Description
Sets the slave/server id and the data rate in bits per second (baud) for serial transmission.
Optionally it also sets the data configuration. Note, there must be 8 data bits for Modbus RTU communication. The default configuration is 8 data bits, no parity, and one stop bit.

#### Syntax
``` C++
modbus.begin(slaveId, baud)
modbus.begin(slaveId, baud, config)
```

#### Parameters
- `slaveId`: the number used to itentify this device on the Modbus network. Allowed data types: `uint8_t` or `byte`.
- `baud`: the baud rate to use for Modbus communication. Common values are: `1200`, `2400`, `4800`, `9600`, `16200`, `38400`, `57600`, and `115200`. Allowed data types: `uint32_t`.
- `config`: the serial port configuration to use. Valid values are:  
`SERIAL_8N1`: no parity (default)  
`SERIAL_8N2`  
`SERIAL_8E1`: even parity  
`SERIAL_8E2`  
`SERIAL_8O1`: odd parity  
`SERIAL_8O2`

_If using a SoftwareSerial port a configuration of `SERIAL_8N1` will be used regardless of what is entered._

---


### setResponseDelay()
Sets an optional response delay (in ms) for the slave (default 0).
If set to a non-zero value, the slave will wait for the specified number of milliseconds before sending the response.
This may be useful if tight control over the dePin from the master is not possible. Adding a delay will allow the master enough time to stop transmitting and avoid issues with multiple drivers on the the physical pins.


#### Syntax
```C++
modbus.setResponseDelay(responseDelay)
```

#### Parameters
- `responseDelay`: `unsigned long` number of milliseconds to wait before responding to requests.

---

### poll()

#### Description
Checks if any Modbus requests are available. If a valid request has been received, an appropriate response will be sent.
This function must be called frequently.

#### Syntax
``` C++
modbus.poll()
```

#### Parameters
None

#### Example
``` C++
# include <ModbusRTUSlave.h>

const uint8_t coilPins[2] = {4, 5};
const uint8_t discreteInputPins[2] = {2, 3};

ModbusRTUSlave modbus(Serial);

bool coils[2];
bool discreteInputs[2];

void setup() {
  pinMode(coilPins[0], OUTPUT);
  pinMode(coilPins[1], OUTPUT);
  pinMode(discreteInputPins[0], INPUT);
  pinMode(discreteInputPins[1], INPUT);

  modbus.configureCoils(coils, 2);
  modbus.configureDiscreteInputs(discreteInputs, 2);
  modbus.begin(1, 38400);
}

void loop() {
  discreteInputs[0] = digitalRead(discreteInputPins[0]);
  discreteInputs[1] = digitalRead(discreteInputPins[1]);

  modbus.poll();

  digitalWrite(coilPins[0], coils[0]);
  digitalWrite(coilPins[1], coils[1]);
}

```
