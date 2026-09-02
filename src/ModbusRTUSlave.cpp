#include "ModbusRTUSlave.h"

namespace {

uint16_t expectedRequestLength(const uint8_t* rtu, uint16_t receivedLen) {
  if (receivedLen < 2) return 0;
  switch (rtu[1]) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
      return 8U;
    case 15:
    case 16: {
      if (receivedLen < 7) return 0;
      const uint16_t expectedLen = 9U + rtu[6];
      return expectedLen <= 256U ? expectedLen : 0;
    }
    default:
      return 0;
  }
}

} // namespace

ModbusRTUSlave::ModbusRTUSlave(Stream& serial, int dePin, int rePin) : _rtuComm(serial, dePin, rePin) {
  
}

void ModbusRTUSlave::setResponseDelay(unsigned long responseDelay) {
  _responseDelay = responseDelay;
}

void ModbusRTUSlave::begin(uint8_t localUnitId, unsigned long baud, uint32_t config) {
  if (localUnitId >= 1 && localUnitId <= 247) _localUnitId = localUnitId;
  _rtuComm.begin(baud, config);
}

bool ModbusRTUSlave::poll() {
  ModbusADU adu;
  ModbusRTUCommError error = _rtuComm.readAdu(adu, expectedRequestLength);
  if (error) return false;
  uint8_t unitId = adu.getUnitId();
  if (unitId != _localUnitId && unitId != 0) return false;
  processPdu(adu);
  if (unitId != 0) {
    delay(_responseDelay);
    _rtuComm.writeAdu(adu);
  }
  return true;
}
