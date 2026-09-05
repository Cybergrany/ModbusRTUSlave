#include "ModbusRTUSlave.h"

ModbusRTUSlave::ModbusRTUSlave(Stream& serial, int dePin, int rePin) : _rtuComm(serial, dePin, rePin) {
  
}

void ModbusRTUSlave::setResponseDelay(unsigned long responseDelay) {
  _responseDelay = responseDelay;
}

void ModbusRTUSlave::setAdditionalFrameCandidateFn(
    ModbusRTUFrameCandidateFn additionalFrameCandidate) {
  _additionalFrameCandidate = additionalFrameCandidate;
}

void ModbusRTUSlave::begin(uint8_t localUnitId, unsigned long baud, uint32_t config) {
  if (localUnitId >= 1 && localUnitId <= 247) _localUnitId = localUnitId;
  _rtuComm.begin(baud, config);
}

bool ModbusRTUSlave::poll() {
  ModbusADU adu;
  bool usedBufferedCandidate = false;
  ModbusRTUCommError error =
      _rtuComm.readAdu(adu, _additionalFrameCandidate, usedBufferedCandidate);
  if (error) return false;
  uint8_t unitId = adu.getUnitId();
  if (unitId != _localUnitId && unitId != 0) return false;

  // A conforming RTU master does not pipeline reply-bearing requests. If this
  // local unicast was recovered only because later traffic was already queued,
  // its original transaction may already have timed out. RTU has no transaction
  // ID with which to match a late response, and writeAdu() could consume the
  // preserved next frame while checking local echo. Drop this stale candidate;
  // the next poll can still recover the trailing request.
  if (usedBufferedCandidate && unitId != 0) return false;

  processPdu(adu);
  if (unitId != 0) {
    delay(_responseDelay);
    _rtuComm.writeAdu(adu);
  }
  return true;
}
