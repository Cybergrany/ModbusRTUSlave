#include "ModbusRTUSlave.h"

namespace {

bool isCandidateLength(uint16_t receivedLen, uint32_t first,
                       uint32_t second = 0) {
  return receivedLen == first || (second != 0 && receivedLen == second);
}

bool isFixedDiagnosticSubfunction(uint16_t subfunction) {
  return (subfunction >= 1U && subfunction <= 4U) ||
         (subfunction >= 10U && subfunction <= 18U) || subfunction == 20U;
}

bool isReadDeviceIdCandidate(const uint8_t* rtu, uint16_t receivedLen) {
  // The request has four PDU bytes. A response has a fixed six-byte header
  // followed by a count-delimited list of ID/length/value objects.
  if (receivedLen == 7U) return true;
  if (receivedLen < 8U) return false;

  uint16_t offset = 8U;
  const uint8_t objectCount = rtu[7];
  for (uint16_t i = 0; i < objectCount; i++) {
    if (offset + 2U > receivedLen) return false;
    const uint8_t objectLength = rtu[offset + 1U];
    offset += 2U;
    if (offset + objectLength > receivedLen) return false;
    offset += objectLength;
  }
  return offset + 2U == receivedLen;
}

bool isStandardFrameCandidate(const uint8_t* rtu, uint16_t receivedLen) {
  if (receivedLen < 2U) return false;
  const uint8_t functionCode = rtu[1];

  // Every exception response has address, exception function, exception code,
  // and CRC. This also lets a delayed slave peel peer exception responses.
  if (functionCode & 0x80U) return receivedLen == 5U;

  uint32_t variableLength = 0;
  switch (rtu[1]) {
    case 1:
    case 2:
    case 3:
    case 4:
      if (receivedLen >= 3U) variableLength = 5U + rtu[2];
      return isCandidateLength(receivedLen, 8U, variableLength);
    case 5:
    case 6:
      return receivedLen == 8U;
    case 7:
      return isCandidateLength(receivedLen, 4U, 5U);
    case 8: {
      if (receivedLen < 4U) return false;
      const uint16_t subfunction =
          (static_cast<uint16_t>(rtu[2]) << 8) | rtu[3];
      // Return Query Data (subfunction 0) has an arbitrary even-sized payload;
      // reserved subfunctions are likewise left to T3.5 or the custom hook.
      return isFixedDiagnosticSubfunction(subfunction) && receivedLen == 8U;
    }
    case 11:
      return isCandidateLength(receivedLen, 4U, 8U);
    case 12:
      if (receivedLen >= 3U) variableLength = 5U + rtu[2];
      return isCandidateLength(receivedLen, 4U, variableLength);
    case 15:
    case 16: {
      if (receivedLen >= 7U) variableLength = 9U + rtu[6];
      return isCandidateLength(receivedLen, 8U, variableLength);
    }
    case 17:
      if (receivedLen >= 3U) variableLength = 5U + rtu[2];
      return isCandidateLength(receivedLen, 4U, variableLength);
    case 20:
    case 21:
      if (receivedLen >= 3U) variableLength = 5U + rtu[2];
      return receivedLen == variableLength;
    case 22:
      return receivedLen == 10U;
    case 23: {
      const uint32_t responseLength =
          receivedLen >= 3U ? 5U + rtu[2] : 0U;
      const uint32_t requestLength =
          receivedLen >= 11U ? 13U + rtu[10] : 0U;
      return isCandidateLength(receivedLen, responseLength, requestLength);
    }
    case 24:
      if (receivedLen >= 4U) {
        variableLength =
            6U + (static_cast<uint16_t>(rtu[2]) << 8) + rtu[3];
      }
      return isCandidateLength(receivedLen, 6U, variableLength);
    case 43:
      if (receivedLen < 3U || rtu[2] != 14U) return false;
      return isReadDeviceIdCandidate(rtu, receivedLen);
    default:
      // Reserved and user-defined functions have no standard length. T3.5 is
      // still sufficient for isolated frames; applications may register an
      // additional classifier to recover their shapes from a queued backlog.
      return false;
  }
}

} // namespace

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
      _rtuComm.readAdu(adu, isStandardFrameCandidate,
                       _additionalFrameCandidate, usedBufferedCandidate);
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
