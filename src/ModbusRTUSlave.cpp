// Includes code derived from CMB27/ModbusRTUSlave (MIT).
// See THIRD_PARTY_NOTICES.md and LICENSES/CMB27-ModbusRTUSlave-MIT.txt.
#include "ModbusRTUSlave.h"
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif
#ifdef MBUS_RTU_SLAVE_EVENT_CALLBACKS
ModbusRTUSlave::EventFn ModbusRTUSlave::_eventFn = nullptr;

void ModbusRTUSlave::setEventFn(EventFn fn){
  _eventFn = fn;
}

void ModbusRTUSlave::noteEvent(uint16_t code, uint16_t units){
  if(_eventFn){
    _eventFn(code, units ? units : uint16_t(1U));
  }
}
#endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
static constexpr uint8_t kModbusExceptionDeviceFailure = 0x04;
ModbusRTUSlave::BridgeLocalRangeFn ModbusRTUSlave::_bridgeLocalRangeFn = nullptr;
ModbusRTUSlave::BridgeWriteAppliedFn ModbusRTUSlave::_bridgeWriteAppliedFn = nullptr;
ModbusRTUSlave::BridgeAdmissionFn ModbusRTUSlave::_bridgeAdmissionFn = nullptr;

void ModbusRTUSlave::setBridgeLocalRangeFn(BridgeLocalRangeFn fn){
  _bridgeLocalRangeFn = fn;
}

void ModbusRTUSlave::setBridgeWriteAppliedFn(BridgeWriteAppliedFn fn){
  _bridgeWriteAppliedFn = fn;
}

void ModbusRTUSlave::setBridgeAdmissionFn(BridgeAdmissionFn fn){
  _bridgeAdmissionFn = fn;
}

bool ModbusRTUSlave::bridgeIsLocalRange(uint16_t start, uint16_t count, bool isCoil) const{
  return _bridgeLocalRangeFn && _bridgeLocalRangeFn(start, count, isCoil);
}

void ModbusRTUSlave::bridgeNotifyWriteApplied(uint16_t start, uint16_t count,
                                              bool isCoil, bool isLocal) const{
  if(_bridgeWriteAppliedFn){
    _bridgeWriteAppliedFn(start, count, isCoil, isLocal);
  }
}
#endif

#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
namespace {

inline void bridgeUpstreamTxDiagUpdateMax(uint32_t& target, uint32_t value){
  if(value > target){
    target = value;
  }
}

inline uint32_t bridgeUpstreamTxDiagUsToMs(uint32_t deltaUs){
  return deltaUs / 1000UL;
}

} // namespace

uint8_t ModbusRTUSlave::bridgeUpstreamTxDiagFcIndex(uint8_t fc){
  switch(fc){
    case 5: return 0U;
    case 6: return 1U;
    case 15: return 2U;
    case 16: return 3U;
    default: return 0xFFU;
  }
}

uint8_t ModbusRTUSlave::bridgeUpstreamTxDiagQtyBucket(uint16_t quantity){
  if(quantity <= 1U){
    return 0U;
  }
  if(quantity <= 4U){
    return 1U;
  }
  if(quantity <= 8U){
    return 2U;
  }
  return 3U;
}

void ModbusRTUSlave::bridgeUpstreamTxDiagNoteThresholds(
    uint32_t valueMs,
    uint32_t (&thresholds)[kBridgeUpstreamTxThresholdCount],
    bool pumpPhase){
  if(pumpPhase){
    if(valueMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_2_MS){
      ++thresholds[0];
    }
    if(valueMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_5_MS){
      ++thresholds[1];
    }
    if(valueMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_10_MS){
      ++thresholds[2];
    }
    if(valueMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_20_MS){
      ++thresholds[3];
    }
    return;
  }

  if(valueMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_5_MS){
    ++thresholds[0];
  }
  if(valueMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_10_MS){
    ++thresholds[1];
  }
  if(valueMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_20_MS){
    ++thresholds[2];
  }
  if(valueMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_40_MS){
    ++thresholds[3];
  }
}

void ModbusRTUSlave::bridgeUpstreamTxDiagNoteAccepted(uint8_t fc, uint16_t quantity){
  const uint8_t fcIndex = bridgeUpstreamTxDiagFcIndex(fc);
  if(fcIndex >= kBridgeUpstreamTxFcCount){
    _bridgeUpstreamTxDiag.pendingActive = false;
    _bridgeUpstreamTxDiag.pendingFcIndex = 0xFFU;
    return;
  }
  const uint8_t qtyBucket = bridgeUpstreamTxDiagQtyBucket(quantity);
  ++_bridgeUpstreamTxDiag.accepted;
  ++_bridgeUpstreamTxDiag.bucketAccepted[fcIndex][qtyBucket];
  _bridgeUpstreamTxDiag.pendingActive = true;
  _bridgeUpstreamTxDiag.pendingFcIndex = fcIndex;
  _bridgeUpstreamTxDiag.pendingQtyBucket = qtyBucket;
}

void ModbusRTUSlave::bridgeUpstreamTxDiagNoteQueued(uint32_t queuedAtUs){
  if(!_bridgeUpstreamTxDiag.pendingActive){
    return;
  }
  ++_bridgeUpstreamTxDiag.queued;
  ++_bridgeUpstreamTxDiag.bucketQueued[_bridgeUpstreamTxDiag.pendingFcIndex]
                                      [_bridgeUpstreamTxDiag.pendingQtyBucket];
  _bridgeUpstreamTxDiag.queuedAtUs = queuedAtUs;
}

void ModbusRTUSlave::bridgeUpstreamTxDiagNotePumpSeen(uint32_t pumpSeenUs){
  if(!_bridgeUpstreamTxDiag.pendingActive){
    return;
  }
  ++_bridgeUpstreamTxDiag.txPumpSeen;
  const uint32_t pumpMs = bridgeUpstreamTxDiagUsToMs(pumpSeenUs - _bridgeUpstreamTxDiag.queuedAtUs);
  _bridgeUpstreamTxDiag.pumpMsSum += pumpMs;
  bridgeUpstreamTxDiagUpdateMax(_bridgeUpstreamTxDiag.pumpMsMax, pumpMs);
  bridgeUpstreamTxDiagNoteThresholds(pumpMs, _bridgeUpstreamTxDiag.pumpOverMs, true);
}

void ModbusRTUSlave::bridgeUpstreamTxDiagNoteTxDone(uint32_t doneUs){
  if(!_bridgeUpstreamTxDiag.pendingActive){
    return;
  }
  ++_bridgeUpstreamTxDiag.txDone;
  const uint32_t doneMs = bridgeUpstreamTxDiagUsToMs(doneUs - _bridgeUpstreamTxDiag.queuedAtUs);
  _bridgeUpstreamTxDiag.doneMsSum += doneMs;
  bridgeUpstreamTxDiagUpdateMax(_bridgeUpstreamTxDiag.doneMsMax, doneMs);
  bridgeUpstreamTxDiagNoteThresholds(doneMs, _bridgeUpstreamTxDiag.doneOverMs, false);
  if(doneMs >= MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_20_MS){
    ++_bridgeUpstreamTxDiag.bucketSlowDone[_bridgeUpstreamTxDiag.pendingFcIndex]
                                          [_bridgeUpstreamTxDiag.pendingQtyBucket];
  }
  bridgeUpstreamTxDiagUpdateMax(
      _bridgeUpstreamTxDiag.bucketDoneMaxMs[_bridgeUpstreamTxDiag.pendingFcIndex]
                                           [_bridgeUpstreamTxDiag.pendingQtyBucket],
      doneMs);
  _bridgeUpstreamTxDiag.pendingActive = false;
  _bridgeUpstreamTxDiag.pendingFcIndex = 0xFFU;
  _bridgeUpstreamTxDiag.pendingQtyBucket = 0U;
  _bridgeUpstreamTxDiag.queuedAtUs = 0U;
}

void ModbusRTUSlave::copyAndResetBridgeUpstreamTxDiag(BridgeUpstreamTxDiagSnapshot& out){
  out.accepted = _bridgeUpstreamTxDiag.accepted;
  out.queued = _bridgeUpstreamTxDiag.queued;
  out.txPumpSeen = _bridgeUpstreamTxDiag.txPumpSeen;
  out.txDone = _bridgeUpstreamTxDiag.txDone;
  out.pumpMsSum = _bridgeUpstreamTxDiag.pumpMsSum;
  out.pumpMsMax = _bridgeUpstreamTxDiag.pumpMsMax;
  out.doneMsSum = _bridgeUpstreamTxDiag.doneMsSum;
  out.doneMsMax = _bridgeUpstreamTxDiag.doneMsMax;
  for(uint8_t i = 0U; i < kBridgeUpstreamTxThresholdCount; ++i){
    out.pumpOverMs[i] = _bridgeUpstreamTxDiag.pumpOverMs[i];
    out.doneOverMs[i] = _bridgeUpstreamTxDiag.doneOverMs[i];
    _bridgeUpstreamTxDiag.pumpOverMs[i] = 0U;
    _bridgeUpstreamTxDiag.doneOverMs[i] = 0U;
  }
  for(uint8_t fcIndex = 0U; fcIndex < kBridgeUpstreamTxFcCount; ++fcIndex){
    for(uint8_t bucket = 0U; bucket < kBridgeUpstreamTxQtyBucketCount; ++bucket){
      out.bucketAccepted[fcIndex][bucket] =
          _bridgeUpstreamTxDiag.bucketAccepted[fcIndex][bucket];
      out.bucketQueued[fcIndex][bucket] =
          _bridgeUpstreamTxDiag.bucketQueued[fcIndex][bucket];
      out.bucketSlowDone[fcIndex][bucket] =
          _bridgeUpstreamTxDiag.bucketSlowDone[fcIndex][bucket];
      out.bucketDoneMaxMs[fcIndex][bucket] =
          _bridgeUpstreamTxDiag.bucketDoneMaxMs[fcIndex][bucket];
      _bridgeUpstreamTxDiag.bucketAccepted[fcIndex][bucket] = 0U;
      _bridgeUpstreamTxDiag.bucketQueued[fcIndex][bucket] = 0U;
      _bridgeUpstreamTxDiag.bucketSlowDone[fcIndex][bucket] = 0U;
      _bridgeUpstreamTxDiag.bucketDoneMaxMs[fcIndex][bucket] = 0U;
    }
  }
  _bridgeUpstreamTxDiag.accepted = 0U;
  _bridgeUpstreamTxDiag.queued = 0U;
  _bridgeUpstreamTxDiag.txPumpSeen = 0U;
  _bridgeUpstreamTxDiag.txDone = 0U;
  _bridgeUpstreamTxDiag.pumpMsSum = 0U;
  _bridgeUpstreamTxDiag.pumpMsMax = 0U;
  _bridgeUpstreamTxDiag.doneMsSum = 0U;
  _bridgeUpstreamTxDiag.doneMsMax = 0U;
}
#endif

namespace {

bool isFixedDiagnosticSubfunction(uint16_t subfunction){
  return (subfunction >= 1U && subfunction <= 4U) ||
         (subfunction >= 10U && subfunction <= 18U) ||
         subfunction == 20U;
}

bool isReadDeviceIdResponseCandidate(const uint8_t* rtu,
                                     uint16_t receivedLen){
  if(receivedLen < 10U || rtu[2] != 14U){
    return false;
  }

  uint16_t offset = 8U;
  const uint8_t objectCount = rtu[7];
  for(uint16_t object = 0U; object < objectCount; ++object){
    if(offset + 2U > receivedLen){
      return false;
    }
    const uint8_t objectLength = rtu[offset + 1U];
    offset = static_cast<uint16_t>(offset + 2U);
    if(offset + objectLength > receivedLen){
      return false;
    }
    offset = static_cast<uint16_t>(offset + objectLength);
  }
  return offset + 2U == receivedLen;
}

uint16_t boundedCandidateLength(uint32_t length){
  return length <= 256U ? static_cast<uint16_t>(length) : 0U;
}

uint16_t expectedStandardRequestLength(const uint8_t* rtu,
                                       uint16_t receivedLen){
  if(receivedLen < 2U){
    return 0U;
  }

  switch(rtu[1]){
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
      return 8U;
    case 7:
    case 11:
    case 12:
    case 17:
      return 4U;
    case 8: {
      if(receivedLen < 4U){
        return 0U;
      }
      const uint16_t subfunction =
          (static_cast<uint16_t>(rtu[2]) << 8) | rtu[3];
      // Return Query Data (subfunction 0) and reserved diagnostics have no
      // deterministic standard length, so T3.5 or the custom hook frames them.
      return isFixedDiagnosticSubfunction(subfunction) ? 8U : 0U;
    }
    case 15:
    case 16:
      return receivedLen >= 7U
                 ? boundedCandidateLength(9U + rtu[6])
                 : 0U;
    case 20:
    case 21:
      return receivedLen >= 3U
                 ? boundedCandidateLength(5U + rtu[2])
                 : 0U;
    case 22:
      return 10U;
    case 23:
      return receivedLen >= 11U
                 ? boundedCandidateLength(13U + rtu[10])
                 : 0U;
    case 24:
      return 6U;
    case 43:
      return receivedLen >= 3U && rtu[2] == 14U ? 7U : 0U;
    case 69:
      if(receivedLen < 4U){
        return 0U;
      }
      switch(rtu[3]){
        case 5:
        case 6:
          return 10U;
        case 15:
        case 16:
          return receivedLen >= 9U
                     ? boundedCandidateLength(11U + rtu[8])
                     : 0U;
        default:
          return 0U;
      }
    default:
      return 0U;
  }
}

uint16_t expectedStandardResponseLength(const uint8_t* rtu,
                                        uint16_t receivedLen){
  if(receivedLen < 2U){
    return 0U;
  }
  if((rtu[1] & 0x80U) != 0U){
    return 5U;
  }

  switch(rtu[1]){
    case 1:
    case 2:
    case 3:
    case 4:
    case 12:
    case 17:
    case 20:
    case 21:
    case 23:
      return receivedLen >= 3U
                 ? boundedCandidateLength(5U + rtu[2])
                 : 0U;
    case 5:
    case 6:
    case 15:
    case 16:
      return 8U;
    case 7:
      return 5U;
    case 8: {
      if(receivedLen < 4U){
        return 0U;
      }
      const uint16_t subfunction =
          (static_cast<uint16_t>(rtu[2]) << 8) | rtu[3];
      return isFixedDiagnosticSubfunction(subfunction) ? 8U : 0U;
    }
    case 11:
      return 8U;
    case 22:
      return 10U;
    case 24:
      return receivedLen >= 4U
                 ? boundedCandidateLength(
                       6U + (static_cast<uint16_t>(rtu[2]) << 8) + rtu[3])
                 : 0U;
    case 43:
      return isReadDeviceIdResponseCandidate(rtu, receivedLen)
                 ? receivedLen
                 : 0U;
    default:
      return 0U;
  }
}

} // namespace

// returns false if malformed length for inner FC
static inline bool fc69_len_ok(const uint8_t *b, uint16_t len){
  /// b[0]=0x00 addr, b[1]=0x45, b[2]=TID, b[3]=IFC, b[4..]=inner PDU, CRC is last 2 bytes
  if (len < 8) return false;                 // smallest possible: FC05/06
  const uint8_t ifc = b[3];
  const uint8_t *p  = &b[4];
  uint16_t need;                              // expected total length incl CRC

  switch (ifc) {
    case 0x05: // Write Single Coil: addr(2)+val(2)
    case 0x06: // Write Single Reg : addr(2)+val(2)
      need = 4 /*outer hdr incl IFC*/ + 4 /*inner*/ + 2 /*CRC*/; // = 10
      return len == need;

    case 0x0F: { // Write Multiple Coils
      if (len < 4+5+2) return false;         // header + addr(2)+qty(2)+bc(1) + CRC(2)
      uint16_t qty = (uint16_t(p[2])<<8) | p[3];
      if (qty == 0 || qty > 1952) return false;
      uint8_t bc = p[4];
      uint16_t expect_bc = (qty + 7) >> 3;
      if (bc != expect_bc) return false;
      need = 4 + 5 + bc + 2;
      return len == need;
    }

    case 0x10: { // Write Multiple Registers
      if (len < 4+5+2) return false;
      uint16_t qty = (uint16_t(p[2])<<8) | p[3];
      if (qty == 0 || qty > 122) return false;
      uint8_t bc = p[4];
      uint16_t expect_bc = qty << 1;
      if (bc != expect_bc) return false;
      need = 4 + 5 + bc + 2;
      return len == need;
    }

    default: return false;                   // only write-type IFCs allowed
  }
}

/*
 * Change log — Targeted Broadcast support (FC 0x45)
 * Date: 2025-09-03
 *
 * Goals
 * - Compliant broadcast handling per Modbus: reads ignored, writes processed, no replies.
 * - Targeted broadcast write on RTU/ASCII using FC 69.
 *
 * Key additions
 * - Public helpers: isBroadcast(), shouldIgnoreBroadcastRead(uint8_t), processTargetedBroadcastFC69().
 * - poll(): ignore broadcast reads, handle FC 69, then normal dispatcher.
 * - _exceptionResponse(): suppress reply when addr == 0.
 * - FC 69 size limits: FC16 ≤ 122 regs, FC0F ≤ 1952 coils.
 *
 * Compatibility
 * - Unicast behavior unchanged.
 * - FC 69 sent unicast → ILLEGAL FUNCTION.
 * - No Modbus TCP changes.
 *
 * Master format for FC 69
 * [Addr=0x00][FC=0x45][TargetUnit][InnerFC∈{0x05,0x06,0x0F,0x10}][Inner-PDU][CRC]
 */

ModbusRTUSlave::ModbusRTUSlave(HardwareSerial& serial, uint8_t dePin) {
  _hardwareSerial = &serial;
  _serial = &serial;
  _dePin = dePin;
}

//#ifdef __AVR__
//ModbusRTUSlave::ModbusRTUSlave(SoftwareSerial& serial, uint8_t dePin) {
//  _softwareSerial = &serial;
//  _serial = &serial;
//  _dePin = dePin;
//}
//#endif

#ifdef HAVE_CDCSERIAL
ModbusRTUSlave::ModbusRTUSlave(Serial_& serial, uint8_t dePin) {
  _usbSerial = &serial;
  _serial = &serial;
  _dePin = dePin;
}
#endif

#ifdef MBUS_RTU_SLAVE_USE_MUTEX
void ModbusRTUSlave::configurePlatformMutex(PlatformMutex* coilMut,
                                PlatformMutex* diMut, 
                                PlatformMutex* irMut,
                                PlatformMutex* hrMut){
  _coilMut = coilMut;
  _diMut = diMut;
  _irMut = irMut;
  _hrMut = hrMut;
}
#endif

void ModbusRTUSlave::configureCoils(bool coils[], uint16_t numCoils) {
  _coils = coils;
  _numCoils = numCoils;
}

void ModbusRTUSlave::configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs) {
  _discreteInputs = discreteInputs;
  _numDiscreteInputs = numDiscreteInputs;
}

void ModbusRTUSlave::configureHoldingRegisters(uint16_t holdingRegisters[], uint16_t numHoldingRegisters) {
  _holdingRegisters = holdingRegisters;
  _numHoldingRegisters = numHoldingRegisters;
}

void ModbusRTUSlave::configureInputRegisters(uint16_t inputRegisters[], uint16_t numInputRegisters) {
  _inputRegisters = inputRegisters;
  _numInputRegisters = numInputRegisters;
}

void ModbusRTUSlave::setAdditionalFrameCandidateFn(
    FrameCandidateFn frameCandidate){
  _additionalFrameCandidate = frameCandidate;
}

#ifdef ESP32
void ModbusRTUSlave::begin(uint8_t id, unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert) {
  if (id >= 1 && id <= 247) _id = id;
  else _id = NO_ID;
  if (_hardwareSerial) {
    _calculateTimeouts(baud, config);
    _hardwareSerial->begin(baud, config, rxPin, txPin, invert);
  }
  #ifdef HAVE_CDCSERIAL
  else if (_usbSerial) {
    _calculateTimeouts(baud, config);
    _usbSerial->begin(baud, config);
    while (!_usbSerial);
  }
  #endif
  if (_dePin != NO_DE_PIN) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  _clearRxBuffer();
}
#else
void ModbusRTUSlave::begin(uint8_t id, unsigned long baud, uint32_t config) {
  if (id >= 1 && id <= 247) _id = id;
  if (_hardwareSerial) {
    _calculateTimeouts(baud, config);
#if defined(ARDUINO_GIGA) && defined(MBUS_RTU_SLAVE_BRIDGE_MODE)
    if (config == SERIAL_8N1) {
      // Bridge-on-GIGA workaround: force 1-arg begin(baud) for 8N1 so we avoid
      // the 2-arg custom buffered-serial config parsing path that can corrupt
      // upstream Modbus frame decoding during activation storms.
      _hardwareSerial->begin(baud);
    } else {
      _hardwareSerial->begin(baud, config);
    }
#else
    _hardwareSerial->begin(baud, config);
#endif
  }
//  #ifdef __AVR__
//  else if (_softwareSerial) {
//    _calculateTimeouts(baud, SERIAL_8N1);
//    _softwareSerial->begin(baud);
//  }
//  #endif
  #ifdef HAVE_CDCSERIAL
  else if (_usbSerial) {
    _calculateTimeouts(baud, config);
    _usbSerial->begin(baud, config);
    while (!_usbSerial);
  }
  #endif
  if (_dePin != NO_DE_PIN) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  _clearRxBuffer();
}
#endif

void ModbusRTUSlave::poll() {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
  const uint32_t now_us = micros();
  if (dbg_.last_poll_us != 0) {
    const uint32_t gap_ms = (now_us - dbg_.last_poll_us) / 1000u;
    dbg_.last_poll_gap_ms = gap_ms;
    if (gap_ms > dbg_.max_poll_gap_ms) dbg_.max_poll_gap_ms = gap_ms;
  }
  dbg_.last_poll_us = now_us;
#endif
  // if (!_serial->available()) return;
  if (!_readRequest()) return;

  // A conforming RTU master does not pipeline requests that require replies.
  // If a local unicast was recoverable only because later traffic was already
  // queued, its transaction may have timed out and RTU has no transaction ID
  // with which to match a late response. Drop this earlier request before any
  // register, bridge-journal, callback, or TX side effect; the trailing frame
  // remains queued for the next poll. Buffered broadcasts remain actionable
  // because they deliberately require no reply.
  if (_rxBufferedCandidate && _buf[0] != 0U && _buf[0] == _id) return;

  const bool br = (_buf[0] == 0);   // broadcast on RTU/ASCII
  const uint8_t fc = _buf[1];

  // Single-switch dispatcher with broadcast short-circuits to save branches
  switch (fc) {
    case 1:
      if (br) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
        ++dbg_.ignored_broadcast_reads;
        dbg_.last_ignored_id = _buf[0];
        dbg_.last_ignored_fc = _buf[1];
        dbg_.last_ignored_start = _bytesToWord(_buf[2], _buf[3]);
        dbg_.last_ignored_count = _bytesToWord(_buf[4], _buf[5]);
        ++dbg_.ignored;
#endif
        return;
      }
      _processReadCoils();
      break;
    case 2:
      if (br) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
        ++dbg_.ignored_broadcast_reads;
        dbg_.last_ignored_id = _buf[0];
        dbg_.last_ignored_fc = _buf[1];
        dbg_.last_ignored_start = _bytesToWord(_buf[2], _buf[3]);
        dbg_.last_ignored_count = _bytesToWord(_buf[4], _buf[5]);
        ++dbg_.ignored;
#endif
        return;
      }
      _processReadDiscreteInputs();
      break;
    case 3:
      if (br) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
        ++dbg_.ignored_broadcast_reads;
        dbg_.last_ignored_id = _buf[0];
        dbg_.last_ignored_fc = _buf[1];
        dbg_.last_ignored_start = _bytesToWord(_buf[2], _buf[3]);
        dbg_.last_ignored_count = _bytesToWord(_buf[4], _buf[5]);
        ++dbg_.ignored;
#endif
        return;
      }
      _processReadHoldingRegisters();
      break;
    case 4:
      if (br) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
        ++dbg_.ignored_broadcast_reads;
        dbg_.last_ignored_id = _buf[0];
        dbg_.last_ignored_fc = _buf[1];
        dbg_.last_ignored_start = _bytesToWord(_buf[2], _buf[3]);
        dbg_.last_ignored_count = _bytesToWord(_buf[4], _buf[5]);
        ++dbg_.ignored;
#endif
        return;
      }
      _processReadInputRegisters();
      break;

    case 5: _processWriteSingleCoil(); break;                 // _writeResponse() suppresses if br
    case 6: _processWriteSingleHoldingRegister(); break;      // _writeResponse() suppresses if br
    case 15: _processWriteMultipleCoils(); break;             // _writeResponse() suppresses if br
    case 16: _processWriteMultipleHoldingRegisters(); break;  // _writeResponse() suppresses if br

    case 69: { // Targeted Broadcast Write (FC 0x45)
              //processing handled inline to avoid manually stripping the buf
      if (!br) { _exceptionResponse(1); break; } // unicast FC69 is illegal
      const uint8_t target  = _buf[2];
      const uint8_t innerFc = _buf[3];
      if (target != _id) return; // not for this unit

      if (!fc69_len_ok(_buf, _rxLen)) return;

      const uint8_t base = 4; // start of inner PDU
      switch (innerFc) {
        case 5: { // Write Single Coil
          uint16_t address = _bytesToWord(_buf[base+0], _buf[base+1]);
          uint16_t value   = _bytesToWord(_buf[base+2], _buf[base+3]);
          if (!_coils || _numCoils == 0) return;
          if (value != 0 && value != 0xFF00) return;
          if (address >= _numCoils) return;
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          const bool isLocal = bridgeIsLocalRange(address, 1, true);
          uint16_t bridgeContext = 0U;
          BridgeIngressReservation bridgeReservation;
          if(!bridgeWriteAllowed(address, 1, true, true, bridgeContext)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(!isLocal && !bridgeReserveCoilIngress(
              address, 1, true, bridgeContext, false,
              bridgeReservation)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
#endif
          #ifdef MBUS_RTU_SLAVE_USE_MUTEX
          if(_coilMut) _coilMut->lock();
          #endif
          _coils[address] = (value == 0xFF00);
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          bool bridgeQueued = isLocal ||
              bridgeCommitCoilIngress(bridgeReservation);
#endif
          #ifdef MBUS_RTU_SLAVE_USE_MUTEX
          if(_coilMut) _coilMut->unlock();
          #endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          if(!bridgeQueued){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          bridgeNotifyWriteApplied(address, 1, true, isLocal);
#endif
          return;
        }
        case 6: { // Write Single Register
          uint16_t address = _bytesToWord(_buf[base+0], _buf[base+1]);
          uint16_t value   = _bytesToWord(_buf[base+2], _buf[base+3]);
          if (!_holdingRegisters || _numHoldingRegisters == 0) return;
          if (address >= _numHoldingRegisters) return;
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          const bool isLocal = bridgeIsLocalRange(address, 1, false);
          uint16_t bridgeContext = 0U;
          BridgeIngressReservation bridgeReservation;
          if(!bridgeWriteAllowed(address, 1, false, true, bridgeContext)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(!isLocal && !bridgeReserveHoldingIngress(
              address, 1, true, bridgeContext, false,
              bridgeReservation)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
#endif
          #ifdef MBUS_RTU_SLAVE_USE_MUTEX
          if(_hrMut) _hrMut->lock();
          #endif
          _holdingRegisters[address] = value;
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          bool bridgeQueued = isLocal ||
              bridgeCommitHoldingIngress(bridgeReservation);
#endif
          #ifdef MBUS_RTU_SLAVE_USE_MUTEX
          if(_hrMut) _hrMut->unlock();
          #endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          if(!bridgeQueued){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          bridgeNotifyWriteApplied(address, 1, false, isLocal);
#endif
          return;
        }
        case 15: { // Write Multiple Coils
          uint16_t start    = _bytesToWord(_buf[base+0], _buf[base+1]);
          uint16_t quantity = _bytesToWord(_buf[base+2], _buf[base+3]);
          uint8_t  bc       = _buf[base+4];
          if (!_coils || _numCoils == 0) return;
          if (quantity == 0 || quantity > 1952) return;           // reduced max for FC69
          if (bc != _div8RndUp(quantity)) return;
          if (quantity > _numCoils || start > (_numCoils - quantity)) return;
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          const bool isLocal = bridgeIsLocalRange(start, quantity, true);
          uint16_t bridgeContext = 0U;
          BridgeIngressReservation bridgeReservation;
          if(!bridgeWriteAllowed(start, quantity, true, true, bridgeContext)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(!isLocal && !bridgeReserveCoilIngress(
              start, quantity, true, bridgeContext, false,
              bridgeReservation)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
#endif
          const uint8_t* data = &_buf[base+5];
          #ifdef MBUS_RTU_SLAVE_USE_MUTEX
          if(_coilMut) _coilMut->lock();
          #endif
          for (uint16_t i = 0; i < quantity; ++i) {
            _coils[start + i] = bitRead(data[i >> 3], i & 7);
          }
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          bool bridgeQueued = isLocal ||
              bridgeCommitCoilIngress(bridgeReservation);
#endif
          #ifdef MBUS_RTU_SLAVE_USE_MUTEX
          if(_coilMut) _coilMut->unlock();
          #endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          if(!bridgeQueued){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          bridgeNotifyWriteApplied(start, quantity, true, isLocal);
#endif
          return;
        }
        case 16: { // Write Multiple Registers
          uint16_t start    = _bytesToWord(_buf[base+0], _buf[base+1]);
          uint16_t quantity = _bytesToWord(_buf[base+2], _buf[base+3]);
          uint8_t  bc       = _buf[base+4];
          if (!_holdingRegisters || _numHoldingRegisters == 0) return;
          if (quantity == 0 || quantity > 122) return;            // reduced max for FC69
          if (bc != (quantity * 2)) return;
          if (quantity > _numHoldingRegisters || start > (_numHoldingRegisters - quantity)) return;
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          const bool isLocal = bridgeIsLocalRange(start, quantity, false);
          uint16_t bridgeContext = 0U;
          BridgeIngressReservation bridgeReservation;
          if(!bridgeWriteAllowed(start, quantity, false, true, bridgeContext)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(!isLocal && !bridgeReserveHoldingIngress(
              start, quantity, true, bridgeContext, false,
              bridgeReservation)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
#endif
          const uint8_t* data = &_buf[base+5];
          #ifdef MBUS_RTU_SLAVE_USE_MUTEX
          if(_hrMut) _hrMut->lock();
          #endif
          for (uint16_t i = 0; i < quantity; ++i) {
            _holdingRegisters[start + i] = _bytesToWord(data[i * 2], data[i * 2 + 1]);
          }
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          bool bridgeQueued = isLocal ||
              bridgeCommitHoldingIngress(bridgeReservation);
#endif
          #ifdef MBUS_RTU_SLAVE_USE_MUTEX
          if(_hrMut) _hrMut->unlock();
          #endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
          if(!bridgeQueued){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          bridgeNotifyWriteApplied(start, quantity, false, isLocal);
#endif
          return;
        }
        default:
          return; // invalid inner FC
      }
    }
      break;

    default:
      if (br) return;        // ignore unknown broadcast
      _exceptionResponse(1); // unicast: ILLEGAL FUNCTION
      break;
    }
}

void ModbusRTUSlave::_processReadCoils() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity     = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 2000) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(3);
  }
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity)) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(2);
  }
  else {
    uint8_t byteCount = _div8RndUp(quantity);
    _buf[2] = byteCount;
    // Zero the reply bytes once, then set bits. Prevents stale bits in last byte per spec.
    memset(&_buf[3], 0, byteCount);
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_coilMut) _coilMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; ++i) {
      if (_coils[startAddress + i]) {
        _buf[3 + (i >> 3)] |= (uint8_t)(1U << (i & 7));
      }
    }
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_coilMut) _coilMut->unlock();
    #endif
    _writeResponse(3 + byteCount);
  }
}

void ModbusRTUSlave::_processReadDiscreteInputs() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity     = _bytesToWord(_buf[4], _buf[5]);
  if (!_discreteInputs || _numDiscreteInputs == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 2000) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(3);
  }
  else if (quantity > _numDiscreteInputs || startAddress > (_numDiscreteInputs - quantity)) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(2);
  }
  else {
    uint8_t byteCount = _div8RndUp(quantity);
    _buf[2] = byteCount;
    // Zero the reply bytes once, then set bits.
    memset(&_buf[3], 0, byteCount);
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_diMut) _diMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; ++i) {
      if (_discreteInputs[startAddress + i]) {
        _buf[3 + (i >> 3)] |= (uint8_t)(1U << (i & 7));
      }
    }
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_diMut) _diMut->unlock();
    #endif
    _writeResponse(3 + byteCount);
  }
}

void ModbusRTUSlave::_processReadHoldingRegisters() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 125) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(3);
  }
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity)) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(2);
  }
  else {
    _buf[2] = quantity * 2;
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_hrMut) _hrMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; i++) {
      _buf[3 + (i * 2)] = highByte(_holdingRegisters[startAddress + i]);
      _buf[4 + (i * 2)] = lowByte(_holdingRegisters[startAddress + i]);
    }
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_hrMut) _hrMut->unlock();
    #endif
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processReadInputRegisters() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_inputRegisters || _numInputRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 125) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(3);
  }
  else if (quantity > _numInputRegisters || startAddress > (_numInputRegisters - quantity)) {
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(2);
  }
  else {
    _buf[2] = quantity * 2;
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_irMut) _irMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; i++) {
      _buf[3 + (i * 2)] = highByte(_inputRegisters[startAddress + i]);
      _buf[4 + (i * 2)] = lowByte(_inputRegisters[startAddress + i]);
    }
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_irMut) _irMut->unlock();
    #endif
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processWriteSingleCoil() {
  uint16_t address = _bytesToWord(_buf[2], _buf[3]);
  uint16_t value = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils ||_numCoils == 0) _exceptionResponse(1);
  else if (value != 0 && value != 0xFF00) _exceptionResponse(3);
  else if (address >= _numCoils) _exceptionResponse(2);
  else {
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    const bool isLocal = bridgeIsLocalRange(address, 1, true);
    uint16_t bridgeContext = 0U;
    BridgeIngressReservation bridgeReservation;
    if(!bridgeWriteAllowed(address, 1, true, false, bridgeContext)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(!isLocal && !bridgeReserveCoilIngress(
        address, 1, false, bridgeContext, _buf[0] != 0U,
        bridgeReservation)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
#endif
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_coilMut) _coilMut->lock();
    #endif
    _coils[address] = (value == 0xFF00);
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    const bool bridgeQueued = isLocal ||
        bridgeCommitCoilIngress(bridgeReservation);
#endif
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_coilMut) _coilMut->unlock();
    #endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    if(!bridgeQueued){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    bridgeNotifyWriteApplied(address, 1, true, isLocal);
#endif
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
    bridgeUpstreamTxDiagNoteAccepted(5U, 1U);
#endif
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteSingleHoldingRegister() {
  uint16_t address = _bytesToWord(_buf[2], _buf[3]);
  uint16_t value = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _exceptionResponse(1);
  else if (address >= _numHoldingRegisters) _exceptionResponse(2);
  else {
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    const bool isLocal = bridgeIsLocalRange(address, 1, false);
    uint16_t bridgeContext = 0U;
    BridgeIngressReservation bridgeReservation;
    if(!bridgeWriteAllowed(address, 1, false, false, bridgeContext)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(!isLocal && !bridgeReserveHoldingIngress(
        address, 1, false, bridgeContext, _buf[0] != 0U,
        bridgeReservation)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
#endif
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_hrMut) _hrMut->lock();
    #endif
    _holdingRegisters[address] = value;
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    const bool bridgeQueued = isLocal ||
        bridgeCommitHoldingIngress(bridgeReservation);
#endif
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_hrMut) _hrMut->unlock();
    #endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    if(!bridgeQueued){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    bridgeNotifyWriteApplied(address, 1, false, isLocal);
#endif
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
    bridgeUpstreamTxDiagNoteAccepted(6U, 1U);
#endif
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteMultipleCoils() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 1968 || _buf[6] != _div8RndUp(quantity)) _exceptionResponse(3);
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity)) _exceptionResponse(2);
  else {
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    const bool isLocal = bridgeIsLocalRange(startAddress, quantity, true);
    uint16_t bridgeContext = 0U;
    BridgeIngressReservation bridgeReservation;
    if(!bridgeWriteAllowed(startAddress, quantity, true, false, bridgeContext)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(!isLocal && !bridgeReserveCoilIngress(
        startAddress, quantity, false, bridgeContext,
        _buf[0] != 0U, bridgeReservation)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
#endif
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_coilMut) _coilMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; i++) {
      _coils[startAddress + i] = bitRead(_buf[7 + (i >> 3)], i & 7);
    }
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    const bool bridgeQueued = isLocal ||
        bridgeCommitCoilIngress(bridgeReservation);
#endif
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_coilMut) _coilMut->unlock();
    #endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    if(!bridgeQueued){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    bridgeNotifyWriteApplied(startAddress, quantity, true, isLocal);
#endif
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
    bridgeUpstreamTxDiagNoteAccepted(15U, quantity);
#endif
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteMultipleHoldingRegisters() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 123 || _buf[6] != (quantity * 2)) _exceptionResponse(3);
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity)) _exceptionResponse(2);
  else {
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    const bool isLocal = bridgeIsLocalRange(startAddress, quantity, false);
    uint16_t bridgeContext = 0U;
    BridgeIngressReservation bridgeReservation;
    if(!bridgeWriteAllowed(startAddress, quantity, false, false, bridgeContext)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(!isLocal && !bridgeReserveHoldingIngress(
        startAddress, quantity, false, bridgeContext, _buf[0] != 0U,
        bridgeReservation)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
#endif
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_hrMut) _hrMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; i++) {
      _holdingRegisters[startAddress + i] = _bytesToWord(_buf[i * 2 + 7], _buf[i * 2 + 8]);
    }
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    const bool bridgeQueued = isLocal ||
        bridgeCommitHoldingIngress(bridgeReservation);
#endif
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    if(_hrMut) _hrMut->unlock();
    #endif
#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    if(!bridgeQueued){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    bridgeNotifyWriteApplied(startAddress, quantity, false, isLocal);
#endif
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
    bridgeUpstreamTxDiagNoteAccepted(16U, quantity);
#endif
    _writeResponse(6);
  }
}

bool ModbusRTUSlave::_candidateCrcGood(uint16_t receivedLen) {
  if(receivedLen < 4U || receivedLen > 256U){
    return false;
  }
  return _crc(static_cast<uint8_t>(receivedLen - 2U)) ==
         _bytesToWord(_buf[receivedLen - 1U], _buf[receivedLen - 2U]);
}

bool ModbusRTUSlave::_finishRequest(bool bufferedCandidate) {
  const uint16_t frameLen = _rxNumBytes;
  _rxInFrame = false;
  _rxBufferedCandidate = bufferedCandidate;

  const bool addressed = (_buf[0] == _id || _buf[0] == 0U);
  if(frameLen >= 4U && addressed){
    const bool ok = _candidateCrcGood(frameLen);
    if(ok){
      _rxLen = frameLen;
    }
#ifdef MBUS_RTU_SLAVE_EVENT_CALLBACKS
    if(!ok){
      noteEvent(kEventCrcMismatch);
    }
#endif
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    if(frameLen > dbg_.max_rx_len){
      dbg_.max_rx_len = frameLen;
    }
#endif
    _rxNumBytes = 0U;
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    if(ok){
      dbg_.last_frame_us = micros();
      ++dbg_.frames_ok;
      dbg_.last_req_us = dbg_.last_frame_us;
      dbg_.last_req_id = _buf[0];
      dbg_.last_req_fc = _buf[1];
      dbg_.last_req_len = _rxLen;
    } else {
      // Capture the completed candidate length before resetting the parser;
      // the old ordering reported zero or a stale preceding request length.
      dbg_.last_bad_len = frameLen;
      dbg_.last_bad_id = _buf[0];
      dbg_.last_bad_fc = _buf[1];
      dbg_.last_bad_start = (uint16_t(_buf[2]) << 8) | _buf[3];
      ++dbg_.frames_bad;
    }
#endif
    return ok;
  }
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
  if(frameLen >= 4U && !addressed){
    ++dbg_.not_addressed;
    dbg_.last_not_addr_id = _buf[0];
  }
  if(frameLen > 0U && frameLen < 4U && addressed){
    ++dbg_.short_frames;
  }
#endif
#ifdef MBUS_RTU_SLAVE_EVENT_CALLBACKS
  if(frameLen != 0U && addressed){
    noteEvent(kEventMalformedFrame);
  }
#endif
  _rxNumBytes = 0U;
  return false;
}

void ModbusRTUSlave::_beginRxDiscard() {
  _rxDiscardUntilIdle = true;
  // Reuse the existing RX-active signal so cooperative schedulers continue
  // servicing the bounded discard state until the bus becomes idle.
  _rxInFrame = true;
  _rxBufferedCandidate = false;
  _rxNumBytes = 0U;
}

void ModbusRTUSlave::_drainRxDiscard() {
  uint16_t drained = 0U;
  // Never let a continuously noisy or broken bus monopolize poll(). Each pass
  // does no more receive work than the normal fixed-size parser buffer.
  while(drained < MODBUS_RTU_SLAVE_BUF_SIZE && _serial->available()){
    if(_serial->read() < 0){
      break;
    }
    ++drained;
    _rxLastByteUs = micros();
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    dbg_.last_byte_us = _rxLastByteUs;
#endif
  }
  if(_serial->available()){
    return;
  }
  if(static_cast<uint32_t>(micros() - _rxLastByteUs) >= _frameTimeout){
    _rxDiscardUntilIdle = false;
    _rxInFrame = false;
  }
}

bool ModbusRTUSlave::_readRequest() {
  int queued = _serial->available();
  if(queued <= 0 && !_rxInFrame){
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    // Preserve the existing diagnostics-path second availability sample.
    _serial->available();
#endif
    return false;
  }
  if(_rxDiscardUntilIdle){
    _drainRxDiscard();
    return false;
  }

  // Only foreign unit IDs can be peer responses. Keeping local and broadcast
  // traffic request-shaped avoids splitting a variable FC15/FC16 request at
  // an accidental CRC-valid eight-byte response prefix.
  bool allowResponseCandidate =
      _rxNumBytes > 0U && _buf[0] != 0U && _buf[0] != _id;
  uint16_t expectedRequestLen = 0U;
  uint16_t expectedResponseLen = 0U;
  if(_rxNumBytes >= 2U){
    expectedRequestLen =
        expectedStandardRequestLength(_buf, _rxNumBytes);
  }
  if(_rxNumBytes >= 2U && allowResponseCandidate){
    expectedResponseLen =
        expectedStandardResponseLength(_buf, _rxNumBytes);
  }

  // If a complete prefix was accumulated by an earlier poll and more bytes
  // are now queued, their historical arrival gap is no longer observable.
  // CRC plus a deterministic function shape is the narrow fallback boundary.
  if(_rxInFrame && queued > 0){
    bool candidate =
        (expectedRequestLen != 0U && _rxNumBytes == expectedRequestLen) ||
        (expectedResponseLen != 0U && _rxNumBytes == expectedResponseLen);
    if(!candidate && _rxNumBytes >= 4U && _additionalFrameCandidate){
      candidate = _additionalFrameCandidate(_buf, _rxNumBytes);
    }
    if(candidate && _candidateCrcGood(_rxNumBytes)){
      return _finishRequest(true);
    }
  }

  // Consume currently queued bytes without waiting. Once a standard length is
  // known, bulk-drain directly to that boundary; the normal path therefore
  // adds no candidate work to each payload byte. Unknown/custom shapes advance
  // one byte at a time so their next possible boundary can be reconsidered.
  while(queued > 0 && _rxNumBytes < MODBUS_RTU_SLAVE_BUF_SIZE){
    uint16_t readLimit;
    if(_additionalFrameCandidate || expectedRequestLen == 0U ||
       (allowResponseCandidate && expectedResponseLen == 0U)){
      readLimit = static_cast<uint16_t>(_rxNumBytes + 1U);
    } else {
      readLimit = MODBUS_RTU_SLAVE_BUF_SIZE;
      if(expectedRequestLen > _rxNumBytes){
        readLimit = expectedRequestLen;
      }
      if(allowResponseCandidate && expectedResponseLen > _rxNumBytes &&
         expectedResponseLen < readLimit){
        readLimit = expectedResponseLen;
      }
    }

    while(queued > 0 && _rxNumBytes < readLimit &&
          _rxNumBytes < MODBUS_RTU_SLAVE_BUF_SIZE){
      const int value = _serial->read();
      if(value < 0){
        queued = _serial->available();
        break;
      }
      if(!_rxInFrame){
        _rxInFrame = true;
        _rxNumBytes = 0U;
      }
      _buf[_rxNumBytes++] = static_cast<uint8_t>(value);
      _rxLastByteUs = micros();
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
      dbg_.last_byte_us = _rxLastByteUs;
#endif
      queued = _serial->available();
    }

    allowResponseCandidate =
        _rxNumBytes > 0U && _buf[0] != 0U && _buf[0] != _id;
    if(expectedRequestLen == 0U && _rxNumBytes >= 2U){
      expectedRequestLen =
          expectedStandardRequestLength(_buf, _rxNumBytes);
    }
    if(expectedResponseLen == 0U && _rxNumBytes >= 2U &&
       allowResponseCandidate){
      expectedResponseLen =
          expectedStandardResponseLength(_buf, _rxNumBytes);
    }
    if(queued > 0){
      bool candidate =
          (expectedRequestLen != 0U && _rxNumBytes == expectedRequestLen) ||
          (expectedResponseLen != 0U && _rxNumBytes == expectedResponseLen);
      if(!candidate && _rxNumBytes >= 4U && _additionalFrameCandidate){
        candidate = _additionalFrameCandidate(_buf, _rxNumBytes);
      }
      if(candidate && _candidateCrcGood(_rxNumBytes)){
        return _finishRequest(true);
      }
    }
  }

#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
  // Preserve the characterized diagnostics-path availability call count.
  const bool overflowQueued =
      _serial->available() && _rxNumBytes >= MODBUS_RTU_SLAVE_BUF_SIZE;
#else
  const bool overflowQueued =
      queued > 0 && _rxNumBytes >= MODBUS_RTU_SLAVE_BUF_SIZE;
#endif
  if(overflowQueued){
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    ++dbg_.rx_overflows;
#endif
    _beginRxDiscard();
    return false;
  }

  if(!_rxInFrame){
    return false;
  }

  // A software timestamp cannot prove an idle interval while unread bytes are
  // queued: a busy cooperative caller may simply not have consumed a valid
  // frame continuation yet. Only an empty queue makes the existing T3.5
  // completion rule authoritative.
  const uint32_t now = micros();
  if(static_cast<uint32_t>(now - _rxLastByteUs) < _frameTimeout){
    return false;
  }
  return _finishRequest(false);
}

void ModbusRTUSlave::_writeResponse(uint8_t len){
  if (_buf[0] == 0) return;                     // no reply on broadcast
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED || defined(MBUS_RTU_SLAVE_DIAGNOSTICS)
  const uint32_t queuedAtUs = micros();
#endif
  uint16_t crc = _crc(len);
  _buf[len] = lowByte(crc); _buf[len+1] = highByte(crc);
  if (_dePin != NO_DE_PIN) digitalWrite(_dePin, HIGH);
  _serial->write(_buf, len+2);
  _txBusy = true; _txLen = len+2;
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
  bridgeUpstreamTxDiagNoteQueued(queuedAtUs);
#endif
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
  ++dbg_.tx_count;
  const uint32_t now_us = queuedAtUs;
  dbg_.last_tx_us = now_us;
  dbg_.last_tx_id = _buf[0];
  dbg_.last_tx_fc = _buf[1];
  dbg_.last_tx_len = len + 2;
  if (dbg_.last_req_us != 0) {
    dbg_.last_resp_us = now_us - dbg_.last_req_us;
    if (dbg_.last_resp_us > dbg_.max_resp_us) dbg_.max_resp_us = dbg_.last_resp_us;
  }
#endif
}

void ModbusRTUSlave::tx_pump(){
  if (!_txBusy) { _txWasBusy = false; return; }

  if (!_txWasBusy) {
    _txWasBusy   = true;
    _txStartUs = micros();
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
    bridgeUpstreamTxDiagNotePumpSeen(_txStartUs);
#endif
    uint32_t charUs;
    #ifdef FLUSH_COMPENSATION_DELAY
      charUs = _flushCompensationDelay;          // ≈ 1 char @ current config
    #else
      charUs = (_charTimeout ? (_charTimeout / 3) : 100); // coarse 1-char fallback
    #endif
    _txDoneUs = _txStartUs + (uint32_t)_txLen * charUs;     // wait until bytes drain
    return;                                               // no blocking this tick
  }

  if ((int32_t)(micros() - _txDoneUs) < 0) return;         // not finished yet

  _serial->flush();                                       // should return fast now
  #ifdef FLUSH_COMPENSATION_DELAY
    delayMicroseconds(_flushCompensationDelay);
  #endif
  if (_dePin != NO_DE_PIN) digitalWrite(_dePin, LOW);
  _txBusy = false;
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED || defined(MBUS_RTU_SLAVE_DIAGNOSTICS)
  const uint32_t txDoneUs = micros();
#endif
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
  bridgeUpstreamTxDiagNoteTxDone(txDoneUs);
#endif
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
  dbg_.last_tx_done_us = txDoneUs;
  dbg_.last_tx_busy_us = dbg_.last_tx_done_us - _txStartUs;
  if (dbg_.last_tx_busy_us > dbg_.max_tx_busy_us) {
    dbg_.max_tx_busy_us = dbg_.last_tx_busy_us;
  }
#endif

  #ifdef MBUS_RTU_SLAVE_PURGE_RX_AFTER_TX
    while (_serial->available()) _serial->read();
  #endif
}

#ifdef MBUS_RTU_SLAVE_WORK_ACCESSORS
ModbusRTUSlave::WorkState ModbusRTUSlave::workState() const {
  WorkState s{};
  if (_rxInFrame) s.flags |= 0x01;
  if (_txBusy) s.flags |= 0x02;
  return s;
}

bool ModbusRTUSlave::hasWorkPending() const {
  const WorkState s = workState();
  const bool rxAvail = (_serial && _serial->available());
  return s.rxInFrame() || s.txBusy() || rxAvail;
}
#endif



void ModbusRTUSlave::_exceptionResponse(uint8_t code) {
  if (_buf[0] == 0) return; // Never reply to broadcast
  _buf[1] |= 0x80;
  _buf[2] = code;
  _writeResponse(3);
#ifdef MBUS_RTU_SLAVE_EVENT_CALLBACKS
  noteEvent(code);
#endif
}

void ModbusRTUSlave::_clearRxBuffer() {
  while (_serial->available()) _serial->read();
  _rxNumBytes = 0U;
  _rxInFrame = false;
  _rxBufferedCandidate = false;
  _rxDiscardUntilIdle = false;
}


void ModbusRTUSlave::_calculateTimeouts(unsigned long baud, uint32_t config) {
  unsigned long bitsPerChar;
  if (config == SERIAL_8E2 || config == SERIAL_8O2) bitsPerChar = 12;
  else if (config == SERIAL_8N2 || config == SERIAL_8E1 || config == SERIAL_8O1) bitsPerChar = 11;
  else bitsPerChar = 10;
  if (baud <= 19200) {
      _charTimeUs  = (bitsPerChar * 1000000UL + baud - 1) / baud;
      _charTimeout = (3 * _charTimeUs) / 2;       // T1.5
      _frameTimeout= (7 * _charTimeUs) / 2;       // T3.5
    } else {
      _charTimeUs  = (bitsPerChar * 1000000UL + baud - 1) / baud;
      _charTimeout = (3 * _charTimeUs) / 2;       // same math at high baud
      _frameTimeout= (7 * _charTimeUs) / 2;
    }
  #ifdef FLUSH_COMPENSATION_DELAY
  _flushCompensationDelay = _charTimeUs + 2;
  #endif
}

uint16_t ModbusRTUSlave::_crc(uint8_t len) {
  uint16_t value = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    value ^= (uint16_t)_buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      bool lsb = value & 1;
      value >>= 1;
      if (lsb) value ^= 0xA001;
    }
  }
  return value;
}

uint16_t ModbusRTUSlave::_div8RndUp(uint16_t value) {
  return (value + 7) >> 3;
}

uint16_t ModbusRTUSlave::_bytesToWord(uint8_t high, uint8_t low) {
  return (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
}

#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
// Firmware is otherwise built for size. The extracted journal adds fixed
// adapter work to every forwarded write, so keep only its producer/consumer
// seams at GCC's speed-optimised level. Native -Os A/B gates cover all three
// write shapes; unsupported compilers retain the same semantics at their
// ordinary optimisation level.
#if defined(__GNUC__) && !defined(__clang__)
#define MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT __attribute__((optimize("O2")))
#else
#define MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT
#endif

MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeReserveCoilIngress(
    uint16_t start, uint16_t count, bool ff,
    uint16_t context, bool responseRequired,
    BridgeIngressReservation& reservation){
  const uint8_t attributes = static_cast<uint8_t>(
      (ff ? kBridgeIngressFlagFireForget : 0U) |
      (responseRequired ? kBridgeIngressFlagResponseRequired : 0U));
  bool reserved = false;
  {
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
    LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
    reserved = _bridgeSourceJournal.reserve(
        BridgeIngressTable::Coils, start, count, context, 1U, attributes,
        reservation);
  }
  if(reserved) return true;
  return bridgeReserveIngressFailure(
      start, count, true, ff, context);
}

MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeReserveHoldingIngress(
    uint16_t start, uint16_t count, bool ff,
    uint16_t context, bool responseRequired,
    BridgeIngressReservation& reservation){
  const uint8_t attributes = static_cast<uint8_t>(
      (ff ? kBridgeIngressFlagFireForget : 0U) |
      (responseRequired ? kBridgeIngressFlagResponseRequired : 0U));
  bool reserved = false;
  {
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
    LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
    reserved = _bridgeSourceJournal.reserve(
        BridgeIngressTable::HoldingRegisters, start, count,
        context, 1U, attributes, reservation);
  }
  if(reserved) return true;
  return bridgeReserveIngressFailure(
      start, count, false, ff, context);
}

// Overflow diagnostics cannot run on a successful admission; keep this larger
// accounting path out of both table-specialized producer frames.
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
bool ModbusRTUSlave::bridgeReserveIngressFailure(
    uint16_t start, uint16_t count, bool isCoil, bool ff,
    uint16_t context){
  // Pre-response saturation is diagnostic only. The exception proves the
  // write was rejected, so this record must carry no public ACK/FAIL debt.
  const uint16_t units = bridgeUnitsForCount(count, isCoil);
  bridgeOverflowPush(isCoil, start, count, units, kBridgeDropReasonOverflow,
                     ff, context);
  return false;
}

MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeCommitCoilIngress(
    const BridgeIngressReservation& reservation){
  bool committed = false;
  {
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
    LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
    committed = _bridgeSourceJournal.publishCoils(
        reservation, &_coils[reservation.start()]);
  }
  if(committed) return true;
  return bridgeCommitIngressFailure(reservation, true);
}

MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeCommitHoldingIngress(
    const BridgeIngressReservation& reservation){
  bool committed = false;
  {
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
    LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
    committed = _bridgeSourceJournal.publishHolding(
        reservation, &_holdingRegisters[reservation.start()]);
  }
  if(committed) return true;
  return bridgeCommitIngressFailure(reservation, false);
}

// Keep contract-violation diagnostics off the acknowledged-write success path.
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
bool ModbusRTUSlave::bridgeCommitIngressFailure(
    const BridgeIngressReservation& reservation, bool isCoil){
  // A reservation can only become stale if the single-producer contract was
  // violated. Record the failure and never emit an ACK for a mutation that was
  // not durably journalled.
  const bool ff =
      (reservation.attributes() & kBridgeIngressFlagFireForget) != 0U;
  bridgeOverflowPush(isCoil, reservation.start(), reservation.count(),
                     bridgeUnitsForCount(reservation.count(), isCoil),
                     kBridgeDropReasonOverflow, ff,
                     reservation.context());
  return false;
}

bool ModbusRTUSlave::bridgeWriteAllowed(uint16_t start, uint16_t count, bool isCoil, bool ff,
                                        uint16_t& context){
  context = 0U;
  if(bridgeIsLocalRange(start, count, isCoil)) return true;
  if(_bridgeAdmissionFn){
    const bool admitted = _bridgeAdmissionFn(
        start, count, isCoil, ff, context);
    if(admitted){
      return true;
    }
    bridgeOverflowPush(isCoil, start, count, bridgeUnitsForCount(count, isCoil),
                       kBridgeDropReasonAdmissionRejected, ff, context);
    return false;
  }
  return true;
}

MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgePeekNext(
    BridgeIngressEntry& pending, BridgeIngressTable& table) const{
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  return _bridgeSourceJournal.peekNext(pending, table);
}

bool ModbusRTUSlave::bridgePeek(
    BridgeIngressTable table, BridgeIngressEntry& pending) const{
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  return _bridgeSourceJournal.peek(table, pending);
}

MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeCommit(
    BridgeIngressTable table, uint16_t sourceToken){
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  return _bridgeSourceJournal.retire(table, sourceToken);
}

bool ModbusRTUSlave::bridgePeekDrop(
    BridgeIngressTable table, BridgeIngressEntry& pending) const{
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  const bool coils = table == BridgeIngressTable::Coils;
  const uint8_t curHead = coils
      ? _bridgeOverflowCoilHead
      : _bridgeOverflowHrHead;
  const uint8_t curTail = coils
      ? _bridgeOverflowCoilTail
      : _bridgeOverflowHrTail;
  if(curHead == curTail) return false;
  const BridgeIngressEntry* queue = coils
      ? _bridgeOverflowCoilQ
      : _bridgeOverflowHrQ;
  memcpy(&pending, &queue[curHead], sizeof(pending));
  return pending.sourceToken != 0U;
}

bool ModbusRTUSlave::bridgeCommitDrop(
    BridgeIngressTable table, uint16_t sourceToken){
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  const bool coils = table == BridgeIngressTable::Coils;
  volatile uint8_t& head = coils
      ? _bridgeOverflowCoilHead
      : _bridgeOverflowHrHead;
  const volatile uint8_t& tail = coils
      ? _bridgeOverflowCoilTail
      : _bridgeOverflowHrTail;
  const BridgeIngressEntry* queue = coils
      ? _bridgeOverflowCoilQ
      : _bridgeOverflowHrQ;
  const uint8_t curHead = head;
  if(curHead == tail || sourceToken == 0U ||
     queue[curHead].sourceToken != sourceToken){
    return false;
  }
  head = static_cast<uint8_t>(
      (curHead + 1U) % kBridgeOverflowQueueSize);
  return true;
}

#undef MBUS_RTU_SLAVE_BRIDGE_SPEED_OPT
#endif

#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
uint16_t ModbusRTUSlave::bridgeUnitsForCount(uint16_t count, bool isCoil) const{
  // Each journal entry carries one snapshot chunk. The work-unit count tells a
  // consumer how many chunks the original Modbus write produced.
  constexpr uint16_t kMaxBridgeWriteCoils = 1968;
  constexpr uint16_t kMaxBridgeWriteHrs = 123;
  constexpr uint16_t kMaxCoilOps = static_cast<uint16_t>(
      (kMaxBridgeWriteCoils + (MBUS_RTU_SLAVE_BRIDGE_MAX_COILS - 1u)) /
      MBUS_RTU_SLAVE_BRIDGE_MAX_COILS);
  constexpr uint16_t kMaxHrOps = static_cast<uint16_t>(
      (kMaxBridgeWriteHrs +
       (MBUS_RTU_SLAVE_BRIDGE_MAX_HOLDING_REGISTERS - 1u)) /
      MBUS_RTU_SLAVE_BRIDGE_MAX_HOLDING_REGISTERS);
  static_assert(kMaxCoilOps <= 0xFFu,
                "coil work-unit count must fit BridgeIngressEntry::units");
  static_assert(kMaxHrOps <= 0xFFu,
                "holding work-unit count must fit BridgeIngressEntry::units");
  return BridgeIngressJournal::operationCount(
      isCoil ? BridgeIngressTable::Coils
             : BridgeIngressTable::HoldingRegisters,
      count);
}

void ModbusRTUSlave::bridgeOverflowPush(bool isCoil, uint16_t start, uint16_t count,
                                        uint16_t units, uint8_t reason, bool ff,
                                        uint16_t context){
  const uint16_t normalizedUnits = units ? units : uint16_t(1);
#ifdef MBUS_RTU_SLAVE_EVENT_CALLBACKS
  noteEvent(reason == kBridgeDropReasonAdmissionRejected
      ? kEventBridgeAdmissionRejected
      : kEventBridgeOverflow,
      normalizedUnits);
#endif
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  BridgeIngressEntry* q = isCoil ? _bridgeOverflowCoilQ : _bridgeOverflowHrQ;
  volatile uint8_t& head = isCoil ? _bridgeOverflowCoilHead : _bridgeOverflowHrHead;
  volatile uint8_t& tail = isCoil ? _bridgeOverflowCoilTail : _bridgeOverflowHrTail;
  const uint8_t size = kBridgeOverflowQueueSize;
  uint8_t curHead = head;
  const uint8_t curTail = tail;
  uint8_t nextTail = static_cast<uint8_t>((curTail + 1U) % size);
  if(nextTail == curHead){
    // Drop the oldest overflow entry to make room for the newest; newest
    // represents the current state we need to roll back.
    curHead = static_cast<uint8_t>((curHead + 1U) % size);
    nextTail = static_cast<uint8_t>((curTail + 1U) % size);
  }

  q[curTail].start = start;
  q[curTail].count = count;
  q[curTail].context = context;
  uint16_t token = static_cast<uint16_t>(_bridgeOverflowToken + 1U);
  if(token == 0U) token = 1U;
  _bridgeOverflowToken = token;
  q[curTail].sourceToken = token;
  q[curTail].units = static_cast<uint8_t>(normalizedUnits);
  q[curTail].attributes = static_cast<uint8_t>(
      (ff ? kBridgeIngressFlagFireForget : 0U) |
      (reason & kBridgeIngressReasonMask));
  q[curTail].snapshotCount = 0U;
  head = curHead;
  tail = nextTail;
}

#endif
