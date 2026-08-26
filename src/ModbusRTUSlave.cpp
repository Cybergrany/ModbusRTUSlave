// Includes code derived from CMB27/ModbusRTUSlave (MIT).
// See THIRD_PARTY_NOTICES.md and LICENSES/CMB27-ModbusRTUSlave-MIT.txt.
#include "ModbusRTUSlave.h"
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif
#ifdef USING_STATS
#include "Pins/SlaveStats.h"
#endif

#ifdef OGM_BRIDGE_MODE
static constexpr uint8_t kModbusExceptionDeviceFailure = 0x04;
ModbusRTUSlave::BridgeLocalRangeFn ModbusRTUSlave::_bridgeLocalRangeFn = nullptr;
ModbusRTUSlave::BridgeLocalWriteFn ModbusRTUSlave::_bridgeLocalWriteFn = nullptr;
ModbusRTUSlave::BridgeAdmissionFn ModbusRTUSlave::_bridgeAdmissionFn = nullptr;

static_assert(sizeof(BridgeIngressEntry) == sizeof(BridgePending),
              "The OGM bridge adapter must not inflate pending records");

void ModbusRTUSlave::setBridgeLocalRangeFn(BridgeLocalRangeFn fn){
  _bridgeLocalRangeFn = fn;
}

void ModbusRTUSlave::setBridgeLocalWriteFn(BridgeLocalWriteFn fn){
  _bridgeLocalWriteFn = fn;
}

void ModbusRTUSlave::setBridgeAdmissionFn(BridgeAdmissionFn fn){
  _bridgeAdmissionFn = fn;
}

bool ModbusRTUSlave::bridgeIsLocalRange(uint16_t start, uint16_t count, bool isCoil) const{
  return _bridgeLocalRangeFn && _bridgeLocalRangeFn(start, count, isCoil);
}

bool ModbusRTUSlave::bridgeShouldNotifyLocalWrite(uint16_t start, uint16_t count, bool isCoil, bool isLocal) const{
  if(isLocal){
    return true;
  }
  // Bridge-active is the first bridge-local control coil, but it is allowed
  // through the normal write queue so activation ordering stays unchanged.
  return isCoil && count != 0 && start == 0;
}

void ModbusRTUSlave::bridgeNotifyLocalWrite(uint16_t start, uint16_t count, bool isCoil) const{
  if(_bridgeLocalWriteFn){
    _bridgeLocalWriteFn(start, count, isCoil);
  }
}
#endif

#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
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
    if(valueMs >= BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_2_MS){
      ++thresholds[0];
    }
    if(valueMs >= BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_5_MS){
      ++thresholds[1];
    }
    if(valueMs >= BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_10_MS){
      ++thresholds[2];
    }
    if(valueMs >= BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_20_MS){
      ++thresholds[3];
    }
    return;
  }

  if(valueMs >= BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_5_MS){
    ++thresholds[0];
  }
  if(valueMs >= BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_10_MS){
    ++thresholds[1];
  }
  if(valueMs >= BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_20_MS){
    ++thresholds[2];
  }
  if(valueMs >= BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_40_MS){
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
  if(doneMs >= BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_20_MS){
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

// returns false if malformed length for inner FC
static inline bool fc69_len_ok(const uint8_t *b, uint8_t len){
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

#ifdef OGM_USE_MUTEX
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
#if defined(ARDUINO_GIGA) && defined(OGM_BRIDGE_MODE)
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
#ifdef USB_DEBUG
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

  const bool br = (_buf[0] == 0);   // broadcast on RTU/ASCII
  const uint8_t fc = _buf[1];

  // Single-switch dispatcher with broadcast short-circuits to save branches
  switch (fc) {
    case 1:
      if (br) {
#ifdef USB_DEBUG
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
#ifdef USB_DEBUG
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
#ifdef USB_DEBUG
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
#ifdef USB_DEBUG
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
#ifdef OGM_BRIDGE_MODE
          const bool isLocal = bridgeIsLocalRange(address, 1, true);
          uint16_t bridgeSessionGeneration = 0U;
          BridgeIngressReservation bridgeReservation;
          if(!bridgeWriteAllowed(address, 1, true, true, bridgeSessionGeneration)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(!isLocal && !bridgeReserveCoilIngress(
              address, 1, true, bridgeSessionGeneration, false,
              bridgeReservation)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
#endif
          #ifdef OGM_USE_MUTEX
          if(_coilMut) _coilMut->lock();
          #endif
          _coils[address] = (value == 0xFF00);
#ifdef OGM_BRIDGE_MODE
          bool bridgeQueued = isLocal ||
              bridgeCommitCoilIngress(bridgeReservation);
#endif
          #ifdef OGM_USE_MUTEX
          if(_coilMut) _coilMut->unlock();
          #endif
#ifdef OGM_BRIDGE_MODE
          if(!bridgeQueued){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(bridgeShouldNotifyLocalWrite(address, 1, true, isLocal)){
            bridgeNotifyLocalWrite(address, 1, true);
          }
#endif
          return;
        }
        case 6: { // Write Single Register
          uint16_t address = _bytesToWord(_buf[base+0], _buf[base+1]);
          uint16_t value   = _bytesToWord(_buf[base+2], _buf[base+3]);
          if (!_holdingRegisters || _numHoldingRegisters == 0) return;
          if (address >= _numHoldingRegisters) return;
#ifdef OGM_BRIDGE_MODE
          const bool isLocal = bridgeIsLocalRange(address, 1, false);
          uint16_t bridgeSessionGeneration = 0U;
          BridgeIngressReservation bridgeReservation;
          if(!bridgeWriteAllowed(address, 1, false, true, bridgeSessionGeneration)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(!isLocal && !bridgeReserveHoldingIngress(
              address, 1, true, bridgeSessionGeneration, false,
              bridgeReservation)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
#endif
          #ifdef OGM_USE_MUTEX
          if(_hrMut) _hrMut->lock();
          #endif
          _holdingRegisters[address] = value;
#ifdef OGM_BRIDGE_MODE
          bool bridgeQueued = isLocal ||
              bridgeCommitHoldingIngress(bridgeReservation);
#endif
          #ifdef OGM_USE_MUTEX
          if(_hrMut) _hrMut->unlock();
          #endif
#ifdef OGM_BRIDGE_MODE
          if(!bridgeQueued){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(bridgeShouldNotifyLocalWrite(address, 1, false, isLocal)){
            bridgeNotifyLocalWrite(address, 1, false);
          }
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
#ifdef OGM_BRIDGE_MODE
          const bool isLocal = bridgeIsLocalRange(start, quantity, true);
          uint16_t bridgeSessionGeneration = 0U;
          BridgeIngressReservation bridgeReservation;
          if(!bridgeWriteAllowed(start, quantity, true, true, bridgeSessionGeneration)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(!isLocal && !bridgeReserveCoilIngress(
              start, quantity, true, bridgeSessionGeneration, false,
              bridgeReservation)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
#endif
          const uint8_t* data = &_buf[base+5];
          #ifdef OGM_USE_MUTEX
          if(_coilMut) _coilMut->lock();
          #endif
          for (uint16_t i = 0; i < quantity; ++i) {
            _coils[start + i] = bitRead(data[i >> 3], i & 7);
          }
#ifdef OGM_BRIDGE_MODE
          bool bridgeQueued = isLocal ||
              bridgeCommitCoilIngress(bridgeReservation);
#endif
          #ifdef OGM_USE_MUTEX
          if(_coilMut) _coilMut->unlock();
          #endif
#ifdef OGM_BRIDGE_MODE
          if(!bridgeQueued){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(bridgeShouldNotifyLocalWrite(start, quantity, true, isLocal)){
            bridgeNotifyLocalWrite(start, quantity, true);
          }
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
#ifdef OGM_BRIDGE_MODE
          const bool isLocal = bridgeIsLocalRange(start, quantity, false);
          uint16_t bridgeSessionGeneration = 0U;
          BridgeIngressReservation bridgeReservation;
          if(!bridgeWriteAllowed(start, quantity, false, true, bridgeSessionGeneration)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(!isLocal && !bridgeReserveHoldingIngress(
              start, quantity, true, bridgeSessionGeneration, false,
              bridgeReservation)){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
#endif
          const uint8_t* data = &_buf[base+5];
          #ifdef OGM_USE_MUTEX
          if(_hrMut) _hrMut->lock();
          #endif
          for (uint16_t i = 0; i < quantity; ++i) {
            _holdingRegisters[start + i] = _bytesToWord(data[i * 2], data[i * 2 + 1]);
          }
#ifdef OGM_BRIDGE_MODE
          bool bridgeQueued = isLocal ||
              bridgeCommitHoldingIngress(bridgeReservation);
#endif
          #ifdef OGM_USE_MUTEX
          if(_hrMut) _hrMut->unlock();
          #endif
#ifdef OGM_BRIDGE_MODE
          if(!bridgeQueued){
            _exceptionResponse(kModbusExceptionDeviceFailure);
            return;
          }
          if(bridgeShouldNotifyLocalWrite(start, quantity, false, isLocal)){
            bridgeNotifyLocalWrite(start, quantity, false);
          }
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
#ifdef USB_DEBUG
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(3);
  }
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity)) {
#ifdef USB_DEBUG
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
    #ifdef OGM_USE_MUTEX
    if(_coilMut) _coilMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; ++i) {
      if (_coils[startAddress + i]) {
        _buf[3 + (i >> 3)] |= (uint8_t)(1U << (i & 7));
      }
    }
    #ifdef OGM_USE_MUTEX
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
#ifdef USB_DEBUG
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(3);
  }
  else if (quantity > _numDiscreteInputs || startAddress > (_numDiscreteInputs - quantity)) {
#ifdef USB_DEBUG
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
    #ifdef OGM_USE_MUTEX
    if(_diMut) _diMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; ++i) {
      if (_discreteInputs[startAddress + i]) {
        _buf[3 + (i >> 3)] |= (uint8_t)(1U << (i & 7));
      }
    }
    #ifdef OGM_USE_MUTEX
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
#ifdef USB_DEBUG
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(3);
  }
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity)) {
#ifdef USB_DEBUG
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
    #ifdef OGM_USE_MUTEX
    if(_hrMut) _hrMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; i++) {
      _buf[3 + (i * 2)] = highByte(_holdingRegisters[startAddress + i]);
      _buf[4 + (i * 2)] = lowByte(_holdingRegisters[startAddress + i]);
    }
    #ifdef OGM_USE_MUTEX
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
#ifdef USB_DEBUG
    ++dbg_.ignored;
    dbg_.last_ignored_id = _buf[0];
    dbg_.last_ignored_fc = _buf[1];
    dbg_.last_ignored_start = startAddress;
    dbg_.last_ignored_count = quantity;
#endif
    _exceptionResponse(3);
  }
  else if (quantity > _numInputRegisters || startAddress > (_numInputRegisters - quantity)) {
#ifdef USB_DEBUG
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
    #ifdef OGM_USE_MUTEX
    if(_irMut) _irMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; i++) {
      _buf[3 + (i * 2)] = highByte(_inputRegisters[startAddress + i]);
      _buf[4 + (i * 2)] = lowByte(_inputRegisters[startAddress + i]);
    }
    #ifdef OGM_USE_MUTEX
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
#ifdef OGM_BRIDGE_MODE
    const bool isLocal = bridgeIsLocalRange(address, 1, true);
    uint16_t bridgeSessionGeneration = 0U;
    BridgeIngressReservation bridgeReservation;
    if(!bridgeWriteAllowed(address, 1, true, false, bridgeSessionGeneration)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(!isLocal && !bridgeReserveCoilIngress(
        address, 1, false, bridgeSessionGeneration, address != 0U,
        bridgeReservation)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
#endif
    #ifdef OGM_USE_MUTEX
    if(_coilMut) _coilMut->lock();
    #endif
    _coils[address] = (value == 0xFF00);
#ifdef OGM_BRIDGE_MODE
    const bool bridgeQueued = isLocal ||
        bridgeCommitCoilIngress(bridgeReservation);
#endif
    #ifdef OGM_USE_MUTEX
    if(_coilMut) _coilMut->unlock();
    #endif
#ifdef OGM_BRIDGE_MODE
    if(!bridgeQueued){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(bridgeShouldNotifyLocalWrite(address, 1, true, isLocal)){
      bridgeNotifyLocalWrite(address, 1, true);
    }
#endif
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
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
#ifdef OGM_BRIDGE_MODE
    const bool isLocal = bridgeIsLocalRange(address, 1, false);
    uint16_t bridgeSessionGeneration = 0U;
    BridgeIngressReservation bridgeReservation;
    if(!bridgeWriteAllowed(address, 1, false, false, bridgeSessionGeneration)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(!isLocal && !bridgeReserveHoldingIngress(
        address, 1, false, bridgeSessionGeneration, true,
        bridgeReservation)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
#endif
    #ifdef OGM_USE_MUTEX
    if(_hrMut) _hrMut->lock();
    #endif
    _holdingRegisters[address] = value;
#ifdef OGM_BRIDGE_MODE
    const bool bridgeQueued = isLocal ||
        bridgeCommitHoldingIngress(bridgeReservation);
#endif
    #ifdef OGM_USE_MUTEX
    if(_hrMut) _hrMut->unlock();
    #endif
#ifdef OGM_BRIDGE_MODE
    if(!bridgeQueued){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(bridgeShouldNotifyLocalWrite(address, 1, false, isLocal)){
      bridgeNotifyLocalWrite(address, 1, false);
    }
#endif
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
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
#ifdef OGM_BRIDGE_MODE
    const bool isLocal = bridgeIsLocalRange(startAddress, quantity, true);
    uint16_t bridgeSessionGeneration = 0U;
    BridgeIngressReservation bridgeReservation;
    if(!bridgeWriteAllowed(startAddress, quantity, true, false, bridgeSessionGeneration)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(!isLocal && !bridgeReserveCoilIngress(
        startAddress, quantity, false, bridgeSessionGeneration,
        !(startAddress == 0U && quantity == 1U), bridgeReservation)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
#endif
    #ifdef OGM_USE_MUTEX
    if(_coilMut) _coilMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; i++) {
      _coils[startAddress + i] = bitRead(_buf[7 + (i >> 3)], i & 7);
    }
#ifdef OGM_BRIDGE_MODE
    const bool bridgeQueued = isLocal ||
        bridgeCommitCoilIngress(bridgeReservation);
#endif
    #ifdef OGM_USE_MUTEX
    if(_coilMut) _coilMut->unlock();
    #endif
#ifdef OGM_BRIDGE_MODE
    if(!bridgeQueued){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(bridgeShouldNotifyLocalWrite(startAddress, quantity, true, isLocal)){
      bridgeNotifyLocalWrite(startAddress, quantity, true);
    }
#endif
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
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
#ifdef OGM_BRIDGE_MODE
    const bool isLocal = bridgeIsLocalRange(startAddress, quantity, false);
    uint16_t bridgeSessionGeneration = 0U;
    BridgeIngressReservation bridgeReservation;
    if(!bridgeWriteAllowed(startAddress, quantity, false, false, bridgeSessionGeneration)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(!isLocal && !bridgeReserveHoldingIngress(
        startAddress, quantity, false, bridgeSessionGeneration, true,
        bridgeReservation)){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
#endif
    #ifdef OGM_USE_MUTEX
    if(_hrMut) _hrMut->lock();
    #endif
    for (uint16_t i = 0; i < quantity; i++) {
      _holdingRegisters[startAddress + i] = _bytesToWord(_buf[i * 2 + 7], _buf[i * 2 + 8]);
    }
#ifdef OGM_BRIDGE_MODE
    const bool bridgeQueued = isLocal ||
        bridgeCommitHoldingIngress(bridgeReservation);
#endif
    #ifdef OGM_USE_MUTEX
    if(_hrMut) _hrMut->unlock();
    #endif
#ifdef OGM_BRIDGE_MODE
    if(!bridgeQueued){
      _exceptionResponse(kModbusExceptionDeviceFailure);
      return;
    }
    if(bridgeShouldNotifyLocalWrite(startAddress, quantity, false, isLocal)){
      bridgeNotifyLocalWrite(startAddress, quantity, false);
    }
#endif
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
    bridgeUpstreamTxDiagNoteAccepted(16U, quantity);
#endif
    _writeResponse(6);
  }
}

bool ModbusRTUSlave::_readRequest() {
  // TODO(rx-framing): Stream exposes queued bytes, not their physical arrival times.
  // If poll() is delayed until two legal RTU frames are buffered, their real T3.5
  // gap is lost here; this drain concatenates them and the single CRC check below
  // rejects both. The edge is local to each slave's service cadence, so another
  // node may parse the same wire traffic correctly. Characterize it by queuing two
  // CRC-valid ADUs before one service pass (including FC69 followed by unicast) and
  // asserting that each is extracted without a spurious CRC failure. Mitigate by
  // peeling one function-length-derived ADU at a time while retaining trailing
  // bytes, or by carrying UART idle/per-byte timing into the parser; do not merely
  // suppress the CRC statistic.
  // ingest available bytes without waiting
  while (_serial->available() && _rxNumBytes < MODBUS_RTU_SLAVE_BUF_SIZE) {
    uint8_t b = _serial->read();
    if (!_rxInFrame) { _rxInFrame = true; _rxNumBytes = 0; }
    _buf[_rxNumBytes++] = b;
    _rxLastByteUs = micros();
#ifdef USB_DEBUG
    dbg_.last_byte_us = _rxLastByteUs;
#endif
  }
#ifdef USB_DEBUG
  if (_serial->available() && _rxNumBytes >= MODBUS_RTU_SLAVE_BUF_SIZE) {
    ++dbg_.rx_overflows;
  }
#endif

  if (!_rxInFrame) return false;

  // frame ends on ≥ t3.5 idle
  const uint32_t now = micros();
  if ((uint32_t)(now - _rxLastByteUs) < _frameTimeout) return false;

  // finalize
  _rxInFrame = false;
  const bool addressed = (_buf[0] == _id || _buf[0] == 0);
  if (_rxNumBytes >= 4 && addressed) {
    const bool ok = (_crc(_rxNumBytes - 2) ==
                     _bytesToWord(_buf[_rxNumBytes - 1], _buf[_rxNumBytes - 2]));
    if (ok) _rxLen = _rxNumBytes; 
#ifdef USB_DEBUG
    if (_rxNumBytes > dbg_.max_rx_len) dbg_.max_rx_len = _rxNumBytes;
#endif
    _rxNumBytes = 0;
#ifdef USB_DEBUG
    if (ok) {
      dbg_.last_frame_us = micros();
      ++dbg_.frames_ok;
      dbg_.last_req_us = dbg_.last_frame_us;
      dbg_.last_req_id = _buf[0];
      dbg_.last_req_fc = _buf[1];
      dbg_.last_req_len = _rxLen;
    } else {
      dbg_.last_bad_len = _rxLen ? _rxLen : _rxNumBytes;
      dbg_.last_bad_id = _buf[0];
      dbg_.last_bad_fc = _buf[1];
      dbg_.last_bad_start = (uint16_t(_buf[2]) << 8) | _buf[3];
      ++dbg_.frames_bad;
    }
#endif
#ifdef USING_STATS
    if(!ok){
      PinTypes_slave::SlaveStats::recordError(PinTypes_slave::SlaveStats::kErrCodeCrcMismatch);
    }
#endif
    return ok;
  }
#ifdef USB_DEBUG
  if (_rxNumBytes >= 4 && !addressed) {
    ++dbg_.not_addressed;
    dbg_.last_not_addr_id = _buf[0];
  }
#endif
#ifdef USING_STATS
  if(_rxNumBytes && addressed){
    PinTypes_slave::SlaveStats::recordError(PinTypes_slave::SlaveStats::kErrCodeBadFrame);
  }
#endif
#ifdef USB_DEBUG
  if (_rxNumBytes > 0 && _rxNumBytes < 4 && addressed) {
    ++dbg_.short_frames;
  }
#endif
  _rxNumBytes = 0;
  return false;
}

void ModbusRTUSlave::_writeResponse(uint8_t len){
  if (_buf[0] == 0) return;                     // no reply on broadcast
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED || defined(USB_DEBUG)
  const uint32_t queuedAtUs = micros();
#endif
  uint16_t crc = _crc(len);
  _buf[len] = lowByte(crc); _buf[len+1] = highByte(crc);
  if (_dePin != NO_DE_PIN) digitalWrite(_dePin, HIGH);
  _serial->write(_buf, len+2);
  _txBusy = true; _txLen = len+2;
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
  bridgeUpstreamTxDiagNoteQueued(queuedAtUs);
#endif
#ifdef USB_DEBUG
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
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
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
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED || defined(USB_DEBUG)
  const uint32_t txDoneUs = micros();
#endif
#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
  bridgeUpstreamTxDiagNoteTxDone(txDoneUs);
#endif
#ifdef USB_DEBUG
  dbg_.last_tx_done_us = txDoneUs;
  dbg_.last_tx_busy_us = dbg_.last_tx_done_us - _txStartUs;
  if (dbg_.last_tx_busy_us > dbg_.max_tx_busy_us) {
    dbg_.max_tx_busy_us = dbg_.last_tx_busy_us;
  }
#endif

  #ifdef MODBUS_PURGE_RX_AFTER_TX
    while (_serial->available()) _serial->read();
  #endif
}

#ifdef OGM_MODBUS_MT_ACCESSORS
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
#ifdef USING_STATS
  PinTypes_slave::SlaveStats::recordError(code);
#endif
}

void ModbusRTUSlave::_clearRxBuffer() {
  while (_serial->available()) _serial->read();
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

#ifdef OGM_BRIDGE_MODE
// Firmware is otherwise built for size. The extracted journal adds fixed
// adapter work to every forwarded write, so keep only its producer/consumer
// seams at GCC's speed-optimised level. Native -Os A/B gates cover all three
// write shapes; unsupported compilers retain the same semantics at their
// ordinary optimisation level.
#if defined(__GNUC__) && !defined(__clang__)
#define OGM_MODBUS_BRIDGE_SPEED_OPT __attribute__((optimize("O2")))
#else
#define OGM_MODBUS_BRIDGE_SPEED_OPT
#endif

OGM_MODBUS_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeReserveCoilIngress(
    uint16_t start, uint16_t count, bool ff,
    uint16_t sessionGeneration, bool publicDebt,
    BridgeIngressReservation& reservation){
  const uint8_t meta = static_cast<uint8_t>(
      (ff ? kBridgePendingFlagFireForget : 0U) |
      (publicDebt ? kBridgePendingFlagPublicDebt : 0U));
  bool reserved = false;
  {
#ifdef OGM_USE_MUTEX
    LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
    reserved = _bridgeSourceJournal.reserve(
        BridgeIngressTable::Coils, start, count, sessionGeneration, 1U, meta,
        reservation);
  }
  if(reserved) return true;
  return bridgeReserveIngressFailure(
      start, count, true, ff, sessionGeneration);
}

OGM_MODBUS_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeReserveHoldingIngress(
    uint16_t start, uint16_t count, bool ff,
    uint16_t sessionGeneration, bool publicDebt,
    BridgeIngressReservation& reservation){
  const uint8_t meta = static_cast<uint8_t>(
      (ff ? kBridgePendingFlagFireForget : 0U) |
      (publicDebt ? kBridgePendingFlagPublicDebt : 0U));
  bool reserved = false;
  {
#ifdef OGM_USE_MUTEX
    LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
    reserved = _bridgeSourceJournal.reserve(
        BridgeIngressTable::HoldingRegisters, start, count,
        sessionGeneration, 1U, meta, reservation);
  }
  if(reserved) return true;
  return bridgeReserveIngressFailure(
      start, count, false, ff, sessionGeneration);
}

// Overflow diagnostics cannot run on a successful admission; keep this larger
// accounting path out of both table-specialized producer frames.
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
bool ModbusRTUSlave::bridgeReserveIngressFailure(
    uint16_t start, uint16_t count, bool isCoil, bool ff,
    uint16_t sessionGeneration){
  // Pre-response saturation is diagnostic only. The exception proves the
  // write was rejected, so this record must carry no public ACK/FAIL debt.
  const uint16_t ops = bridgeOpsForCount(count, isCoil);
  bridgeOverflowPush(isCoil, start, count, ops, kBridgeDropReasonOverflow,
                     ff, sessionGeneration);
  return false;
}

OGM_MODBUS_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeCommitCoilIngress(
    const BridgeIngressReservation& reservation){
  bool committed = false;
  {
#ifdef OGM_USE_MUTEX
    LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
    committed = _bridgeSourceJournal.publishCoils(
        reservation, &_coils[reservation.start()]);
  }
  if(committed) return true;
  return bridgeCommitIngressFailure(reservation, true);
}

OGM_MODBUS_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeCommitHoldingIngress(
    const BridgeIngressReservation& reservation){
  bool committed = false;
  {
#ifdef OGM_USE_MUTEX
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
  // violated. Preserve the legacy failure accounting and never emit an ACK for
  // a mutation that was not durably journalled.
  const bool ff =
      (reservation.attributes() & kBridgePendingFlagFireForget) != 0U;
  bridgeOverflowPush(isCoil, reservation.start(), reservation.count(),
                     bridgeOpsForCount(reservation.count(), isCoil),
                     kBridgeDropReasonOverflow, ff,
                     reservation.context());
  return false;
}

bool ModbusRTUSlave::bridgeWriteAllowed(uint16_t start, uint16_t count, bool isCoil, bool ff,
                                        uint16_t& sessionGeneration){
  sessionGeneration = 0U;
  if(bridgeIsLocalRange(start, count, isCoil)) return true;
  if(_bridgeAdmissionFn){
    const bool admitted = _bridgeAdmissionFn(
        start, count, isCoil, ff, sessionGeneration);
    if(admitted){
      return true;
    }
    bridgeOverflowPush(isCoil, start, count, bridgeOpsForCount(count, isCoil),
                       kBridgeDropReasonInactive, ff,
                       sessionGeneration);
    return false;
  }
  // Bridge-active coil is always at position 0.
  if(!_coils || _numCoils == 0) return true;
  const bool active = _coils[0];
  if(active){
    sessionGeneration = 1U;
    return true;
  }
  if(isCoil && start == 0 && count == 1){
    sessionGeneration = 1U;
    return true; // allow toggling bridge_active while inactive
  }
  // Bridge is inactive: publish this range in the overflow/drop path with a
  // dedicated reason so runtime always applies fail/health accounting, even if
  // bridge_active flips before processing.
  bridgeOverflowPush(isCoil, start, count, bridgeOpsForCount(count, isCoil),
                     kBridgeDropReasonInactive, ff, sessionGeneration);
  return false;
}

bool ModbusRTUSlave::bridgePeekCoils(BridgePending& pending) const{
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  return _bridgeSourceJournal.peek(BridgeIngressTable::Coils, pending);
}

OGM_MODBUS_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgePeekNext(BridgePending& pending, bool& isCoil) const{
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  BridgeIngressTable table = BridgeIngressTable::Coils;
  if(!_bridgeSourceJournal.peekNext(pending, table)) return false;
  isCoil = table == BridgeIngressTable::Coils;
  return true;
}

OGM_MODBUS_BRIDGE_SPEED_OPT
bool ModbusRTUSlave::bridgeCommitNext(bool isCoil, uint16_t sourceToken){
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  return _bridgeSourceJournal.retire(
      isCoil ? BridgeIngressTable::Coils
             : BridgeIngressTable::HoldingRegisters,
      sourceToken);
}

bool ModbusRTUSlave::bridgeCommitCoils(uint16_t sourceToken){
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  return _bridgeSourceJournal.retire(BridgeIngressTable::Coils, sourceToken);
}

bool ModbusRTUSlave::bridgePeekHolding(BridgePending& pending) const{
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  return _bridgeSourceJournal.peek(
      BridgeIngressTable::HoldingRegisters, pending);
}

bool ModbusRTUSlave::bridgeCommitHolding(uint16_t sourceToken){
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  return _bridgeSourceJournal.retire(
      BridgeIngressTable::HoldingRegisters, sourceToken);
}

bool ModbusRTUSlave::bridgePeekOverflowCoils(BridgePending& pending) const{
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  const uint8_t curHead = _bridgeOverflowCoilHead;
  if(curHead == _bridgeOverflowCoilTail) return false;
  memcpy(&pending, &_bridgeOverflowCoilQ[curHead], sizeof(pending));
  return pending.sourceToken != 0U;
}

bool ModbusRTUSlave::bridgeCommitOverflowCoils(uint16_t sourceToken){
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  const uint8_t curHead = _bridgeOverflowCoilHead;
  if(curHead == _bridgeOverflowCoilTail || sourceToken == 0U ||
     _bridgeOverflowCoilQ[curHead].sourceToken != sourceToken){
    return false;
  }
  _bridgeOverflowCoilHead = static_cast<uint8_t>(
      (curHead + 1U) % kBridgeOverflowQueueSize);
  return true;
}

bool ModbusRTUSlave::bridgePeekOverflowHolding(BridgePending& pending) const{
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  const uint8_t curHead = _bridgeOverflowHrHead;
  if(curHead == _bridgeOverflowHrTail) return false;
  memcpy(&pending, &_bridgeOverflowHrQ[curHead], sizeof(pending));
  return pending.sourceToken != 0U;
}

bool ModbusRTUSlave::bridgeCommitOverflowHolding(uint16_t sourceToken){
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  const uint8_t curHead = _bridgeOverflowHrHead;
  if(curHead == _bridgeOverflowHrTail || sourceToken == 0U ||
     _bridgeOverflowHrQ[curHead].sourceToken != sourceToken){
    return false;
  }
  _bridgeOverflowHrHead = static_cast<uint8_t>(
      (curHead + 1U) % kBridgeOverflowQueueSize);
  return true;
}

#undef OGM_MODBUS_BRIDGE_SPEED_OPT
#endif

/**
 * TODO: expose for all writable slave types, so this can be polled instead of looping over all
 * pins
 */
#ifdef OGM_BRIDGE_MODE
bool ModbusRTUSlave::bridgeConsumeCoils(uint16_t& start, uint16_t& count, uint16_t& ops, bool& ff,
                                        uint8_t& snapshotCount, bool snapshot[]){
  BridgePending pending;
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  if(!_bridgeSourceJournal.peek(BridgeIngressTable::Coils, pending) ||
     !_bridgeSourceJournal.retire(
         BridgeIngressTable::Coils, pending.sourceToken)){
    return false;
  }
  start = pending.start;
  count = pending.count;
  ops = pending.ops ? static_cast<uint16_t>(pending.ops) : uint16_t(1U);
  ff = (pending.meta & kBridgePendingFlagFireForget) != 0U;
  snapshotCount = pending.snapshotCount;
  if(snapshot && snapshotCount != 0U){
    memcpy(snapshot, pending.snapshot.coils,
           snapshotCount * sizeof(bool));
  }
  return true;
}

bool ModbusRTUSlave::bridgeConsumeCoils(uint16_t& start, uint16_t& count, uint16_t& ops, bool& ff){
  uint8_t snapshotCount = 0U;
  return bridgeConsumeCoils(start, count, ops, ff, snapshotCount, nullptr);
}

bool ModbusRTUSlave::bridgeConsumeCoils(uint16_t& start, uint16_t& count, uint16_t& ops){
  bool ff = false;
  return bridgeConsumeCoils(start, count, ops, ff);
}

bool ModbusRTUSlave::bridgeConsumeHolding(uint16_t& start, uint16_t& count, uint16_t& ops, bool& ff,
                                          uint8_t& snapshotCount, uint16_t snapshot[]){
  BridgePending pending;
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeSourceQueueMutex);
#endif
  if(!_bridgeSourceJournal.peek(
      BridgeIngressTable::HoldingRegisters, pending) ||
     !_bridgeSourceJournal.retire(
         BridgeIngressTable::HoldingRegisters, pending.sourceToken)){
    return false;
  }
  start = pending.start;
  count = pending.count;
  ops = pending.ops ? static_cast<uint16_t>(pending.ops) : uint16_t(1U);
  ff = (pending.meta & kBridgePendingFlagFireForget) != 0U;
  snapshotCount = pending.snapshotCount;
  if(snapshot && snapshotCount != 0U){
    memcpy(snapshot, pending.snapshot.holding,
           snapshotCount * sizeof(uint16_t));
  }
  return true;
}

bool ModbusRTUSlave::bridgeConsumeHolding(uint16_t& start, uint16_t& count, uint16_t& ops, bool& ff){
  uint8_t snapshotCount = 0U;
  return bridgeConsumeHolding(start, count, ops, ff, snapshotCount, nullptr);
}

bool ModbusRTUSlave::bridgeConsumeHolding(uint16_t& start, uint16_t& count, uint16_t& ops){
  bool ff = false;
  return bridgeConsumeHolding(start, count, ops, ff);
}

bool ModbusRTUSlave::bridgeConsumeOverflowCoils(uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason, bool& ff){
  return bridgeOverflowDequeue(true, start, count, ops, reason, ff);
}

bool ModbusRTUSlave::bridgeConsumeOverflowHolding(uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason, bool& ff){
  return bridgeOverflowDequeue(false, start, count, ops, reason, ff);
}

bool ModbusRTUSlave::bridgeConsumeOverflowCoils(uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason){
  bool ff = false;
  return bridgeConsumeOverflowCoils(start, count, ops, reason, ff);
}

bool ModbusRTUSlave::bridgeConsumeOverflowHolding(uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason){
  bool ff = false;
  return bridgeConsumeOverflowHolding(start, count, ops, reason, ff);
}

bool ModbusRTUSlave::bridgeConsumeOverflowCoils(uint16_t& start, uint16_t& count, uint16_t& ops){
  uint8_t reason = kBridgeDropReasonOverflow;
  return bridgeConsumeOverflowCoils(start, count, ops, reason);
}

bool ModbusRTUSlave::bridgeConsumeOverflowHolding(uint16_t& start, uint16_t& count, uint16_t& ops){
  uint8_t reason = kBridgeDropReasonOverflow;
  return bridgeConsumeOverflowHolding(start, count, ops, reason);
}

bool ModbusRTUSlave::bridgeConsumeOverflow(){
  uint16_t s = 0, c = 0, o = 0;
  uint8_t r = kBridgeDropReasonOverflow;
  return bridgeConsumeOverflowCoils(s, c, o, r) || bridgeConsumeOverflowHolding(s, c, o, r);
}

uint16_t ModbusRTUSlave::bridgeOpsForCount(uint16_t count, bool isCoil) const{
  // Keep opsCount aligned with master bridge ack/fail tracking; this mirrors
  // the downstream chunking implied by MAX_MULTI_*.
  static_assert(PinIndexDefines::MAX_MULTI_COILS > 0, "MAX_MULTI_COILS must be > 0 for ops accounting");
  static_assert(PinIndexDefines::MAX_MULTI_HRS > 0, "MAX_MULTI_HRS must be > 0 for ops accounting");
  constexpr uint16_t kMaxBridgeWriteCoils = 1968;
  constexpr uint16_t kMaxBridgeWriteHrs = 123;
  constexpr uint16_t kMaxCoilOps = static_cast<uint16_t>(
      (kMaxBridgeWriteCoils + (PinIndexDefines::MAX_MULTI_COILS - 1u)) /
      PinIndexDefines::MAX_MULTI_COILS);
  constexpr uint16_t kMaxHrOps = static_cast<uint16_t>(
      (kMaxBridgeWriteHrs + (PinIndexDefines::MAX_MULTI_HRS - 1u)) /
      PinIndexDefines::MAX_MULTI_HRS);
  static_assert(kMaxCoilOps <= 0xFFu, "BridgePending::ops no longer fits coil write op count");
  static_assert(kMaxHrOps <= 0xFFu, "BridgePending::ops no longer fits holding write op count");
  return BridgeIngressJournal::operationCount(
      isCoil ? BridgeIngressTable::Coils
             : BridgeIngressTable::HoldingRegisters,
      count);
}

void ModbusRTUSlave::bridgeOverflowPush(bool isCoil, uint16_t start, uint16_t count,
                                        uint16_t ops, uint8_t reason, bool ff,
                                        uint16_t sessionGeneration){
#ifdef USING_STATS
  if(reason == kBridgeDropReasonInactive){
    PinTypes_slave::SlaveStats::recordError(PinTypes_slave::SlaveStats::kErrCodeBridgeInactive);
  }else{
    PinTypes_slave::SlaveStats::recordError(PinTypes_slave::SlaveStats::kErrCodeBridgeOverflow);
    PinTypes_slave::SlaveStats::recordOverflow(ops ? ops : uint16_t(1));
  }
#endif
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  BridgePending* q = isCoil ? _bridgeOverflowCoilQ : _bridgeOverflowHrQ;
  volatile uint8_t& head = isCoil ? _bridgeOverflowCoilHead : _bridgeOverflowHrHead;
  volatile uint8_t& tail = isCoil ? _bridgeOverflowCoilTail : _bridgeOverflowHrTail;
  const uint8_t size = kBridgeOverflowQueueSize;
  const uint16_t normalizedOps = ops ? ops : uint16_t(1);

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
  q[curTail].sessionGeneration = sessionGeneration;
  uint16_t token = static_cast<uint16_t>(_bridgeOverflowToken + 1U);
  if(token == 0U) token = 1U;
  _bridgeOverflowToken = token;
  q[curTail].sourceToken = token;
  q[curTail].ops = static_cast<uint8_t>(normalizedOps);
  q[curTail].meta = static_cast<uint8_t>(
      (ff ? kBridgePendingFlagFireForget : 0U) |
      (reason & kBridgePendingReasonMask));
  q[curTail].snapshotCount = 0U;
  head = curHead;
  tail = nextTail;
}

bool ModbusRTUSlave::bridgeOverflowDequeue(bool isCoil, uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason, bool& ff){
#ifdef OGM_USE_MUTEX
  LockGuard queueGuard(_bridgeOverflowQueueMutex);
#endif
  BridgePending* q = isCoil ? _bridgeOverflowCoilQ : _bridgeOverflowHrQ;
  volatile uint8_t& head = isCoil ? _bridgeOverflowCoilHead : _bridgeOverflowHrHead;
  volatile uint8_t& tail = isCoil ? _bridgeOverflowCoilTail : _bridgeOverflowHrTail;
  uint8_t curHead = head;
  const uint8_t curTail = tail;
  if(curHead == curTail) return false;
  const BridgePending& pending = q[curHead];
  start = pending.start;
  count = pending.count;
  ops = pending.ops ? static_cast<uint16_t>(pending.ops) : uint16_t(1U);
  reason = static_cast<uint8_t>(pending.meta & kBridgePendingReasonMask);
  ff = (pending.meta & kBridgePendingFlagFireForget) != 0U;
  curHead = static_cast<uint8_t>(
      (curHead + 1U) % kBridgeOverflowQueueSize);
  head = curHead;
  return true;
}
#endif
