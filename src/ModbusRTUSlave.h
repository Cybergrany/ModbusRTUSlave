// Includes code derived from CMB27/ModbusRTUSlave (MIT).
// See THIRD_PARTY_NOTICES.md and LICENSES/CMB27-ModbusRTUSlave-MIT.txt.
#ifndef ModbusRTUSlave_h
#define ModbusRTUSlave_h

// #define MODBUS_USE_CRC_TABLE_256

#ifndef MODBUS_RTU_SLAVE_BUF_SIZE
#define MODBUS_RTU_SLAVE_BUF_SIZE 256
#endif
#define NO_DE_PIN 255
#define NO_ID 0

#if defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_GIGA)
#define FLUSH_COMPENSATION_DELAY
#endif

#include "Arduino.h"
#include "detail/ModbusRTUIngressJournal.h"
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
#ifndef MBUS_RTU_SLAVE_MUTEX_HEADER
#define MBUS_RTU_SLAVE_MUTEX_HEADER "platform/PlatformMutex.h"
#endif
#include MBUS_RTU_SLAVE_MUTEX_HEADER
#endif
//#ifdef __AVR__
////#include <SoftwareSerial.h>
//#endif

#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
#ifndef MBUS_RTU_SLAVE_BRIDGE_MAX_COILS
#define MBUS_RTU_SLAVE_BRIDGE_MAX_COILS 64U
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_MAX_HOLDING_REGISTERS
#define MBUS_RTU_SLAVE_BRIDGE_MAX_HOLDING_REGISTERS 32U
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_QUEUE_SIZE
#define MBUS_RTU_SLAVE_BRIDGE_QUEUE_SIZE 50U
#endif

static_assert(MBUS_RTU_SLAVE_BRIDGE_MAX_COILS > 0U,
              "Bridge coil snapshots need non-zero capacity");
static_assert(MBUS_RTU_SLAVE_BRIDGE_MAX_HOLDING_REGISTERS > 0U,
              "Bridge register snapshots need non-zero capacity");
static_assert(MBUS_RTU_SLAVE_BRIDGE_QUEUE_SIZE > 1U &&
                  MBUS_RTU_SLAVE_BRIDGE_QUEUE_SIZE <= 255U,
              "Bridge queue size must fit its uint8_t ring indexes");

static constexpr uint8_t kBridgeQueueSize =
    static_cast<uint8_t>(MBUS_RTU_SLAVE_BRIDGE_QUEUE_SIZE);
static constexpr uint8_t kBridgeOverflowQueueSize = kBridgeQueueSize;

using BridgeIngressJournal = ModbusRTU::FixedCapacityIngressJournal<
    MBUS_RTU_SLAVE_BRIDGE_MAX_COILS,
    MBUS_RTU_SLAVE_BRIDGE_MAX_HOLDING_REGISTERS,
    kBridgeQueueSize>;
using BridgeIngressEntry = BridgeIngressJournal::Entry;
#endif

// Optional bridge-only diag for the actual master-facing reply path.
//
// This is intentionally lean:
// - compile-gated only
// - no per-request logging
// - no added state on bridge runtime queues
// - one snapshot/reset API so the bridge upstream thread can log summaries
//
// It focuses on accepted write replies because that is the path that should
// acknowledge immediately regardless of downstream handler work.
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS 0
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_INTERVAL_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_INTERVAL_MS 1000UL
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_2_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_2_MS 2UL
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_5_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_5_MS 5UL
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_10_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_10_MS 10UL
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_20_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_PUMP_SLOW_20_MS 20UL
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_5_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_5_MS 5UL
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_10_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_10_MS 10UL
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_20_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_20_MS 20UL
#endif
#ifndef MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_40_MS
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_DONE_SLOW_40_MS 40UL
#endif
#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS && defined(MBUS_RTU_SLAVE_BRIDGE_MODE)
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED 1
#else
#define MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED 0
#endif

class ModbusRTUSlave {
  public:
    // --- Targeted Broadcast (FC 0x45) API ---
    // static constexpr uint8_t  FC_TARGETED_BROADCAST = 69;   // 0x45
    // static constexpr uint16_t FC69_MAX_REGS         = 122;  // write multiple registers
    // static constexpr uint16_t FC69_MAX_COILS        = 1952; // write multiple coils

    // Offsets within current request buffer for FC 69
    // static constexpr uint8_t FC69_OFFSET_TARGET     = 2;
    static constexpr uint8_t FC69_OFFSET_INNER_FC   = 3;
    // static constexpr uint8_t FC69_OFFSET_INNER_PDU  = 4;

    ModbusRTUSlave(HardwareSerial& serial, uint8_t dePin = NO_DE_PIN);
//    #ifdef __AVR__
//    ModbusRTUSlave(SoftwareSerial& serial, uint8_t dePin = NO_DE_PIN);
//    #endif
    #ifdef HAVE_CDCSERIAL
    ModbusRTUSlave(Serial_& serial, uint8_t dePin = NO_DE_PIN);
    #endif
    //can pass nullptr
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    void configurePlatformMutex(PlatformMutex* coilMut,
                                PlatformMutex* diMut,
                                PlatformMutex* irMut,
                                PlatformMutex* hrMut);
    #endif
    void configureCoils(bool coils[], uint16_t numCoils);
    void configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs);
    void configureHoldingRegisters(uint16_t holdingRegisters[], uint16_t numHoldingRegisters);
    void configureInputRegisters(uint16_t inputRegisters[], uint16_t numInputRegisters);
    #ifdef ESP32
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false);
    #else
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1);
    #endif
    void poll();
    void tx_pump();
#ifdef MBUS_RTU_SLAVE_WORK_ACCESSORS
    struct WorkState {
      uint8_t flags = 0;
      bool rxInFrame() const { return flags & 0x01; }
      bool txBusy() const { return flags & 0x02; }
    };
    WorkState workState() const;
    bool hasWorkPending() const;
#endif
#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    struct DebugInfo {
      volatile uint32_t last_poll_us = 0;
      volatile uint32_t last_poll_gap_ms = 0;
      volatile uint32_t max_poll_gap_ms = 0;
      volatile uint32_t last_byte_us = 0;
      volatile uint32_t last_frame_us = 0;
      volatile uint32_t frames_ok = 0;
      volatile uint32_t frames_bad = 0;
      volatile uint16_t last_bad_len = 0;
      volatile uint8_t last_bad_id = 0;
      volatile uint8_t last_bad_fc = 0;
      volatile uint16_t last_bad_start = 0;
      volatile uint32_t short_frames = 0;
      volatile uint32_t rx_overflows = 0;
      volatile uint16_t max_rx_len = 0;
      volatile uint32_t ignored = 0;
      volatile uint32_t ignored_broadcast_reads = 0;
      volatile uint8_t last_ignored_id = 0;
      volatile uint8_t last_ignored_fc = 0;
      volatile uint16_t last_ignored_start = 0;
      volatile uint16_t last_ignored_count = 0;
      volatile uint32_t not_addressed = 0;
      volatile uint8_t last_not_addr_id = 0;
      volatile uint32_t tx_count = 0;
      volatile uint32_t last_tx_us = 0;
      volatile uint8_t last_tx_id = 0;
      volatile uint8_t last_tx_fc = 0;
      volatile uint16_t last_tx_len = 0;
      volatile uint32_t last_req_us = 0;
      volatile uint8_t last_req_id = 0;
      volatile uint8_t last_req_fc = 0;
      volatile uint16_t last_req_len = 0;
      volatile uint32_t last_resp_us = 0;
      volatile uint32_t max_resp_us = 0;
      volatile uint32_t last_tx_done_us = 0;
      volatile uint32_t last_tx_busy_us = 0;
      volatile uint32_t max_tx_busy_us = 0;
    };
    const DebugInfo& debugInfo() const { return dbg_; }
#endif

#ifdef MBUS_RTU_SLAVE_EVENT_CALLBACKS
    enum EventCode : uint16_t {
      kEventCrcMismatch = 0x0100U,
      kEventMalformedFrame = 0x0101U,
      kEventBridgeOverflow = 0x0102U,
      kEventBridgeAdmissionRejected = 0x0103U,
    };
    // units is one for protocol/parser errors and the affected journal work
    // unit count for a bridge overflow or rejected admission.
    using EventFn = void (*)(uint16_t code, uint16_t units);
    static void setEventFn(EventFn fn);
#endif

#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    enum BridgeDropReason : uint8_t {
      kBridgeDropReasonOverflow = 0,
      kBridgeDropReasonAdmissionRejected = 1,
    };
    static constexpr uint8_t kBridgeIngressFlagFireForget = 0x80U;
    static constexpr uint8_t kBridgeIngressFlagResponseRequired = 0x40U;
    static constexpr uint8_t kBridgeIngressReasonMask = 0x3FU;

    // Bridge helpers: report pending upstream writes (single or multi) per table.
    bool bridgeConsumeCoils(uint16_t& start, uint16_t& count, uint16_t& ops, bool& ff,
                            uint8_t& snapshotCount, bool snapshot[]);
    bool bridgeConsumeCoils(uint16_t& start, uint16_t& count, uint16_t& ops, bool& ff);
    bool bridgeConsumeCoils(uint16_t& start, uint16_t& count, uint16_t& ops);
    bool bridgeConsumeHolding(uint16_t& start, uint16_t& count, uint16_t& ops, bool& ff,
                              uint8_t& snapshotCount, uint16_t snapshot[]);
    bool bridgeConsumeHolding(uint16_t& start, uint16_t& count, uint16_t& ops, bool& ff);
    bool bridgeConsumeHolding(uint16_t& start, uint16_t& count, uint16_t& ops);
    bool bridgeConsumeOverflowCoils(uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason, bool& ff);
    bool bridgeConsumeOverflowHolding(uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason, bool& ff);
    bool bridgeConsumeOverflowCoils(uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason);
    bool bridgeConsumeOverflowHolding(uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason);
    bool bridgeConsumeOverflowCoils(uint16_t& start, uint16_t& count, uint16_t& ops);
    bool bridgeConsumeOverflowHolding(uint16_t& start, uint16_t& count, uint16_t& ops);
    // Convenience check when the caller only needs to know that a drop exists.
    bool bridgeConsumeOverflow();
    // Durable source transfer API. Peek copies the current head without
    // releasing it. Commit succeeds only for the exact token returned by peek.
    // A destination queue that is full must not call commit.
    // The combined API preserves accepted order across the separate coil and
    // holding rings. isCoil identifies the selected ring for exact commit.
    bool bridgePeekNext(BridgeIngressEntry& pending, bool& isCoil) const;
    bool bridgeCommitNext(bool isCoil, uint16_t sourceToken);
    bool bridgePeekCoils(BridgeIngressEntry& pending) const;
    bool bridgeCommitCoils(uint16_t sourceToken);
    bool bridgePeekHolding(BridgeIngressEntry& pending) const;
    bool bridgeCommitHolding(uint16_t sourceToken);
    bool bridgePeekOverflowCoils(BridgeIngressEntry& pending) const;
    bool bridgeCommitOverflowCoils(uint16_t sourceToken);
    bool bridgePeekOverflowHolding(BridgeIngressEntry& pending) const;
    bool bridgeCommitOverflowHolding(uint16_t sourceToken);
    // Optional range policy and applied-write observer. Local ranges bypass
    // the ingress journal. The observer runs after any successful table
    // mutation, before a unicast reply is queued.
    using BridgeLocalRangeFn = bool (*)(uint16_t start, uint16_t count, bool isCoil);
    using BridgeWriteAppliedFn = void (*)(uint16_t start, uint16_t count,
                                          bool isCoil, bool isLocal);
    using BridgeAdmissionFn = bool (*)(uint16_t start, uint16_t count,
                                      bool isCoil, bool fireForget,
                                      uint16_t& context);
    static void setBridgeLocalRangeFn(BridgeLocalRangeFn fn);
    static void setBridgeWriteAppliedFn(BridgeWriteAppliedFn fn);
    static void setBridgeAdmissionFn(BridgeAdmissionFn fn);
#endif

#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
    static constexpr uint8_t kBridgeUpstreamTxFcCount = 4U;
    static constexpr uint8_t kBridgeUpstreamTxQtyBucketCount = 4U;
    static constexpr uint8_t kBridgeUpstreamTxThresholdCount = 4U;

    struct BridgeUpstreamTxDiagSnapshot {
      uint32_t accepted = 0;
      uint32_t queued = 0;
      uint32_t txPumpSeen = 0;
      uint32_t txDone = 0;
      uint32_t pumpMsSum = 0;
      uint32_t pumpMsMax = 0;
      uint32_t doneMsSum = 0;
      uint32_t doneMsMax = 0;
      uint32_t pumpOverMs[kBridgeUpstreamTxThresholdCount]{};
      uint32_t doneOverMs[kBridgeUpstreamTxThresholdCount]{};
      uint32_t bucketAccepted[kBridgeUpstreamTxFcCount][kBridgeUpstreamTxQtyBucketCount]{};
      uint32_t bucketQueued[kBridgeUpstreamTxFcCount][kBridgeUpstreamTxQtyBucketCount]{};
      uint32_t bucketSlowDone[kBridgeUpstreamTxFcCount][kBridgeUpstreamTxQtyBucketCount]{};
      uint32_t bucketDoneMaxMs[kBridgeUpstreamTxFcCount][kBridgeUpstreamTxQtyBucketCount]{};
    };

    void copyAndResetBridgeUpstreamTxDiag(BridgeUpstreamTxDiagSnapshot& out);
#endif

  private:
    HardwareSerial *_hardwareSerial = 0;
    #ifdef MBUS_RTU_SLAVE_USE_MUTEX
    PlatformMutex* _coilMut = nullptr;
    PlatformMutex* _diMut = nullptr;
    PlatformMutex* _irMut = nullptr;
    PlatformMutex* _hrMut = nullptr;
    #endif
//    #ifdef __AVR__
//    SoftwareSerial *_softwareSerial = 0;
//    #endif
    #ifdef HAVE_CDCSERIAL
    Serial_ *_usbSerial = 0;
    #endif
    Stream *_serial;
    uint8_t _dePin;
    uint8_t _buf[MODBUS_RTU_SLAVE_BUF_SIZE];
    bool *_coils = 0;
    bool *_discreteInputs = 0;
    uint16_t *_holdingRegisters = 0;
    uint16_t *_inputRegisters = 0;
    uint16_t _numCoils = 0;
    uint16_t _numDiscreteInputs = 0;
    uint16_t _numHoldingRegisters = 0;
    uint16_t _numInputRegisters = 0;
    uint8_t _id = NO_ID;
    unsigned long _charTimeUs = 0;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;

    // Per-instance RX frame tracking
    uint16_t _rxNumBytes = 0;
    uint32_t _rxLastByteUs = 0;
    volatile bool _rxInFrame = false;

    #ifdef FLUSH_COMPENSATION_DELAY
    unsigned long _flushCompensationDelay;
    #endif

    void _processReadCoils();
    void _processReadDiscreteInputs();
    void _processReadHoldingRegisters();
    void _processReadInputRegisters();
    void _processWriteSingleCoil();
    void _processWriteSingleHoldingRegister();
    void _processWriteMultipleCoils();
    void _processWriteMultipleHoldingRegisters();

    bool _readRequest();
    void _writeResponse(uint8_t len);
    void _exceptionResponse(uint8_t code);
    void _clearRxBuffer();

    void _calculateTimeouts(unsigned long baud, uint32_t config);
    uint16_t _crc(uint8_t len);
    uint16_t _div8RndUp(uint16_t value);
    uint16_t _bytesToWord(uint8_t high, uint8_t low);

    volatile bool _txBusy=false;
    uint8_t _txLen=0; uint8_t _rxLen = 0;

    // Per-instance TX tracking
    bool _txWasBusy = false;
    uint32_t _txStartUs = 0;
    uint32_t _txDoneUs = 0;

#ifdef MBUS_RTU_SLAVE_DIAGNOSTICS
    DebugInfo dbg_{};
#endif

#ifdef MBUS_RTU_SLAVE_BRIDGE_MODE
    using BridgeIngressReservation = BridgeIngressJournal::Reservation;
    using BridgeIngressTable = BridgeIngressJournal::Table;

    // Only successfully admitted writes enter the source journal. Rejected or
    // saturated writes are retained separately as diagnostic entries.
    BridgeIngressJournal _bridgeSourceJournal;

    BridgeIngressEntry _bridgeOverflowCoilQ[kBridgeOverflowQueueSize];
    volatile uint8_t _bridgeOverflowCoilHead = 0U;
    volatile uint8_t _bridgeOverflowCoilTail = 0U;
    BridgeIngressEntry _bridgeOverflowHrQ[kBridgeOverflowQueueSize];
    volatile uint8_t _bridgeOverflowHrHead = 0U;
    volatile uint8_t _bridgeOverflowHrTail = 0U;
    uint16_t _bridgeOverflowToken = 1U;

    // Reserve every required source slot before register mutation. The stack
    // reservation is then committed under the register-image lock so its
    // immutable snapshot is durable before an ACK can be queued.
    bool bridgeReserveCoilIngress(
        uint16_t start, uint16_t count, bool ff,
        uint16_t context, bool responseRequired,
        BridgeIngressReservation& reservation);
    bool bridgeReserveHoldingIngress(
        uint16_t start, uint16_t count, bool ff,
        uint16_t context, bool responseRequired,
        BridgeIngressReservation& reservation);
    bool bridgeReserveIngressFailure(
        uint16_t start, uint16_t count, bool isCoil, bool ff,
        uint16_t context);
    bool bridgeCommitCoilIngress(
        const BridgeIngressReservation& reservation);
    bool bridgeCommitHoldingIngress(
        const BridgeIngressReservation& reservation);
    bool bridgeCommitIngressFailure(
        const BridgeIngressReservation& reservation, bool isCoil);
    bool bridgeWriteAllowed(uint16_t start, uint16_t count, bool isCoil, bool ff,
                            uint16_t& context);
    bool bridgeIsLocalRange(uint16_t start, uint16_t count, bool isCoil) const;
    void bridgeNotifyWriteApplied(uint16_t start, uint16_t count, bool isCoil,
                                  bool isLocal) const;
    uint16_t bridgeUnitsForCount(uint16_t count, bool isCoil) const;
    void bridgeOverflowPush(bool isCoil, uint16_t start, uint16_t count, uint16_t units,
                            uint8_t reason, bool ff, uint16_t context);
    bool bridgeOverflowDequeue(bool isCoil, uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason, bool& ff);
    static BridgeLocalRangeFn _bridgeLocalRangeFn;
    static BridgeWriteAppliedFn _bridgeWriteAppliedFn;
    static BridgeAdmissionFn _bridgeAdmissionFn;
#ifdef MBUS_RTU_SLAVE_USE_MUTEX
    // Source and diagnostic rings are SPSC across the upstream and bridge
    // runtime threads. Protect payload publication as well as head/tail state;
    // volatile indexes alone do not provide a cross-core happens-before edge.
    mutable SafePlatformMutex _bridgeSourceQueueMutex;
    mutable SafePlatformMutex _bridgeOverflowQueueMutex;
#endif

#endif

#ifdef MBUS_RTU_SLAVE_EVENT_CALLBACKS
    static EventFn _eventFn;
    static void noteEvent(uint16_t code, uint16_t units = 1U);
#endif

#if MBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS_ENABLED
    struct BridgeUpstreamTxDiagState {
      uint32_t accepted = 0;
      uint32_t queued = 0;
      uint32_t txPumpSeen = 0;
      uint32_t txDone = 0;
      uint32_t pumpMsSum = 0;
      uint32_t pumpMsMax = 0;
      uint32_t doneMsSum = 0;
      uint32_t doneMsMax = 0;
      uint32_t pumpOverMs[kBridgeUpstreamTxThresholdCount]{};
      uint32_t doneOverMs[kBridgeUpstreamTxThresholdCount]{};
      uint32_t bucketAccepted[kBridgeUpstreamTxFcCount][kBridgeUpstreamTxQtyBucketCount]{};
      uint32_t bucketQueued[kBridgeUpstreamTxFcCount][kBridgeUpstreamTxQtyBucketCount]{};
      uint32_t bucketSlowDone[kBridgeUpstreamTxFcCount][kBridgeUpstreamTxQtyBucketCount]{};
      uint32_t bucketDoneMaxMs[kBridgeUpstreamTxFcCount][kBridgeUpstreamTxQtyBucketCount]{};
      uint8_t pendingFcIndex = 0xFFU;
      uint8_t pendingQtyBucket = 0U;
      bool pendingActive = false;
      uint32_t queuedAtUs = 0;
    };

    static uint8_t bridgeUpstreamTxDiagFcIndex(uint8_t fc);
    static uint8_t bridgeUpstreamTxDiagQtyBucket(uint16_t quantity);
    static void bridgeUpstreamTxDiagNoteThresholds(uint32_t valueMs,
                                                   uint32_t (&thresholds)[kBridgeUpstreamTxThresholdCount],
                                                   bool pumpPhase);
    void bridgeUpstreamTxDiagNoteAccepted(uint8_t fc, uint16_t quantity);
    void bridgeUpstreamTxDiagNoteQueued(uint32_t queuedAtUs);
    void bridgeUpstreamTxDiagNotePumpSeen(uint32_t pumpSeenUs);
    void bridgeUpstreamTxDiagNoteTxDone(uint32_t doneUs);

    BridgeUpstreamTxDiagState _bridgeUpstreamTxDiag{};
#endif
};

#endif
