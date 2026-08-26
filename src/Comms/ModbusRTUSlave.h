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
#include "Comms/ModbusRTUIngressJournal.h"
#ifdef OGM_USE_MUTEX
#include "platform/PlatformMutex.h"
#endif
//#ifdef __AVR__
////#include <SoftwareSerial.h>
//#endif

#ifdef OGM_BRIDGE_MODE
#include "IO/ExternalPins/PinIndexDefines.h"
// Preserve the existing bridge-facing names while moving the queue mechanism
// behind a product-neutral, allocation-free public API. PinIndexDefines is now
// only an OGM adapter input used to instantiate the generic snapshot bounds.
static constexpr uint8_t kBridgeQueueSize = 50;
static constexpr uint8_t kBridgeOverflowQueueSize = kBridgeQueueSize;
using BridgePendingSnapshot = ModbusRTU::FixedCapacityIngressSnapshot<
    PinIndexDefines::MAX_MULTI_COILS,
    PinIndexDefines::MAX_MULTI_HRS>;

// OGM compatibility record. Its established names and byte width stay intact;
// BridgePendingIngressTraits maps the neutral journal concepts at compile time
// so forwarding requires neither type punning nor an additional record copy.
struct BridgePending {
  uint16_t start = 0U;
  uint16_t count = 0U;
  uint16_t sessionGeneration = 0U;
  uint16_t sourceToken = 0U;
  uint8_t ops = 1U;
  uint8_t meta = 0U;
  uint8_t snapshotCount = 0U;
  BridgePendingSnapshot snapshot{};
};

struct BridgePendingIngressTraits {
  using Entry = BridgePending;

  static uint16_t sourceToken(const Entry& entry) {
    return entry.sourceToken;
  }

  static void initialize(
      Entry& entry, uint16_t start, uint16_t count, uint16_t context,
      uint16_t sourceToken, uint8_t units, uint8_t attributes,
      uint8_t snapshotCount) {
    entry.start = start;
    entry.count = count;
    entry.sessionGeneration = context;
    entry.sourceToken = sourceToken;
    entry.ops = units;
    entry.meta = attributes;
    entry.snapshotCount = snapshotCount;
  }

  #if defined(__GNUC__) || defined(__clang__)
  __attribute__((always_inline))
  #endif
  static inline void copyCoils(
      Entry& entry, const bool* values, uint16_t count) {
    if(count == 1U){
      entry.snapshot.coils[0] = values[0];
      return;
    }
    memcpy(entry.snapshot.coils, values,
           static_cast<size_t>(count) * sizeof(bool));
  }

  #if defined(__GNUC__) || defined(__clang__)
  __attribute__((always_inline))
  #endif
  static inline void copyHolding(
      Entry& entry, const uint16_t* values, uint16_t count) {
    if(count == 1U){
      entry.snapshot.holding[0] = values[0];
      return;
    }
    memcpy(entry.snapshot.holding, values,
           static_cast<size_t>(count) * sizeof(uint16_t));
  }
};

using BridgeIngressJournal = ModbusRTU::FixedCapacityIngressJournal<
    PinIndexDefines::MAX_MULTI_COILS,
    PinIndexDefines::MAX_MULTI_HRS,
    kBridgeQueueSize,
    BridgePendingIngressTraits>;
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
#ifndef BRIDGE_UPSTREAM_TX_DIAG
#define BRIDGE_UPSTREAM_TX_DIAG 0
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_INTERVAL_MS
#define BRIDGE_UPSTREAM_TX_DIAG_INTERVAL_MS 1000UL
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_2_MS
#define BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_2_MS 2UL
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_5_MS
#define BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_5_MS 5UL
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_10_MS
#define BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_10_MS 10UL
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_20_MS
#define BRIDGE_UPSTREAM_TX_DIAG_PUMP_SLOW_20_MS 20UL
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_5_MS
#define BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_5_MS 5UL
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_10_MS
#define BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_10_MS 10UL
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_20_MS
#define BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_20_MS 20UL
#endif
#ifndef BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_40_MS
#define BRIDGE_UPSTREAM_TX_DIAG_DONE_SLOW_40_MS 40UL
#endif
#if BRIDGE_UPSTREAM_TX_DIAG && defined(OGM_BRIDGE_MODE)
#define BRIDGE_UPSTREAM_TX_DIAG_ENABLED 1
#else
#define BRIDGE_UPSTREAM_TX_DIAG_ENABLED 0
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
    #ifdef OGM_USE_MUTEX
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
#ifdef OGM_MODBUS_MT_ACCESSORS
    struct WorkState {
      uint8_t flags = 0;
      bool rxInFrame() const { return flags & 0x01; }
      bool txBusy() const { return flags & 0x02; }
    };
    WorkState workState() const;
    bool hasWorkPending() const;
#endif
#ifdef USB_DEBUG
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

#ifdef OGM_BRIDGE_MODE
    enum BridgeDropReason : uint8_t {
      kBridgeDropReasonOverflow = 0,
      kBridgeDropReasonInactive = 1,
    };
    static constexpr uint8_t kBridgePendingFlagFireForget = 0x80U;
    static constexpr uint8_t kBridgePendingFlagPublicDebt = 0x40U;
    static constexpr uint8_t kBridgePendingReasonMask = 0x3FU;

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
    // Legacy aggregate overflow check (no details).
    bool bridgeConsumeOverflow();
    // Durable source transfer API. Peek copies the current head without
    // releasing it. Commit succeeds only for the exact token returned by peek.
    // A destination queue that is full must not call commit.
    // The combined API preserves accepted order across the separate coil and
    // holding rings. isCoil identifies the selected ring for exact commit.
    bool bridgePeekNext(BridgePending& pending, bool& isCoil) const;
    bool bridgeCommitNext(bool isCoil, uint16_t sourceToken);
    bool bridgePeekCoils(BridgePending& pending) const;
    bool bridgeCommitCoils(uint16_t sourceToken);
    bool bridgePeekHolding(BridgePending& pending) const;
    bool bridgeCommitHolding(uint16_t sourceToken);
    bool bridgePeekOverflowCoils(BridgePending& pending) const;
    bool bridgeCommitOverflowCoils(uint16_t sourceToken);
    bool bridgePeekOverflowHolding(BridgePending& pending) const;
    bool bridgeCommitOverflowHolding(uint16_t sourceToken);
    // Optional bridge-local range hook (used by bridge firmware to bypass forwarding).
    using BridgeLocalRangeFn = bool (*)(uint16_t start, uint16_t count, bool isCoil);
    using BridgeLocalWriteFn = void (*)(uint16_t start, uint16_t count, bool isCoil);
    using BridgeAdmissionFn = bool (*)(uint16_t start, uint16_t count,
                                      bool isCoil, bool fireForget,
                                      uint16_t& sessionGeneration);
    static void setBridgeLocalRangeFn(BridgeLocalRangeFn fn);
    static void setBridgeLocalWriteFn(BridgeLocalWriteFn fn);
    static void setBridgeAdmissionFn(BridgeAdmissionFn fn);
#endif

#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
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
    #ifdef OGM_USE_MUTEX
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

#ifdef USB_DEBUG
    DebugInfo dbg_{};
#endif

#ifdef OGM_BRIDGE_MODE
    using BridgeIngressReservation = BridgeIngressJournal::Reservation;
    using BridgeIngressTable = BridgeIngressJournal::Table;

    // Only successfully admitted writes enter the neutral journal. OGM loss
    // accounting remains an explicit adapter concern below.
    BridgeIngressJournal _bridgeSourceJournal;

    BridgePending _bridgeOverflowCoilQ[kBridgeOverflowQueueSize];
    volatile uint8_t _bridgeOverflowCoilHead = 0U;
    volatile uint8_t _bridgeOverflowCoilTail = 0U;
    BridgePending _bridgeOverflowHrQ[kBridgeOverflowQueueSize];
    volatile uint8_t _bridgeOverflowHrHead = 0U;
    volatile uint8_t _bridgeOverflowHrTail = 0U;
    uint16_t _bridgeOverflowToken = 1U;

    // Reserve every required source slot before register mutation. The stack
    // reservation is then committed under the register-image lock so its
    // immutable snapshot is durable before an ACK can be queued.
    bool bridgeReserveCoilIngress(
        uint16_t start, uint16_t count, bool ff,
        uint16_t sessionGeneration, bool publicDebt,
        BridgeIngressReservation& reservation);
    bool bridgeReserveHoldingIngress(
        uint16_t start, uint16_t count, bool ff,
        uint16_t sessionGeneration, bool publicDebt,
        BridgeIngressReservation& reservation);
    bool bridgeReserveIngressFailure(
        uint16_t start, uint16_t count, bool isCoil, bool ff,
        uint16_t sessionGeneration);
    bool bridgeCommitCoilIngress(
        const BridgeIngressReservation& reservation);
    bool bridgeCommitHoldingIngress(
        const BridgeIngressReservation& reservation);
    bool bridgeCommitIngressFailure(
        const BridgeIngressReservation& reservation, bool isCoil);
    bool bridgeWriteAllowed(uint16_t start, uint16_t count, bool isCoil, bool ff,
                            uint16_t& sessionGeneration);
    bool bridgeIsLocalRange(uint16_t start, uint16_t count, bool isCoil) const;
    bool bridgeShouldNotifyLocalWrite(uint16_t start, uint16_t count, bool isCoil, bool isLocal) const;
    void bridgeNotifyLocalWrite(uint16_t start, uint16_t count, bool isCoil) const;
    uint16_t bridgeOpsForCount(uint16_t count, bool isCoil) const;
    void bridgeOverflowPush(bool isCoil, uint16_t start, uint16_t count, uint16_t ops,
                            uint8_t reason, bool ff, uint16_t sessionGeneration);
    bool bridgeOverflowDequeue(bool isCoil, uint16_t& start, uint16_t& count, uint16_t& ops, uint8_t& reason, bool& ff);
    static BridgeLocalRangeFn _bridgeLocalRangeFn;
    static BridgeLocalWriteFn _bridgeLocalWriteFn;
    static BridgeAdmissionFn _bridgeAdmissionFn;
#ifdef OGM_USE_MUTEX
    // Source and diagnostic rings are SPSC across the upstream and bridge
    // runtime threads. Protect payload publication as well as head/tail state;
    // volatile indexes alone do not provide a cross-core happens-before edge.
    mutable SafePlatformMutex _bridgeSourceQueueMutex;
    mutable SafePlatformMutex _bridgeOverflowQueueMutex;
#endif
#endif

#if BRIDGE_UPSTREAM_TX_DIAG_ENABLED
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
