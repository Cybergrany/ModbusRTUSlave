#include <unity.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <functional>
#include <initializer_list>
#include <type_traits>
#include <utility>
#include <vector>

#include "Arduino.h"
#include "ModbusRTUSlave.h"

// The production source is compiled directly because test_build_src is off.
// This keeps the characterization target isolated from the firmware entrypoint.
#include "../../src/ModbusRTUSlave.cpp"

namespace {

constexpr uint8_t kUnitId = 7;
constexpr uint8_t kDePin = 5;
constexpr uint32_t kBaud = 19200;
constexpr uint32_t kFrameTimeoutUs8N1At19200 = 1823;
constexpr uint32_t kTxBudgetPerByteUs8N1At19200 = 260;

uint16_t crc16(const uint8_t* data, std::size_t len) {
  uint16_t value = 0xFFFFu;
  for (std::size_t i = 0; i < len; ++i) {
    value ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const bool lsb = (value & 1u) != 0u;
      value >>= 1;
      if (lsb) value ^= 0xA001u;
    }
  }
  return value;
}

std::vector<uint8_t> frame(std::initializer_list<uint8_t> payload) {
  std::vector<uint8_t> bytes(payload);
  const uint16_t crc = crc16(bytes.data(), bytes.size());
  bytes.push_back(static_cast<uint8_t>(crc & 0xFFu));
  bytes.push_back(static_cast<uint8_t>(crc >> 8));
  return bytes;
}

std::vector<uint8_t> frame(std::vector<uint8_t> payload) {
  const uint16_t crc = crc16(payload.data(), payload.size());
  payload.push_back(static_cast<uint8_t>(crc & 0xFFu));
  payload.push_back(static_cast<uint8_t>(crc >> 8));
  return payload;
}

std::vector<uint8_t> readRequest(uint8_t id, uint8_t fc,
                                 uint16_t start, uint16_t quantity) {
  return frame({id, fc, highByte(start), lowByte(start),
                highByte(quantity), lowByte(quantity)});
}

std::vector<uint8_t> writeSingle(uint8_t id, uint8_t fc,
                                 uint16_t address, uint16_t value) {
  return frame({id, fc, highByte(address), lowByte(address),
                highByte(value), lowByte(value)});
}

std::vector<uint8_t> writeMultipleHolding(
    uint8_t id, uint16_t start, const std::vector<uint16_t>& values) {
  std::vector<uint8_t> payload{
      id, 16, highByte(start), lowByte(start),
      highByte(values.size()), lowByte(values.size()),
      static_cast<uint8_t>(values.size() * 2u)};
  for (uint16_t value : values) {
    payload.push_back(highByte(value));
    payload.push_back(lowByte(value));
  }
  return frame(std::move(payload));
}

std::vector<uint8_t> writeMultipleCoils(
    uint8_t id, uint16_t start, const std::vector<bool>& values) {
  const uint8_t byteCount = static_cast<uint8_t>((values.size() + 7u) / 8u);
  std::vector<uint8_t> payload{
      id, 15, highByte(start), lowByte(start),
      highByte(values.size()), lowByte(values.size()), byteCount};
  payload.resize(payload.size() + byteCount, 0u);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i]) payload[7u + (i >> 3u)] |= static_cast<uint8_t>(1u << (i & 7u));
  }
  return frame(std::move(payload));
}

std::vector<uint8_t> targetedWrite(uint8_t target, uint8_t innerFc,
                                   const std::vector<uint8_t>& innerPdu) {
  std::vector<uint8_t> payload{0, 69, target, innerFc};
  payload.insert(payload.end(), innerPdu.begin(), innerPdu.end());
  return frame(std::move(payload));
}

void assertBytes(const std::vector<uint8_t>& expected,
                 const std::vector<uint8_t>& actual) {
  TEST_ASSERT_EQUAL_UINT32(expected.size(), actual.size());
  if (!expected.empty() && expected.size() == actual.size()) {
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), actual.data(), expected.size());
  }
}

class FakeSerial final : public HardwareSerial {
 public:
  struct WriteRecord {
    std::vector<uint8_t> bytes;
    uint32_t atUs = 0;
    uint64_t sequence = 0;
  };

  void begin(unsigned long newBaud) override {
    baud = newBaud;
    config = SERIAL_8N1;
    ++beginCalls;
  }

  void begin(unsigned long newBaud, uint32_t newConfig) override {
    baud = newBaud;
    config = newConfig;
    ++beginCalls;
  }

  int available() override {
    ++availableCalls;
    return static_cast<int>(rx.size());
  }

  int read() override {
    ++readCalls;
    if (rx.empty()) return -1;
    const uint8_t value = rx.front();
    rx.pop_front();
    return value;
  }

  std::size_t write(const uint8_t* data, std::size_t len) override {
    ++writeCalls;
    lastWriteLen = len;
    lastWriteSequence = ArduinoTest::nextEventSequence();
    if (captureWrites) {
      writes.push_back({std::vector<uint8_t>(data, data + len),
                        ArduinoTest::nowUs, lastWriteSequence});
    }
    if (onWrite) onWrite();
    return len;
  }

  void flush() override {
    ++flushCalls;
    lastFlushUs = ArduinoTest::nowUs;
    lastFlushSequence = ArduinoTest::nextEventSequence();
  }

  void inject(const std::vector<uint8_t>& bytes) {
    rx.insert(rx.end(), bytes.begin(), bytes.end());
  }

  void clearTrace() {
    writes.clear();
    availableCalls = 0;
    readCalls = 0;
    writeCalls = 0;
    flushCalls = 0;
    lastWriteLen = 0;
    lastWriteSequence = 0;
    lastFlushUs = 0;
    lastFlushSequence = 0;
    onWrite = nullptr;
  }

  std::deque<uint8_t> rx;
  std::vector<WriteRecord> writes;
  std::function<void()> onWrite;
  bool captureWrites = true;
  unsigned long baud = 0;
  uint32_t config = 0;
  uint32_t beginCalls = 0;
  uint64_t availableCalls = 0;
  uint64_t readCalls = 0;
  uint64_t writeCalls = 0;
  uint64_t flushCalls = 0;
  std::size_t lastWriteLen = 0;
  uint64_t lastWriteSequence = 0;
  uint32_t lastFlushUs = 0;
  uint64_t lastFlushSequence = 0;
};

struct Fixture;
Fixture* gFixture = nullptr;

struct AdmissionCall {
  uint16_t start = 0;
  uint16_t count = 0;
  bool isCoil = false;
  bool fireForget = false;
  bool observedCoil = false;
  uint16_t observedHolding = 0;
  uint64_t sequence = 0;
};

struct AppliedWriteCall {
  uint16_t start = 0;
  uint16_t count = 0;
  bool isCoil = false;
  bool isLocal = false;
  uint64_t sequence = 0;
};

struct EventCall {
  uint16_t code = 0;
  uint16_t units = 0;
  uint64_t sequence = 0;
};

struct Fixture {
  explicit Fixture(uint8_t dePin = kDePin)
      : slave(serial, dePin) {
    ArduinoTest::reset();
    coils.fill(false);
    discreteInputs.fill(false);
    holding.fill(0);
    inputRegisters.fill(0);
    slave.configureCoils(coils.data(), coils.size());
    slave.configureDiscreteInputs(discreteInputs.data(), discreteInputs.size());
    slave.configureHoldingRegisters(holding.data(), holding.size());
    slave.configureInputRegisters(inputRegisters.data(), inputRegisters.size());
    slave.configurePlatformMutex(&coilMutex, &discreteInputMutex,
                                 &inputRegisterMutex, &holdingMutex);
    gFixture = this;
    ModbusRTUSlave::setBridgeLocalRangeFn(&Fixture::localRangeThunk);
    ModbusRTUSlave::setBridgeWriteAppliedFn(&Fixture::appliedWriteThunk);
    ModbusRTUSlave::setBridgeAdmissionFn(&Fixture::admissionThunk);
    ModbusRTUSlave::setEventFn(&Fixture::eventThunk);
    slave.begin(kUnitId, kBaud, SERIAL_8N1);
    ArduinoTest::clearTrace();
    serial.clearTrace();
  }

  static bool localRangeThunk(uint16_t start, uint16_t count, bool isCoil) {
    (void)count;
    (void)isCoil;
    return gFixture && (gFixture->allLocal || start == gFixture->localStart);
  }

  static void appliedWriteThunk(uint16_t start, uint16_t count, bool isCoil,
                                bool isLocal) {
    if (!gFixture) return;
    if (gFixture->benchmarkMode) {
      ++gFixture->appliedWriteCount;
      return;
    }
    const uint64_t sequence = ArduinoTest::nextEventSequence();
    ++gFixture->appliedWriteCount;
    gFixture->appliedWrites.push_back(
        {start, count, isCoil, isLocal, sequence});
  }

  static void eventThunk(uint16_t code, uint16_t units) {
    if (!gFixture) return;
    gFixture->events.push_back(
        {code, units, ArduinoTest::nextEventSequence()});
  }

  static bool admissionThunk(uint16_t start, uint16_t count,
                             bool isCoil, bool fireForget,
                             uint16_t& context) {
    if (!gFixture) return false;
    context = gFixture->context;
    AdmissionCall call;
    call.start = start;
    call.count = count;
    call.isCoil = isCoil;
    call.fireForget = fireForget;
    call.sequence = ArduinoTest::nextEventSequence();
    if (isCoil && start < gFixture->coils.size()) {
      call.observedCoil = gFixture->coils[start];
    }
    if (!isCoil && start < gFixture->holding.size()) {
      call.observedHolding = gFixture->holding[start];
    }
    ++gFixture->admissionCount;
    if (!gFixture->benchmarkMode) gFixture->admissions.push_back(call);
    return gFixture->admit;
  }

  void startFrame(const std::vector<uint8_t>& request) {
    serial.inject(request);
    slave.poll();
  }

  void finishFrame(uint32_t gapUs = kFrameTimeoutUs8N1At19200) {
    ArduinoTest::advance(gapUs);
    slave.poll();
  }

  void transact(const std::vector<uint8_t>& request) {
    startFrame(request);
    finishFrame();
  }

  void finishTx() {
    TEST_ASSERT_TRUE(slave.workState().txBusy());
    const uint32_t duration = static_cast<uint32_t>(serial.lastWriteLen) *
                              kTxBudgetPerByteUs8N1At19200;
    slave.tx_pump();
    ArduinoTest::advance(duration);
    slave.tx_pump();
    TEST_ASSERT_FALSE(slave.workState().txBusy());
  }

  FakeSerial serial;
  ModbusRTUSlave slave;
  std::array<bool, 256> coils{};
  std::array<bool, 256> discreteInputs{};
  std::array<uint16_t, 128> holding{};
  std::array<uint16_t, 128> inputRegisters{};
  PlatformMutex coilMutex;
  PlatformMutex discreteInputMutex;
  PlatformMutex inputRegisterMutex;
  PlatformMutex holdingMutex;
  bool admit = true;
  bool allLocal = false;
  uint16_t localStart = 0xFFFFu;
  uint16_t context = 42u;
  bool benchmarkMode = false;
  uint64_t admissionCount = 0;
  uint64_t appliedWriteCount = 0;
  std::vector<AdmissionCall> admissions;
  std::vector<AppliedWriteCall> appliedWrites;
  std::vector<EventCall> events;
};

void assertSingleTx(const Fixture& fixture,
                    const std::vector<uint8_t>& expected) {
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.serial.writes.size());
  if (fixture.serial.writes.size() == 1u) {
    assertBytes(expected, fixture.serial.writes[0].bytes);
  }
}

void assertIngressHeader(const ModbusRTUSlave::BridgeIngressEntry& pending,
                         uint16_t start, uint16_t count,
                         uint16_t context, uint16_t sourceToken,
                         uint8_t expectedAttributes) {
  TEST_ASSERT_EQUAL_UINT16(start, pending.start);
  TEST_ASSERT_EQUAL_UINT16(count, pending.count);
  TEST_ASSERT_EQUAL_UINT16(context, pending.context);
  TEST_ASSERT_EQUAL_UINT16(sourceToken, pending.sourceToken);
  TEST_ASSERT_EQUAL_UINT8(1u, pending.units);
  TEST_ASSERT_EQUAL_UINT8(expectedAttributes, pending.attributes);
  TEST_ASSERT_EQUAL_UINT8(count, pending.snapshotCount);
}

void test_frame_completes_exactly_at_t35_and_read_wire_image_is_stable() {
  Fixture fixture;
  fixture.holding[1] = 0x1234u;
  fixture.holding[2] = 0xABCDu;

  fixture.startFrame(readRequest(kUnitId, 3, 1, 2));
  TEST_ASSERT_TRUE(fixture.slave.workState().rxInFrame());
  TEST_ASSERT_TRUE(fixture.slave.hasWorkPending());
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);

  ArduinoTest::advance(kFrameTimeoutUs8N1At19200 - 1u);
  fixture.slave.poll();
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_TRUE(fixture.slave.workState().rxInFrame());

  ArduinoTest::advance(1u);
  fixture.slave.poll();
  assertSingleTx(fixture, frame({kUnitId, 3, 4, 0x12, 0x34, 0xAB, 0xCD}));
  TEST_ASSERT_FALSE(fixture.slave.workState().rxInFrame());
  TEST_ASSERT_TRUE(fixture.slave.workState().txBusy());
  TEST_ASSERT_EQUAL_UINT32(kFrameTimeoutUs8N1At19200,
                           fixture.serial.writes[0].atUs);
}

void test_partial_frame_restarts_t35_from_the_last_byte() {
  Fixture fixture;
  fixture.holding[3] = 0xCAFEu;
  const auto request = readRequest(kUnitId, 3, 3, 1);

  fixture.serial.inject(std::vector<uint8_t>(request.begin(), request.begin() + 3));
  fixture.slave.poll();
  ArduinoTest::advance(kFrameTimeoutUs8N1At19200 - 20u);
  fixture.serial.inject(std::vector<uint8_t>(request.begin() + 3, request.end()));
  fixture.slave.poll();
  ArduinoTest::advance(kFrameTimeoutUs8N1At19200 - 1u);
  fixture.slave.poll();
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  ArduinoTest::advance(1u);
  fixture.slave.poll();

  assertSingleTx(fixture, frame({kUnitId, 3, 2, 0xCA, 0xFE}));
  TEST_ASSERT_EQUAL_UINT32(
      (kFrameTimeoutUs8N1At19200 - 20u) + kFrameTimeoutUs8N1At19200,
      fixture.serial.writes[0].atUs);
}

void test_crc_and_foreign_unit_rejection_leave_transport_quiet_then_resync() {
  Fixture fixture;
  fixture.holding[0] = 0x55AAu;
  auto badCrc = readRequest(kUnitId, 3, 0, 1);
  badCrc.back() ^= 0x80u;
  fixture.transact(badCrc);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(badCrc.size() + 4u, fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(badCrc.size(), fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(badCrc.size() + 4u, ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.events.size());
  TEST_ASSERT_EQUAL_HEX16(ModbusRTUSlave::kEventCrcMismatch,
                          fixture.events[0].code);
  TEST_ASSERT_EQUAL_UINT16(1u, fixture.events[0].units);
  const auto foreign = readRequest(9, 3, 0, 1);
  fixture.transact(foreign);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(badCrc.size() + foreign.size() + 8u,
                           fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(badCrc.size() + foreign.size(),
                           fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(badCrc.size() + foreign.size() + 8u,
                           ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.slave.debugInfo().frames_bad);
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.slave.debugInfo().not_addressed);
  const auto good = readRequest(kUnitId, 3, 0, 1);
  fixture.transact(good);
  assertSingleTx(fixture, frame({kUnitId, 3, 2, 0x55, 0xAA}));
  TEST_ASSERT_EQUAL_UINT64(badCrc.size() + foreign.size() + good.size() + 12u,
                           fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(badCrc.size() + foreign.size() + good.size(),
                           fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(badCrc.size() + foreign.size() + good.size() + 14u,
                           ArduinoTest::microsCalls);
}

void test_short_addressed_frame_reports_one_malformed_event_without_reply() {
  Fixture fixture;
  fixture.transact({kUnitId, 3u, 0u});

  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.events.size());
  TEST_ASSERT_EQUAL_HEX16(ModbusRTUSlave::kEventMalformedFrame,
                          fixture.events[0].code);
  TEST_ASSERT_EQUAL_UINT16(1u, fixture.events[0].units);
}

void test_all_read_function_wire_images_and_coil_padding_are_stable() {
  Fixture fixture;
  fixture.coils[1] = true;
  fixture.coils[4] = true;
  fixture.coils[9] = true;
  fixture.discreteInputs[0] = true;
  fixture.discreteInputs[7] = true;
  fixture.holding[2] = 0x1020u;
  fixture.inputRegisters[4] = 0xBEEFu;

  fixture.transact(readRequest(kUnitId, 1, 1, 10));
  assertSingleTx(fixture, frame({kUnitId, 1, 2, 0x09, 0x01}));
  fixture.finishTx();
  fixture.serial.writes.clear();

  fixture.transact(readRequest(kUnitId, 2, 0, 8));
  assertSingleTx(fixture, frame({kUnitId, 2, 1, 0x81}));
  fixture.finishTx();
  fixture.serial.writes.clear();

  fixture.transact(readRequest(kUnitId, 3, 2, 1));
  assertSingleTx(fixture, frame({kUnitId, 3, 2, 0x10, 0x20}));
  fixture.finishTx();
  fixture.serial.writes.clear();

  fixture.transact(readRequest(kUnitId, 4, 4, 1));
  assertSingleTx(fixture, frame({kUnitId, 4, 2, 0xBE, 0xEF}));
}

void test_single_write_ack_images_mutation_and_snapshots_are_stable() {
  Fixture fixture;
  ModbusRTUSlave::BridgeIngressEntry coilAtAck;
  bool coilQueueVisibleAtAck = false;
  bool coilValueAtAck = false;
  fixture.serial.onWrite = [&]() {
    coilQueueVisibleAtAck = fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, coilAtAck);
    coilValueAtAck = fixture.coils[3];
  };
  fixture.transact(writeSingle(kUnitId, 5, 3, 0xFF00u));
  TEST_ASSERT_TRUE(fixture.coils[3]);
  assertSingleTx(fixture, writeSingle(kUnitId, 5, 3, 0xFF00u));
  TEST_ASSERT_TRUE(coilQueueVisibleAtAck);
  TEST_ASSERT_TRUE(coilValueAtAck);
  TEST_ASSERT_TRUE(coilAtAck.snapshot.coils[0]);
  ModbusRTUSlave::BridgeIngressEntry pending;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, pending));
  TEST_ASSERT_EQUAL_UINT16(3u, pending.start);
  TEST_ASSERT_EQUAL_UINT8(1u, pending.snapshotCount);
  TEST_ASSERT_TRUE(pending.snapshot.coils[0]);
  TEST_ASSERT_BITS_HIGH(ModbusRTUSlave::kBridgeIngressFlagResponseRequired, pending.attributes);
  fixture.coils[3] = false;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, pending));
  TEST_ASSERT_TRUE(pending.snapshot.coils[0]);
  fixture.finishTx();

  Fixture holdingFixture;
  ModbusRTUSlave::BridgeIngressEntry holdingAtAck;
  bool holdingQueueVisibleAtAck = false;
  uint16_t holdingValueAtAck = 0;
  holdingFixture.serial.onWrite = [&]() {
    holdingQueueVisibleAtAck =
        holdingFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, holdingAtAck);
    holdingValueAtAck = holdingFixture.holding[4];
  };
  holdingFixture.transact(writeSingle(kUnitId, 6, 4, 0xCAFEu));
  TEST_ASSERT_EQUAL_HEX16(0xCAFEu, holdingFixture.holding[4]);
  assertSingleTx(holdingFixture, writeSingle(kUnitId, 6, 4, 0xCAFEu));
  TEST_ASSERT_TRUE(holdingQueueVisibleAtAck);
  TEST_ASSERT_EQUAL_HEX16(0xCAFEu, holdingValueAtAck);
  TEST_ASSERT_EQUAL_HEX16(0xCAFEu, holdingAtAck.snapshot.holding[0]);
  TEST_ASSERT_TRUE(holdingFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_EQUAL_HEX16(0xCAFEu, pending.snapshot.holding[0]);
  holdingFixture.holding[4] = 0x0BADu;
  TEST_ASSERT_TRUE(holdingFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_EQUAL_HEX16(0xCAFEu, pending.snapshot.holding[0]);
}

void test_multiple_write_ack_images_mutation_and_snapshots_are_stable() {
  Fixture coilFixture;
  const std::vector<bool> coilValues{true, false, true, true, false,
                                     false, true, false, true};
  ModbusRTUSlave::BridgeIngressEntry coilsAtAck;
  bool coilQueueVisibleAtAck = false;
  std::array<bool, 9> coilImageAtAck{};
  coilFixture.serial.onWrite = [&]() {
    coilQueueVisibleAtAck = coilFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, coilsAtAck);
    for (std::size_t i = 0; i < coilImageAtAck.size(); ++i) {
      coilImageAtAck[i] = coilFixture.coils[10 + i];
    }
  };
  coilFixture.transact(writeMultipleCoils(kUnitId, 10, coilValues));
  for (std::size_t i = 0; i < coilValues.size(); ++i) {
    TEST_ASSERT_EQUAL(coilValues[i], coilFixture.coils[10 + i]);
    TEST_ASSERT_EQUAL(coilValues[i], coilImageAtAck[i]);
  }
  TEST_ASSERT_TRUE(coilQueueVisibleAtAck);
  assertSingleTx(coilFixture, frame({kUnitId, 15, 0, 10, 0, 9}));
  ModbusRTUSlave::BridgeIngressEntry pending;
  TEST_ASSERT_TRUE(coilFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, pending));
  TEST_ASSERT_EQUAL_UINT8(coilValues.size(), pending.snapshotCount);
  for (std::size_t i = 0; i < coilValues.size(); ++i) {
    TEST_ASSERT_EQUAL(coilValues[i], pending.snapshot.coils[i]);
    TEST_ASSERT_EQUAL(coilValues[i], coilsAtAck.snapshot.coils[i]);
    coilFixture.coils[10 + i] = !coilValues[i];
  }
  TEST_ASSERT_TRUE(coilFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, pending));
  for (std::size_t i = 0; i < coilValues.size(); ++i) {
    TEST_ASSERT_EQUAL(coilValues[i], pending.snapshot.coils[i]);
  }

  Fixture holdingFixture;
  const std::vector<uint16_t> values{0x1111u, 0x2222u, 0x3333u};
  ModbusRTUSlave::BridgeIngressEntry holdingAtAck;
  bool holdingQueueVisibleAtAck = false;
  std::array<uint16_t, 3> holdingImageAtAck{};
  holdingFixture.serial.onWrite = [&]() {
    holdingQueueVisibleAtAck =
        holdingFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, holdingAtAck);
    for (std::size_t i = 0; i < holdingImageAtAck.size(); ++i) {
      holdingImageAtAck[i] = holdingFixture.holding[6 + i];
    }
  };
  holdingFixture.transact(writeMultipleHolding(kUnitId, 6, values));
  for (std::size_t i = 0; i < values.size(); ++i) {
    TEST_ASSERT_EQUAL_HEX16(values[i], holdingFixture.holding[6 + i]);
    TEST_ASSERT_EQUAL_HEX16(values[i], holdingImageAtAck[i]);
  }
  TEST_ASSERT_TRUE(holdingQueueVisibleAtAck);
  assertSingleTx(holdingFixture, frame({kUnitId, 16, 0, 6, 0, 3}));
  TEST_ASSERT_TRUE(holdingFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_EQUAL_UINT8(values.size(), pending.snapshotCount);
  TEST_ASSERT_EQUAL_UINT16_ARRAY(values.data(), pending.snapshot.holding,
                                 values.size());
  TEST_ASSERT_EQUAL_UINT16_ARRAY(values.data(), holdingAtAck.snapshot.holding,
                                 values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    holdingFixture.holding[6 + i] = static_cast<uint16_t>(0x9000u + i);
  }
  TEST_ASSERT_TRUE(holdingFixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_EQUAL_UINT16_ARRAY(values.data(), pending.snapshot.holding,
                                 values.size());
}

void test_exception_wire_images_do_not_mutate_or_enqueue() {
  Fixture fixture;
  fixture.coils[3] = false;
  fixture.transact(writeSingle(kUnitId, 5, 3, 0x1234u));
  TEST_ASSERT_FALSE(fixture.coils[3]);
  assertSingleTx(fixture, frame({kUnitId, static_cast<uint8_t>(0x80u | 5u), 3}));
  ModbusRTUSlave::BridgeIngressEntry pending;
  auto table = ModbusRTUSlave::BridgeIngressTable::Coils;
  TEST_ASSERT_FALSE(fixture.slave.bridgePeekNext(pending, table));
  fixture.finishTx();
  fixture.serial.writes.clear();

  fixture.transact(readRequest(kUnitId, 3, 127, 2));
  assertSingleTx(fixture, frame({kUnitId, static_cast<uint8_t>(0x80u | 3u), 2}));
  fixture.finishTx();
  fixture.serial.writes.clear();

  fixture.transact(frame({kUnitId, 0x44, 0, 0, 0, 1}));
  assertSingleTx(fixture, frame({kUnitId, 0xC4, 1}));
  TEST_ASSERT_EQUAL_UINT32(3u, fixture.events.size());
  TEST_ASSERT_EQUAL_UINT16(3u, fixture.events[0].code);
  TEST_ASSERT_EQUAL_UINT16(2u, fixture.events[1].code);
  TEST_ASSERT_EQUAL_UINT16(1u, fixture.events[2].code);
}

void test_broadcast_read_is_ignored_and_write_is_durable_without_reply() {
  Fixture fixture;
  fixture.holding[5] = 0x1111u;
  const auto read = readRequest(0, 3, 5, 1);
  fixture.transact(read);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.slave.debugInfo().ignored_broadcast_reads);
  TEST_ASSERT_EQUAL_UINT64(read.size() + 4u, fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(read.size(), fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(read.size() + 5u, ArduinoTest::microsCalls);

  const auto write = writeSingle(0, 6, 5, 0x2222u);
  fixture.transact(write);
  TEST_ASSERT_EQUAL_HEX16(0x2222u, fixture.holding[5]);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(read.size() + write.size() + 8u,
                           fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(read.size() + write.size(), fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(read.size() + write.size() + 10u,
                           ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.admissionCount);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.holdingMutex.lockCount);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.holdingMutex.unlockCount);
  ModbusRTUSlave::BridgeIngressEntry pending;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_FALSE((pending.attributes & ModbusRTUSlave::kBridgeIngressFlagFireForget) != 0u);
  TEST_ASSERT_FALSE((pending.attributes & ModbusRTUSlave::kBridgeIngressFlagResponseRequired) != 0u);
  TEST_ASSERT_EQUAL_UINT16(fixture.context, pending.context);
  fixture.holding[5] = 0x3333u;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_EQUAL_HEX16(0x2222u, pending.snapshot.holding[0]);
}

void test_denied_ordinary_broadcast_is_silent_and_nonmutating() {
  Fixture fixture;
  fixture.admit = false;
  fixture.context = 73U;
  fixture.holding[7] = 0x1111U;
  const auto request = writeSingle(0U, 6U, 7U, 0x2222U);

  fixture.transact(request);

  TEST_ASSERT_EQUAL_UINT64(0U, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_HEX16(0x1111U, fixture.holding[7]);
  TEST_ASSERT_EQUAL_UINT64(0U, fixture.holdingMutex.lockCount);
  TEST_ASSERT_EQUAL_UINT64(1U, fixture.admissionCount);
  ModbusRTUSlave::BridgeIngressEntry source;
  TEST_ASSERT_FALSE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, source));
  ModbusRTUSlave::BridgeIngressEntry dropped;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, dropped));
  TEST_ASSERT_EQUAL_UINT16(7U, dropped.start);
  TEST_ASSERT_EQUAL_UINT16(1U, dropped.count);
  TEST_ASSERT_EQUAL_UINT16(73U, dropped.context);
  TEST_ASSERT_EQUAL_UINT8(ModbusRTUSlave::kBridgeDropReasonAdmissionRejected,
                          dropped.attributes &
                              ModbusRTUSlave::kBridgeIngressReasonMask);
  TEST_ASSERT_FALSE(
      (dropped.attributes & ModbusRTUSlave::kBridgeIngressFlagFireForget) != 0U);
  TEST_ASSERT_FALSE(
      (dropped.attributes & ModbusRTUSlave::kBridgeIngressFlagResponseRequired) != 0U);
  TEST_ASSERT_EQUAL_UINT8(0U, dropped.snapshotCount);
}

void test_targeted_broadcast_filters_target_and_length_then_snapshots_fire_forget() {
  Fixture fixture;
  fixture.holding[8] = 0x1000u;

  fixture.transact(targetedWrite(8, 6, {0, 8, 0x20, 0x00}));
  TEST_ASSERT_EQUAL_HEX16(0x1000u, fixture.holding[8]);
  ModbusRTUSlave::BridgeIngressEntry pending;
  TEST_ASSERT_FALSE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));

  fixture.transact(targetedWrite(kUnitId, 15, {0, 8, 0, 9, 1, 0xFF}));
  TEST_ASSERT_FALSE(fixture.coils[8]);
  TEST_ASSERT_FALSE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, pending));

  fixture.transact(targetedWrite(kUnitId, 6, {0, 8, 0xBE, 0xEF}));
  TEST_ASSERT_EQUAL_HEX16(0xBEEFu, fixture.holding[8]);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_TRUE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_TRUE((pending.attributes & ModbusRTUSlave::kBridgeIngressFlagFireForget) != 0u);
  TEST_ASSERT_FALSE((pending.attributes & ModbusRTUSlave::kBridgeIngressFlagResponseRequired) != 0u);
  TEST_ASSERT_EQUAL_HEX16(0xBEEFu, pending.snapshot.holding[0]);
}

void test_targeted_broadcast_all_inner_writes_preserve_cross_table_order() {
  Fixture fixture;
  const std::vector<bool> coils{true, false, true, true, false,
                                false, true, false, true};
  fixture.transact(targetedWrite(kUnitId, 5, {0, 10, 0xFF, 0x00}));
  fixture.transact(targetedWrite(kUnitId, 6, {0, 10, 0x12, 0x34}));
  fixture.transact(targetedWrite(kUnitId, 15,
                                 {0, 20, 0, 9, 2, 0x4D, 0x01}));
  fixture.transact(targetedWrite(kUnitId, 16,
                                 {0, 20, 0, 3, 6,
                                  0x11, 0x11, 0x22, 0x22, 0x33, 0x33}));
  fixture.transact(targetedWrite(kUnitId, 5, {0, 11, 0xFF, 0x00}));
  fixture.transact(targetedWrite(kUnitId, 6, {0, 11, 0x56, 0x78}));
  fixture.transact(targetedWrite(kUnitId, 15,
                                 {0, 30, 0, 9, 2, 0xB2, 0x00}));
  fixture.transact(targetedWrite(kUnitId, 16,
                                 {0, 30, 0, 3, 6,
                                  0x44, 0x44, 0x55, 0x55, 0x66, 0x66}));

  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_TRUE(fixture.coils[10]);
  TEST_ASSERT_EQUAL_HEX16(0x1234u, fixture.holding[10]);
  for (std::size_t i = 0; i < coils.size(); ++i) {
    TEST_ASSERT_EQUAL(coils[i], fixture.coils[20 + i]);
  }
  TEST_ASSERT_EQUAL_HEX16(0x1111u, fixture.holding[20]);
  TEST_ASSERT_EQUAL_HEX16(0x2222u, fixture.holding[21]);
  TEST_ASSERT_EQUAL_HEX16(0x3333u, fixture.holding[22]);

  struct ExpectedSource {
    bool isCoil;
    uint16_t start;
    uint16_t token;
  };
  // The generator starts at 1, so the first visible token is 2. Together with
  // the rollover case (which pins token 1 after zero is skipped), this fixes
  // every token value 1..9 and the shared cross-table sequence.
  const std::array<ExpectedSource, 8> expected{{
      {true, 10u, 2u}, {false, 10u, 3u},
      {true, 20u, 4u}, {false, 20u, 5u},
      {true, 11u, 6u}, {false, 11u, 7u},
      {true, 30u, 8u}, {false, 30u, 9u}}};
  for (const auto& item : expected) {
    ModbusRTUSlave::BridgeIngressEntry pending;
    auto table = ModbusRTUSlave::BridgeIngressTable::Coils;
    TEST_ASSERT_TRUE(fixture.slave.bridgePeekNext(pending, table));
    TEST_ASSERT_EQUAL(
        item.isCoil,
        table == ModbusRTUSlave::BridgeIngressTable::Coils);
    TEST_ASSERT_EQUAL_UINT16(item.start, pending.start);
    TEST_ASSERT_EQUAL_UINT16(item.token, pending.sourceToken);
    TEST_ASSERT_EQUAL_UINT8(1u, pending.units);
    TEST_ASSERT_TRUE((pending.attributes & ModbusRTUSlave::kBridgeIngressFlagFireForget) != 0u);
    TEST_ASSERT_FALSE((pending.attributes & ModbusRTUSlave::kBridgeIngressFlagResponseRequired) != 0u);
    TEST_ASSERT_TRUE(fixture.slave.bridgeCommit(table, pending.sourceToken));
  }
  ModbusRTUSlave::BridgeIngressEntry empty;
  auto emptyTable = ModbusRTUSlave::BridgeIngressTable::Coils;
  TEST_ASSERT_FALSE(fixture.slave.bridgePeekNext(empty, emptyTable));
}

void test_unicast_targeted_broadcast_is_illegal_and_has_no_side_effect() {
  Fixture fixture;
  fixture.holding[2] = 0x0102u;
  fixture.transact(frame({kUnitId, 69, kUnitId, 6, 0, 2, 0xAB, 0xCD}));
  TEST_ASSERT_EQUAL_HEX16(0x0102u, fixture.holding[2]);
  assertSingleTx(fixture, frame({kUnitId, static_cast<uint8_t>(0x80u | 69u), 1}));
  ModbusRTUSlave::BridgeIngressEntry pending;
  auto table = ModbusRTUSlave::BridgeIngressTable::Coils;
  TEST_ASSERT_FALSE(fixture.slave.bridgePeekNext(pending, table));
}

void test_targeted_broadcast_denial_is_silent_nonmutating_and_records_ff_drop() {
  Fixture fixture;
  fixture.admit = false;
  fixture.context = 88u;
  fixture.holding[9] = 0x1111u;
  fixture.transact(targetedWrite(kUnitId, 6, {0, 9, 0x22, 0x22}));

  TEST_ASSERT_EQUAL_HEX16(0x1111u, fixture.holding[9]);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.holdingMutex.lockCount);
  ModbusRTUSlave::BridgeIngressEntry overflow;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, overflow));
  TEST_ASSERT_EQUAL_UINT8(ModbusRTUSlave::kBridgeDropReasonAdmissionRejected,
                          overflow.attributes & ModbusRTUSlave::kBridgeIngressReasonMask);
  TEST_ASSERT_TRUE((overflow.attributes & ModbusRTUSlave::kBridgeIngressFlagFireForget) != 0u);
  TEST_ASSERT_EQUAL_UINT16(88u, overflow.context);
}

void test_admission_precedes_mutation_snapshot_and_ack() {
  Fixture fixture;
  fixture.holding[2] = 0xAAAAu;
  bool queueVisibleAtWrite = false;
  uint16_t valueVisibleAtWrite = 0;
  ModbusRTUSlave::BridgeIngressEntry pendingAtWrite;
  fixture.serial.onWrite = [&]() {
    queueVisibleAtWrite = fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pendingAtWrite);
    valueVisibleAtWrite = fixture.holding[2];
  };

  fixture.transact(writeSingle(kUnitId, 6, 2, 0xBBBBu));
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.admissions.size());
  TEST_ASSERT_EQUAL_HEX16(0xAAAAu, fixture.admissions[0].observedHolding);
  TEST_ASSERT_TRUE(fixture.admissions[0].sequence < fixture.serial.lastWriteSequence);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.holdingMutex.lockCount);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.holdingMutex.unlockCount);
  TEST_ASSERT_TRUE(fixture.admissions[0].sequence < fixture.holdingMutex.lastLockSequence);
  TEST_ASSERT_TRUE(fixture.holdingMutex.lastLockSequence <
                   fixture.holdingMutex.lastUnlockSequence);
  TEST_ASSERT_TRUE(fixture.holdingMutex.lastUnlockSequence <
                   fixture.serial.lastWriteSequence);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.admissions[0].sequence);
  TEST_ASSERT_EQUAL_UINT64(4u, fixture.holdingMutex.lastLockSequence);
  TEST_ASSERT_EQUAL_UINT64(7u, fixture.holdingMutex.lastUnlockSequence);
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.appliedWrites.size());
  TEST_ASSERT_EQUAL_UINT64(8u, fixture.appliedWrites[0].sequence);
  TEST_ASSERT_FALSE(fixture.appliedWrites[0].isLocal);
  TEST_ASSERT_EQUAL_UINT32(1u, ArduinoTest::pinWrites.size());
  TEST_ASSERT_EQUAL_UINT8(HIGH, ArduinoTest::pinWrites[0].value);
  TEST_ASSERT_EQUAL_UINT64(9u, ArduinoTest::pinWrites[0].sequence);
  TEST_ASSERT_EQUAL_UINT64(10u, fixture.serial.lastWriteSequence);
  TEST_ASSERT_TRUE(queueVisibleAtWrite);
  TEST_ASSERT_EQUAL_HEX16(0xBBBBu, valueVisibleAtWrite);
  assertIngressHeader(
      pendingAtWrite, 2u, 1u, fixture.context, 2u,
      ModbusRTUSlave::kBridgeIngressFlagResponseRequired);
  TEST_ASSERT_EQUAL_HEX16(0xBBBBu, pendingAtWrite.snapshot.holding[0]);
  assertSingleTx(fixture, writeSingle(kUnitId, 6, 2, 0xBBBBu));

  // The transfer payload is an immutable admission-time image, not a later
  // view of the shared holding-register table.
  fixture.holding[2] = 0xCCCCu;
  ModbusRTUSlave::BridgeIngressEntry durable;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, durable));
  TEST_ASSERT_EQUAL_UINT16(pendingAtWrite.sourceToken, durable.sourceToken);
  TEST_ASSERT_EQUAL_HEX16(0xBBBBu, durable.snapshot.holding[0]);
}

void test_admission_denial_prevents_mutation_and_returns_device_failure() {
  Fixture fixture;
  fixture.admit = false;
  fixture.context = 77u;
  fixture.holding[4] = 0x1111u;
  fixture.transact(writeSingle(kUnitId, 6, 4, 0x2222u));

  TEST_ASSERT_EQUAL_HEX16(0x1111u, fixture.holding[4]);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.holdingMutex.lockCount);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.holdingMutex.unlockCount);
  assertSingleTx(fixture, frame({kUnitId, static_cast<uint8_t>(0x80u | 6u), 4}));
  ModbusRTUSlave::BridgeIngressEntry pending;
  TEST_ASSERT_FALSE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_TRUE(fixture.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_EQUAL_UINT8(ModbusRTUSlave::kBridgeDropReasonAdmissionRejected,
                          pending.attributes & ModbusRTUSlave::kBridgeIngressReasonMask);
  TEST_ASSERT_EQUAL_UINT16(77u, pending.context);
  TEST_ASSERT_EQUAL_UINT8(0u, pending.snapshotCount);
  TEST_ASSERT_EQUAL_UINT32(2u, fixture.events.size());
  TEST_ASSERT_EQUAL_HEX16(ModbusRTUSlave::kEventBridgeAdmissionRejected,
                          fixture.events[0].code);
  TEST_ASSERT_EQUAL_UINT16(1u, fixture.events[0].units);
  TEST_ASSERT_EQUAL_UINT16(4u, fixture.events[1].code);
  TEST_ASSERT_TRUE(fixture.events[0].sequence < fixture.serial.lastWriteSequence);
  TEST_ASSERT_TRUE(fixture.events[1].sequence > fixture.serial.lastWriteSequence);
}

void test_local_write_bypasses_queue_and_notifies_before_ack() {
  Fixture fixture;
  fixture.localStart = 12u;
  fixture.transact(writeSingle(kUnitId, 5, 12, 0xFF00u));

  TEST_ASSERT_TRUE(fixture.coils[12]);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.admissionCount);
  TEST_ASSERT_EQUAL_UINT32(1u, fixture.appliedWrites.size());
  TEST_ASSERT_TRUE(fixture.appliedWrites[0].isLocal);
  TEST_ASSERT_TRUE(fixture.appliedWrites[0].sequence < fixture.serial.lastWriteSequence);
  ModbusRTUSlave::BridgeIngressEntry pending;
  TEST_ASSERT_FALSE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, pending));
  assertSingleTx(fixture, writeSingle(kUnitId, 5, 12, 0xFF00u));
}

void test_bridge_defaults_admit_without_product_policy_or_reserved_coils() {
  Fixture fixture;
  ModbusRTUSlave::setBridgeAdmissionFn(nullptr);

  fixture.transact(writeSingle(kUnitId, 5, 0, 0xFF00u));

  TEST_ASSERT_TRUE(fixture.coils[0]);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.admissionCount);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.appliedWriteCount);
  assertSingleTx(fixture, writeSingle(kUnitId, 5, 0, 0xFF00u));
  ModbusRTUSlave::BridgeIngressEntry entry;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::Coils, entry));
  assertIngressHeader(
      entry, 0u, 1u, 0u, 2u,
      ModbusRTUSlave::kBridgeIngressFlagResponseRequired);
  TEST_ASSERT_TRUE(entry.snapshot.coils[0]);
}

void test_combined_source_queue_preserves_cross_table_order_and_exact_commit() {
  Fixture fixture;
  fixture.transact(writeSingle(kUnitId, 6, 1, 0x1111u));
  fixture.finishTx();
  fixture.transact(writeSingle(kUnitId, 5, 2, 0xFF00u));
  fixture.finishTx();
  fixture.transact(writeSingle(kUnitId, 6, 3, 0x3333u));

  // A later producer-side table update must not perturb any queued transfer
  // image. The source token, metadata and snapshot sequence below are the
  // golden handoff contract used by the bridge runtime.
  fixture.holding[1] = 0xA001u;
  fixture.coils[2] = false;
  fixture.holding[3] = 0xA003u;

  ModbusRTUSlave::BridgeIngressEntry pending;
  auto table = ModbusRTUSlave::BridgeIngressTable::Coils;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeekNext(pending, table));
  TEST_ASSERT_EQUAL(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters,
                    table);
  assertIngressHeader(
      pending, 1u, 1u, fixture.context, 2u,
      ModbusRTUSlave::kBridgeIngressFlagResponseRequired);
  TEST_ASSERT_EQUAL_HEX16(0x1111u, pending.snapshot.holding[0]);
  const uint16_t firstToken = pending.sourceToken;
  TEST_ASSERT_FALSE(fixture.slave.bridgeCommit(
      ModbusRTUSlave::BridgeIngressTable::HoldingRegisters,
      firstToken + 1u));
  ModbusRTUSlave::BridgeIngressEntry unchanged;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeekNext(unchanged, table));
  TEST_ASSERT_EQUAL_UINT16(firstToken, unchanged.sourceToken);
  TEST_ASSERT_EQUAL_HEX16(0x1111u, unchanged.snapshot.holding[0]);
  TEST_ASSERT_TRUE(fixture.slave.bridgeCommit(table, firstToken));

  TEST_ASSERT_TRUE(fixture.slave.bridgePeekNext(pending, table));
  TEST_ASSERT_EQUAL(ModbusRTUSlave::BridgeIngressTable::Coils, table);
  assertIngressHeader(
      pending, 2u, 1u, fixture.context, 3u,
      ModbusRTUSlave::kBridgeIngressFlagResponseRequired);
  TEST_ASSERT_TRUE(pending.snapshot.coils[0]);
  TEST_ASSERT_TRUE(fixture.slave.bridgeCommit(table, pending.sourceToken));

  TEST_ASSERT_TRUE(fixture.slave.bridgePeekNext(pending, table));
  TEST_ASSERT_EQUAL(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters,
                    table);
  assertIngressHeader(
      pending, 3u, 1u, fixture.context, 4u,
      ModbusRTUSlave::kBridgeIngressFlagResponseRequired);
  TEST_ASSERT_EQUAL_HEX16(0x3333u, pending.snapshot.holding[0]);
  TEST_ASSERT_TRUE(fixture.slave.bridgeCommit(table, pending.sourceToken));
  TEST_ASSERT_FALSE(fixture.slave.bridgePeekNext(pending, table));
}

void test_large_write_chunks_snapshots_without_reordering_or_flag_duplication() {
  Fixture fixture;
  std::vector<uint16_t> values(33u);
  for (uint16_t i = 0; i < values.size(); ++i) values[i] = 0x4000u + i;
  fixture.transact(writeMultipleHolding(kUnitId, 10, values));

  ModbusRTUSlave::BridgeIngressEntry first;
  ModbusRTUSlave::BridgeIngressEntry second;
  auto table = ModbusRTUSlave::BridgeIngressTable::Coils;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeekNext(first, table));
  TEST_ASSERT_EQUAL(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters,
                    table);
  TEST_ASSERT_EQUAL_UINT16(10u, first.start);
  TEST_ASSERT_EQUAL_UINT16(32u, first.count);
  TEST_ASSERT_EQUAL_UINT8(32u, first.snapshotCount);
  TEST_ASSERT_EQUAL_UINT8(1u, first.units);
  TEST_ASSERT_TRUE((first.attributes & ModbusRTUSlave::kBridgeIngressFlagResponseRequired) != 0u);
  TEST_ASSERT_TRUE(fixture.slave.bridgeCommit(table, first.sourceToken));

  TEST_ASSERT_TRUE(fixture.slave.bridgePeekNext(second, table));
  TEST_ASSERT_EQUAL_UINT16(42u, second.start);
  TEST_ASSERT_EQUAL_UINT16(1u, second.count);
  TEST_ASSERT_EQUAL_UINT8(1u, second.snapshotCount);
  TEST_ASSERT_EQUAL_UINT16(first.sourceToken + 1u, second.sourceToken);
  TEST_ASSERT_EQUAL_HEX16(values[32], second.snapshot.holding[0]);
}

void test_source_queue_saturation_rejects_before_mutation_and_records_overflow() {
  Fixture fixture;
  for (uint16_t i = 0; i < kBridgeQueueSize - 1u; ++i) {
    fixture.transact(writeSingle(0, 6, 20, static_cast<uint16_t>(0x1000u + i)));
  }
  const uint16_t acceptedValue = fixture.holding[20];
  bool overflowVisibleAtReply = false;
  uint16_t valueVisibleAtReply = 0;
  fixture.serial.onWrite = [&]() {
    ModbusRTUSlave::BridgeIngressEntry overflow;
    overflowVisibleAtReply = fixture.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, overflow);
    valueVisibleAtReply = fixture.holding[20];
  };
  fixture.transact(writeSingle(kUnitId, 6, 20, 0xFFFFu));

  TEST_ASSERT_EQUAL_HEX16(acceptedValue, fixture.holding[20]);
  TEST_ASSERT_TRUE(overflowVisibleAtReply);
  TEST_ASSERT_EQUAL_HEX16(acceptedValue, valueVisibleAtReply);
  assertSingleTx(fixture, frame({kUnitId, static_cast<uint8_t>(0x80u | 6u), 4}));
  ModbusRTUSlave::BridgeIngressEntry overflow;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, overflow));
  TEST_ASSERT_EQUAL_UINT16(20u, overflow.start);
  TEST_ASSERT_EQUAL_UINT8(ModbusRTUSlave::kBridgeDropReasonOverflow,
                          overflow.attributes & ModbusRTUSlave::kBridgeIngressReasonMask);
  TEST_ASSERT_EQUAL_UINT8(0u, overflow.snapshotCount);
  TEST_ASSERT_EQUAL_UINT32(2u, fixture.events.size());
  TEST_ASSERT_EQUAL_HEX16(ModbusRTUSlave::kEventBridgeOverflow,
                          fixture.events[0].code);
  TEST_ASSERT_EQUAL_UINT16(1u, fixture.events[0].units);
  TEST_ASSERT_EQUAL_UINT16(4u, fixture.events[1].code);
  TEST_ASSERT_TRUE(fixture.events[0].sequence < fixture.serial.lastWriteSequence);
  TEST_ASSERT_TRUE(fixture.events[1].sequence > fixture.serial.lastWriteSequence);

  uint16_t previousToken = 0;
  uint16_t queued = 0;
  ModbusRTUSlave::BridgeIngressEntry pending;
  while (fixture.slave.bridgePeek(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending)) {
    TEST_ASSERT_TRUE(previousToken == 0u || pending.sourceToken > previousToken);
    previousToken = pending.sourceToken;
    TEST_ASSERT_TRUE(fixture.slave.bridgeCommit(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending.sourceToken));
    ++queued;
  }
  TEST_ASSERT_EQUAL_UINT16(kBridgeQueueSize - 1u, queued);
}

void test_multi_chunk_capacity_is_reserved_before_any_mutation() {
  Fixture fixture;
  for (uint16_t i = 0; i < kBridgeQueueSize - 2u; ++i) {
    fixture.transact(writeSingle(0, 6, 1, static_cast<uint16_t>(0x2000u + i)));
  }
  std::vector<uint16_t> values(33u, 0xBBBBu);
  for (uint16_t i = 0; i < values.size(); ++i) fixture.holding[40u + i] = 0xAAAAu;

  fixture.transact(writeMultipleHolding(kUnitId, 40, values));

  for (uint16_t i = 0; i < values.size(); ++i) {
    TEST_ASSERT_EQUAL_HEX16(0xAAAAu, fixture.holding[40u + i]);
  }
  assertSingleTx(fixture, frame({kUnitId, static_cast<uint8_t>(0x80u | 16u), 4}));
  ModbusRTUSlave::BridgeIngressEntry overflow;
  TEST_ASSERT_TRUE(fixture.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, overflow));
  TEST_ASSERT_EQUAL_UINT16(40u, overflow.start);
  TEST_ASSERT_EQUAL_UINT16(33u, overflow.count);
  TEST_ASSERT_EQUAL_UINT8(2u, overflow.units);
  TEST_ASSERT_EQUAL_UINT8(0u, overflow.snapshotCount);
}

void test_overflow_tokens_and_newest_retention_are_stable() {
  Fixture ordered;
  ordered.admit = false;
  ordered.transact(targetedWrite(kUnitId, 5, {0, 70, 0xFF, 0x00}));
  ordered.transact(targetedWrite(kUnitId, 6, {0, 71, 0x12, 0x34}));
  ordered.transact(targetedWrite(kUnitId, 15, {0, 72, 0, 2, 1, 0x03}));

  ModbusRTUSlave::BridgeIngressEntry coil;
  ModbusRTUSlave::BridgeIngressEntry holding;
  TEST_ASSERT_TRUE(ordered.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::Coils, coil));
  TEST_ASSERT_TRUE(ordered.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, holding));
  TEST_ASSERT_EQUAL_UINT16(2u, coil.sourceToken);
  TEST_ASSERT_EQUAL_UINT16(3u, holding.sourceToken);
  TEST_ASSERT_EQUAL_UINT16(70u, coil.start);
  TEST_ASSERT_EQUAL_UINT16(71u, holding.start);
  TEST_ASSERT_EQUAL_UINT8(
      ModbusRTUSlave::kBridgeIngressFlagFireForget |
          ModbusRTUSlave::kBridgeDropReasonAdmissionRejected,
      coil.attributes);
  TEST_ASSERT_EQUAL_UINT8(coil.attributes, holding.attributes);
  TEST_ASSERT_EQUAL_UINT8(0u, coil.snapshotCount);
  TEST_ASSERT_EQUAL_UINT8(0u, holding.snapshotCount);
  TEST_ASSERT_TRUE(
      ordered.slave.bridgeCommitDrop(ModbusRTUSlave::BridgeIngressTable::Coils, coil.sourceToken));
  TEST_ASSERT_TRUE(ordered.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::Coils, coil));
  TEST_ASSERT_EQUAL_UINT16(4u, coil.sourceToken);
  TEST_ASSERT_EQUAL_UINT16(72u, coil.start);

  Fixture saturated;
  saturated.admit = false;
  constexpr uint16_t kRejected = kBridgeOverflowQueueSize + 3u;
  for (uint16_t i = 0; i < kRejected; ++i) {
    saturated.transact(targetedWrite(
        kUnitId, 6,
        {highByte(i), lowByte(i), highByte(0xD000u + i),
         lowByte(0xD000u + i)}));
  }
  TEST_ASSERT_EQUAL_UINT64(kRejected, saturated.admissionCount);
  TEST_ASSERT_EQUAL_UINT64(0u, saturated.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, saturated.holdingMutex.lockCount);

  constexpr uint16_t kRetained = kBridgeOverflowQueueSize - 1u;
  constexpr uint16_t kFirstRetained = kRejected - kRetained;
  for (uint16_t i = kFirstRetained; i < kRejected; ++i) {
    ModbusRTUSlave::BridgeIngressEntry retained;
    TEST_ASSERT_TRUE(saturated.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, retained));
    TEST_ASSERT_EQUAL_UINT16(i, retained.start);
    TEST_ASSERT_EQUAL_UINT16(1u, retained.count);
    TEST_ASSERT_EQUAL_UINT8(1u, retained.units);
    TEST_ASSERT_EQUAL_UINT16(saturated.context,
                             retained.context);
    TEST_ASSERT_EQUAL_UINT16(i + 2u, retained.sourceToken);
    TEST_ASSERT_EQUAL_UINT8(
        ModbusRTUSlave::kBridgeIngressFlagFireForget |
            ModbusRTUSlave::kBridgeDropReasonAdmissionRejected,
        retained.attributes);
    TEST_ASSERT_EQUAL_UINT8(0u, retained.snapshotCount);
    TEST_ASSERT_TRUE(saturated.slave.bridgeCommitDrop(
        ModbusRTUSlave::BridgeIngressTable::HoldingRegisters,
        retained.sourceToken));
  }
  ModbusRTUSlave::BridgeIngressEntry empty;
  TEST_ASSERT_FALSE(saturated.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, empty));
}

void test_source_and_overflow_tokens_skip_zero_at_uint16_rollover() {
  // Exercise the production token generators rather than only the standalone
  // source-order policy. Keep each churn entry committed so ring capacity does
  // not obscure the shared sequence rollover being characterized.
  Fixture source;
  source.benchmarkMode = true;
  source.serial.captureWrites = false;
  const auto sourceChurn =
      targetedWrite(kUnitId, 6, {0, 90, 0x12, 0x34});
  ModbusRTUSlave::BridgeIngressEntry pending;
  auto table = ModbusRTUSlave::BridgeIngressTable::Coils;
  constexpr uint32_t kAdvanceToBeforeWrap = 65533U;
  for (uint32_t index = 0U; index < kAdvanceToBeforeWrap; ++index) {
    source.transact(sourceChurn);
    TEST_ASSERT_TRUE(source.slave.bridgePeekNext(pending, table));
    TEST_ASSERT_EQUAL(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters,
                      table);
    TEST_ASSERT_NOT_EQUAL(0U, pending.sourceToken);
    TEST_ASSERT_TRUE(
        source.slave.bridgeCommit(table, pending.sourceToken));
  }

  source.transact(targetedWrite(kUnitId, 5, {0, 91, 0xFF, 0x00}));
  source.transact(targetedWrite(kUnitId, 6, {0, 92, 0xBE, 0xEF}));
  TEST_ASSERT_TRUE(source.slave.bridgePeekNext(pending, table));
  TEST_ASSERT_EQUAL(ModbusRTUSlave::BridgeIngressTable::Coils, table);
  TEST_ASSERT_EQUAL_UINT16(0xFFFFU, pending.sourceToken);
  TEST_ASSERT_TRUE(source.slave.bridgeCommit(table, pending.sourceToken));
  TEST_ASSERT_TRUE(source.slave.bridgePeekNext(pending, table));
  TEST_ASSERT_EQUAL(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters,
                    table);
  TEST_ASSERT_EQUAL_UINT16(1U, pending.sourceToken);
  TEST_ASSERT_TRUE(source.slave.bridgeCommit(table, pending.sourceToken));
  TEST_ASSERT_FALSE(source.slave.bridgePeekNext(pending, table));

  Fixture dropped;
  dropped.admit = false;
  dropped.benchmarkMode = true;
  dropped.serial.captureWrites = false;
  const auto dropChurn =
      targetedWrite(kUnitId, 6, {0, 93, 0x45, 0x67});
  for (uint32_t index = 0U; index < kAdvanceToBeforeWrap; ++index) {
    dropped.transact(dropChurn);
    TEST_ASSERT_TRUE(dropped.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
    TEST_ASSERT_NOT_EQUAL(0U, pending.sourceToken);
    TEST_ASSERT_TRUE(
        dropped.slave.bridgeCommitDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending.sourceToken));
  }

  dropped.transact(targetedWrite(kUnitId, 5, {0, 94, 0xFF, 0x00}));
  dropped.transact(targetedWrite(kUnitId, 6, {0, 95, 0x89, 0xAB}));
  TEST_ASSERT_TRUE(dropped.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::Coils, pending));
  TEST_ASSERT_EQUAL_UINT16(0xFFFFU, pending.sourceToken);
  TEST_ASSERT_TRUE(
      dropped.slave.bridgeCommitDrop(ModbusRTUSlave::BridgeIngressTable::Coils, pending.sourceToken));
  TEST_ASSERT_TRUE(dropped.slave.bridgePeekDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending));
  TEST_ASSERT_EQUAL_UINT16(1U, pending.sourceToken);
  TEST_ASSERT_TRUE(
      dropped.slave.bridgeCommitDrop(ModbusRTUSlave::BridgeIngressTable::HoldingRegisters, pending.sourceToken));
}

void test_poll_and_tx_pump_operation_counts_are_exact() {
  Fixture fixture;

  fixture.slave.poll();
  TEST_ASSERT_EQUAL_UINT64(2u, fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(1u, ArduinoTest::microsCalls);

  fixture.serial.clearTrace();
  ArduinoTest::clearTrace();
  const auto request =
      writeMultipleHolding(kUnitId, 20, {0x1111u, 0x2222u});
  const auto acknowledgement = frame({kUnitId, 16, 0, 20, 0, 2});

  fixture.startFrame(request);
  TEST_ASSERT_EQUAL_UINT64(request.size() + 2u,
                           fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(request.size(), fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(request.size() + 2u, ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_TRUE(fixture.slave.workState().rxInFrame());

  fixture.finishFrame();
  TEST_ASSERT_EQUAL_UINT64(request.size() + 4u,
                           fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(request.size(), fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(request.size() + 6u, ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.flushCalls);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.admissionCount);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.holdingMutex.lockCount);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.holdingMutex.unlockCount);
  TEST_ASSERT_EQUAL_UINT32(1u, ArduinoTest::pinWrites.size());
  TEST_ASSERT_EQUAL_UINT8(HIGH, ArduinoTest::pinWrites[0].value);
  TEST_ASSERT_EQUAL_UINT64(0u, ArduinoTest::millisCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, ArduinoTest::delayMicrosecondsCalls);
  assertSingleTx(fixture, acknowledgement);

  const uint32_t duration =
      static_cast<uint32_t>(acknowledgement.size()) *
      kTxBudgetPerByteUs8N1At19200;
  fixture.slave.tx_pump();
  TEST_ASSERT_EQUAL_UINT64(request.size() + 7u, ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.flushCalls);
  TEST_ASSERT_TRUE(fixture.slave.workState().txBusy());

  ArduinoTest::advance(duration - 1u);
  fixture.slave.tx_pump();
  TEST_ASSERT_EQUAL_UINT64(request.size() + 8u, ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.flushCalls);
  TEST_ASSERT_TRUE(fixture.slave.workState().txBusy());

  ArduinoTest::advance(1u);
  fixture.slave.tx_pump();
  TEST_ASSERT_EQUAL_UINT64(request.size() + 10u, ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.serial.flushCalls);
  TEST_ASSERT_EQUAL_UINT32(2u, ArduinoTest::pinWrites.size());
  TEST_ASSERT_EQUAL_UINT8(LOW, ArduinoTest::pinWrites[1].value);
  TEST_ASSERT_FALSE(fixture.slave.workState().txBusy());
  TEST_ASSERT_EQUAL_UINT64(request.size() + 4u,
                           fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(request.size(), fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, ArduinoTest::delayMicrosecondsCalls);
}

void test_tx_pump_due_boundary_preserves_nonblocking_de_order() {
  Fixture fixture;
  fixture.transact(writeSingle(kUnitId, 6, 1, 0x1234u));
  TEST_ASSERT_EQUAL_UINT32(1u, ArduinoTest::pinWrites.size());
  TEST_ASSERT_EQUAL_UINT8(HIGH, ArduinoTest::pinWrites.back().value);
  TEST_ASSERT_TRUE(ArduinoTest::pinWrites.back().sequence <
                   fixture.serial.lastWriteSequence);

  fixture.slave.tx_pump();
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.flushCalls);
  TEST_ASSERT_TRUE(fixture.slave.workState().txBusy());
  const uint32_t duration = 8u * kTxBudgetPerByteUs8N1At19200;
  ArduinoTest::advance(duration - 1u);
  fixture.slave.tx_pump();
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.flushCalls);
  TEST_ASSERT_TRUE(fixture.slave.workState().txBusy());

  ArduinoTest::advance(1u);
  fixture.slave.tx_pump();
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.serial.flushCalls);
  TEST_ASSERT_FALSE(fixture.slave.workState().txBusy());
  TEST_ASSERT_EQUAL_UINT8(LOW, ArduinoTest::pinWrites.back().value);
  TEST_ASSERT_TRUE(fixture.serial.lastFlushSequence <
                   ArduinoTest::pinWrites.back().sequence);
  TEST_ASSERT_EQUAL_UINT64(0u, ArduinoTest::delayMicrosecondsCalls);
  TEST_ASSERT_EQUAL_UINT32(duration, fixture.slave.debugInfo().last_tx_busy_us);
}

void test_frame_and_tx_deadlines_are_rollover_safe() {
  Fixture fixture;
  ArduinoTest::nowUs = UINT32_MAX - 3000u;
  fixture.holding[1] = 0xAAAAu;
  fixture.startFrame(readRequest(kUnitId, 3, 1, 1));
  fixture.finishFrame();
  TEST_ASSERT_EQUAL_UINT64(1u, fixture.serial.writeCalls);
  fixture.slave.tx_pump();
  const uint32_t duration = static_cast<uint32_t>(fixture.serial.lastWriteLen) *
                            kTxBudgetPerByteUs8N1At19200;
  ArduinoTest::advance(duration - 1u);
  fixture.slave.tx_pump();
  TEST_ASSERT_TRUE(fixture.slave.workState().txBusy());
  ArduinoTest::advance(1u);
  fixture.slave.tx_pump();
  TEST_ASSERT_FALSE(fixture.slave.workState().txBusy());
}

void test_work_pending_tracks_serial_rx_frame_and_tx_lifecycle() {
  Fixture fixture;
  TEST_ASSERT_FALSE(fixture.slave.hasWorkPending());
  const auto request = readRequest(kUnitId, 3, 0, 1);
  fixture.serial.inject(request);
  TEST_ASSERT_TRUE(fixture.slave.hasWorkPending());
  fixture.slave.poll();
  TEST_ASSERT_TRUE(fixture.slave.workState().rxInFrame());
  fixture.finishFrame();
  TEST_ASSERT_FALSE(fixture.slave.workState().rxInFrame());
  TEST_ASSERT_TRUE(fixture.slave.workState().txBusy());
  fixture.finishTx();
  TEST_ASSERT_FALSE(fixture.slave.hasWorkPending());
}

void test_upstream_tx_diagnostics_preserve_queue_pump_done_order_and_buckets() {
  Fixture fixture;
  fixture.transact(writeMultipleHolding(kUnitId, 2, {1u, 2u, 3u}));
  ArduinoTest::advance(6000u);
  fixture.slave.tx_pump();
  ArduinoTest::advance(8u * kTxBudgetPerByteUs8N1At19200);
  fixture.slave.tx_pump();

  ModbusRTUSlave::BridgeUpstreamTxDiagSnapshot snapshot;
  fixture.slave.copyAndResetBridgeUpstreamTxDiag(snapshot);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.accepted);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.queued);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.txPumpSeen);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.txDone);
  TEST_ASSERT_EQUAL_UINT32(6u, snapshot.pumpMsMax);
  TEST_ASSERT_EQUAL_UINT32(8u, snapshot.doneMsMax);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.pumpOverMs[0]);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.pumpOverMs[1]);
  TEST_ASSERT_EQUAL_UINT32(0u, snapshot.pumpOverMs[2]);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.doneOverMs[0]);
  TEST_ASSERT_EQUAL_UINT32(0u, snapshot.doneOverMs[1]);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.bucketAccepted[3][1]);
  TEST_ASSERT_EQUAL_UINT32(1u, snapshot.bucketQueued[3][1]);

  ModbusRTUSlave::BridgeUpstreamTxDiagSnapshot cleared;
  fixture.slave.copyAndResetBridgeUpstreamTxDiag(cleared);
  TEST_ASSERT_EQUAL_UINT32(0u, cleared.accepted);
  TEST_ASSERT_EQUAL_UINT32(0u, cleared.txDone);
}

void test_idle_poll_and_request_throughput_stay_within_characterized_budgets() {
  Fixture fixture(NO_DE_PIN);
  constexpr uint32_t kSamples = 5u;
  // One 100k idle sample is only about 1.6ms on the characterization host.
  // Lengthen only this very cheap lane so scheduler/build activity cannot
  // dominate three of five samples and produce a false strict-gate failure.
  constexpr uint32_t kIdlePolls = 1000000u;
  std::array<int64_t, kSamples> idleSamples{};
  for (uint32_t sample = 0; sample < kSamples; ++sample) {
    const auto started = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < kIdlePolls; ++i) fixture.slave.poll();
    idleSamples[sample] = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - started).count();
  }
  std::sort(idleSamples.begin(), idleSamples.end());

  // Current MBUS_RTU_SLAVE_DIAGNOSTICS path samples availability once for ingestion and once
  // for overflow accounting. Pin the two-call idle budget explicitly.
  TEST_ASSERT_EQUAL_UINT64(kSamples * kIdlePolls * 2u,
                           fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kIdlePolls, ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, fixture.serial.flushCalls);
  TEST_ASSERT_EQUAL_UINT64(0u, ArduinoTest::delayMicrosecondsCalls);
  TEST_ASSERT_TRUE(idleSamples[kSamples / 2u] <
                   std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::seconds(5)).count());

  fixture.serial.clearTrace();
  ArduinoTest::clearTrace();
  fixture.allLocal = true;
  fixture.benchmarkMode = true;
  fixture.serial.captureWrites = false;
  const auto request = writeSingle(kUnitId, 6, 1, 0x1234u);
  constexpr uint32_t kRequests = 100000u;
  std::array<int64_t, kSamples> requestSamples{};
  for (uint32_t sample = 0; sample < kSamples; ++sample) {
    const auto started = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < kRequests; ++i) {
      fixture.transact(request);
      fixture.slave.tx_pump();
      ArduinoTest::advance(8u * kTxBudgetPerByteUs8N1At19200);
      fixture.slave.tx_pump();
      // Mirror the normal service loop's following idle pump, which resets the
      // per-frame TX edge state before another request is accepted.
      fixture.slave.tx_pump();
    }
    requestSamples[sample] =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - started).count();
  }
  std::sort(requestSamples.begin(), requestSamples.end());
  TEST_ASSERT_EQUAL_UINT64(kSamples * kRequests, fixture.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kRequests, fixture.serial.flushCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kRequests * request.size(),
                           fixture.serial.readCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kRequests * 12u,
                           fixture.serial.availableCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kRequests * 17u,
                           ArduinoTest::microsCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kRequests, fixture.appliedWriteCount);
  TEST_ASSERT_EQUAL_UINT64(0u, ArduinoTest::delayMicrosecondsCalls);
  TEST_ASSERT_TRUE(requestSamples[kSamples / 2u] <
                   std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::seconds(5)).count());

  // Preserve the originally calibrated FC6 hot loop exactly.
  Fixture forwarded(NO_DE_PIN);
  forwarded.benchmarkMode = true;
  forwarded.serial.captureWrites = false;
  const auto forwardedRequest = writeSingle(kUnitId, 6, 1, 0x5678u);
  constexpr uint32_t kForwardedRequests = 100000u;
  std::array<int64_t, kSamples> forwardedSamples{};
  uint64_t forwardedTokenChecksum = 0u;
  uint32_t forwardedDrainFailures = 0u;
  for (uint32_t sample = 0; sample < kSamples; ++sample) {
    const auto started = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < kForwardedRequests; ++i) {
      forwarded.transact(forwardedRequest);
      ModbusRTUSlave::BridgeIngressEntry pending;
      auto table = ModbusRTUSlave::BridgeIngressTable::Coils;
      if (!forwarded.slave.bridgePeekNext(pending, table) ||
          table == ModbusRTUSlave::BridgeIngressTable::Coils) {
        ++forwardedDrainFailures;
      } else {
        forwardedTokenChecksum += pending.sourceToken;
        if (!forwarded.slave.bridgeCommit(table, pending.sourceToken)) {
          ++forwardedDrainFailures;
        }
      }
      forwarded.slave.tx_pump();
      ArduinoTest::advance(8u * kTxBudgetPerByteUs8N1At19200);
      forwarded.slave.tx_pump();
      forwarded.slave.tx_pump();
    }
    forwardedSamples[sample] =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - started).count();
  }
  std::sort(forwardedSamples.begin(), forwardedSamples.end());
  TEST_ASSERT_EQUAL_UINT32(0u, forwardedDrainFailures);
  TEST_ASSERT_NOT_EQUAL(0u, forwardedTokenChecksum);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedRequests,
                           forwarded.admissionCount);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedRequests,
                           forwarded.appliedWriteCount);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedRequests,
                           forwarded.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedRequests,
                           forwarded.serial.flushCalls);
  ModbusRTUSlave::BridgeIngressEntry noPending;
  auto noPendingTable = ModbusRTUSlave::BridgeIngressTable::Coils;
  TEST_ASSERT_FALSE(
      forwarded.slave.bridgePeekNext(noPending, noPendingTable));
  TEST_ASSERT_TRUE(forwardedSamples[kSamples / 2u] <
                   std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::seconds(5)).count());

  // The bulk helper adds exact per-chunk order checks. Keep it out of the FC6
  // lane so expanding coverage cannot perturb that established benchmark.
  auto measureForwarded = [](Fixture& lane,
                             const std::vector<uint8_t>& laneRequest,
                             uint16_t expectedStart,
                             uint16_t expectedCount,
                             uint8_t expectedChunks,
                             uint32_t operations,
                             std::array<int64_t, kSamples>& samples,
                             uint64_t& tokenChecksum,
                             uint32_t& failures) {
    lane.benchmarkMode = true;
    lane.serial.captureWrites = false;
    for (uint32_t sample = 0; sample < kSamples; ++sample) {
      const auto started = std::chrono::steady_clock::now();
      for (uint32_t i = 0; i < operations; ++i) {
        lane.transact(laneRequest);
        for (uint8_t chunk = 0; chunk < expectedChunks; ++chunk) {
          ModbusRTUSlave::BridgeIngressEntry pending;
          auto table = ModbusRTUSlave::BridgeIngressTable::Coils;
          if (!lane.slave.bridgePeekNext(pending, table)) {
            ++failures;
            continue;
          }
          const uint16_t chunkOffset = static_cast<uint16_t>(chunk) * 32u;
          const uint16_t remaining =
              static_cast<uint16_t>(expectedCount - chunkOffset);
          const uint16_t chunkCount = remaining > 32u ? 32u : remaining;
          if (table == ModbusRTUSlave::BridgeIngressTable::Coils ||
              pending.start != expectedStart + chunkOffset ||
              pending.count != chunkCount ||
              pending.snapshotCount != chunkCount || pending.units != 1u) {
            ++failures;
          }
          tokenChecksum += pending.sourceToken;
          if (!lane.slave.bridgeCommit(table, pending.sourceToken)) {
            ++failures;
          }
        }
        lane.slave.tx_pump();
        ArduinoTest::advance(8u * kTxBudgetPerByteUs8N1At19200);
        lane.slave.tx_pump();
        lane.slave.tx_pump();
      }
      samples[sample] =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - started).count();
    }
    std::sort(samples.begin(), samples.end());
  };

  std::vector<uint16_t> maxValues(32u);
  for (uint16_t i = 0; i < maxValues.size(); ++i) {
    maxValues[i] = static_cast<uint16_t>(0x6000u + i);
  }
  Fixture forwardedMax(NO_DE_PIN);
  const auto forwardedMaxRequest =
      writeMultipleHolding(kUnitId, 10u, maxValues);
  constexpr uint32_t kForwardedMaxRequests = 10000u;
  std::array<int64_t, kSamples> forwardedMaxSamples{};
  uint64_t forwardedMaxTokenChecksum = 0u;
  uint32_t forwardedMaxFailures = 0u;
  measureForwarded(forwardedMax, forwardedMaxRequest, 10u, 32u, 1u,
                   kForwardedMaxRequests, forwardedMaxSamples,
                   forwardedMaxTokenChecksum, forwardedMaxFailures);
  TEST_ASSERT_EQUAL_UINT32(0u, forwardedMaxFailures);
  TEST_ASSERT_NOT_EQUAL(0u, forwardedMaxTokenChecksum);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedMaxRequests,
                           forwardedMax.admissionCount);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedMaxRequests,
                           forwardedMax.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedMaxRequests,
                           forwardedMax.serial.flushCalls);

  std::vector<uint16_t> multichunkValues(65u);
  for (uint16_t i = 0; i < multichunkValues.size(); ++i) {
    multichunkValues[i] = static_cast<uint16_t>(0x7000u + i);
  }
  Fixture forwardedMultichunk(NO_DE_PIN);
  const auto forwardedMultichunkRequest =
      writeMultipleHolding(kUnitId, 20u, multichunkValues);
  constexpr uint32_t kForwardedMultichunkRequests = 10000u;
  std::array<int64_t, kSamples> forwardedMultichunkSamples{};
  uint64_t forwardedMultichunkTokenChecksum = 0u;
  uint32_t forwardedMultichunkFailures = 0u;
  measureForwarded(
      forwardedMultichunk, forwardedMultichunkRequest, 20u, 65u, 3u,
      kForwardedMultichunkRequests, forwardedMultichunkSamples,
      forwardedMultichunkTokenChecksum, forwardedMultichunkFailures);
  TEST_ASSERT_EQUAL_UINT32(0u, forwardedMultichunkFailures);
  TEST_ASSERT_NOT_EQUAL(0u, forwardedMultichunkTokenChecksum);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedMultichunkRequests,
                           forwardedMultichunk.admissionCount);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedMultichunkRequests,
                           forwardedMultichunk.serial.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(kSamples * kForwardedMultichunkRequests,
                           forwardedMultichunk.serial.flushCalls);

  const auto idleNsPerOp = idleSamples[kSamples / 2u] / kIdlePolls;
  const auto requestNsPerOp = requestSamples[kSamples / 2u] / kRequests;
  const auto forwardedNsPerOp =
      forwardedSamples[kSamples / 2u] / kForwardedRequests;
  const auto forwardedMaxNsPerOp =
      forwardedMaxSamples[kSamples / 2u] / kForwardedMaxRequests;
  const auto forwardedMultichunkNsPerOp =
      forwardedMultichunkSamples[kSamples / 2u] /
      kForwardedMultichunkRequests;
  // Emit both raw medians and per-operation summaries before applying the
  // strict gate. A failed same-host run must still leave actionable evidence,
  // and the raw totals avoid integer-rounding noise in the A/B comparator.
  std::printf(
      "modbus_perf: idle_median_ns=%lld idle_ops=%u "
      "request_median_ns=%lld request_ops=%u\n",
      static_cast<long long>(idleSamples[kSamples / 2u]), kIdlePolls,
      static_cast<long long>(requestSamples[kSamples / 2u]), kRequests);
  std::printf(
      "modbus_forwarded_perf: forwarded_median_ns=%lld "
      "forwarded_ops=%u max_median_ns=%lld max_ops=%u "
      "multichunk_median_ns=%lld multichunk_ops=%u\n",
      static_cast<long long>(forwardedSamples[kSamples / 2u]),
      kForwardedRequests,
      static_cast<long long>(forwardedMaxSamples[kSamples / 2u]),
      kForwardedMaxRequests,
      static_cast<long long>(forwardedMultichunkSamples[kSamples / 2u]),
      kForwardedMultichunkRequests);
  std::printf(
      "characterization: idle=%lldns/op requests=%lldns/op "
      "forwarded=%lldns/op forwarded_max=%lldns/op "
      "forwarded_multichunk=%lldns/op object=%zuB pending=%zuB\n",
      static_cast<long long>(idleNsPerOp),
      static_cast<long long>(requestNsPerOp),
      static_cast<long long>(forwardedNsPerOp),
      static_cast<long long>(forwardedMaxNsPerOp),
      static_cast<long long>(forwardedMultichunkNsPerOp),
      sizeof(ModbusRTUSlave), sizeof(ModbusRTUSlave::BridgeIngressEntry));
  std::fflush(stdout);
  if (std::getenv("MBUS_RTU_SLAVE_STRICT_PERFORMANCE") != nullptr) {
    // These conservative absolute ceilings catch material regressions while
    // ordinary CI keeps only deterministic operation-count gates. Medians
    // reject scheduler outliers on a pinned CPU.
    TEST_ASSERT_TRUE(idleNsPerOp <= 20);
    TEST_ASSERT_TRUE(requestNsPerOp <= 780);
    // This ceiling is intentionally reviewed separately from the local FC6
    // lane: it includes admission, immutable source publication, custom-entry
    // peek, and exact-token retirement on every request.
    TEST_ASSERT_TRUE(forwardedNsPerOp <= 900);
    // Multi-entry writes naturally cost more than the single-entry lane, so
    // they retain separate ceilings.
    TEST_ASSERT_TRUE(forwardedMaxNsPerOp <= 3600);
    TEST_ASSERT_TRUE(forwardedMultichunkNsPerOp <= 6400);
  }
}

void test_bridge_mode_native_object_footprint_stays_bounded() {
  static_assert(std::is_standard_layout<ModbusRTUSlave::BridgeIngressEntry>::value,
                "ModbusRTUSlave::BridgeIngressEntry must remain a zero-copy journal record");
  static_assert(std::is_trivially_copyable<ModbusRTUSlave::BridgeIngressEntry>::value,
                "ModbusRTUSlave::BridgeIngressEntry must remain safe for journal snapshot memcpy");
  TEST_ASSERT_EQUAL_UINT32(76u, sizeof(ModbusRTUSlave::BridgeIngressEntry));
  TEST_ASSERT_EQUAL_UINT32(16144u, sizeof(ModbusRTUSlave));
}

} // namespace

int main(int, char**) {
  UNITY_BEGIN();
  if (std::getenv("MBUS_RTU_SLAVE_PERFORMANCE_ONLY") != nullptr) {
    RUN_TEST(test_idle_poll_and_request_throughput_stay_within_characterized_budgets);
    return UNITY_END();
  }
  RUN_TEST(test_frame_completes_exactly_at_t35_and_read_wire_image_is_stable);
  RUN_TEST(test_partial_frame_restarts_t35_from_the_last_byte);
  RUN_TEST(test_crc_and_foreign_unit_rejection_leave_transport_quiet_then_resync);
  RUN_TEST(test_short_addressed_frame_reports_one_malformed_event_without_reply);
  RUN_TEST(test_all_read_function_wire_images_and_coil_padding_are_stable);
  RUN_TEST(test_single_write_ack_images_mutation_and_snapshots_are_stable);
  RUN_TEST(test_multiple_write_ack_images_mutation_and_snapshots_are_stable);
  RUN_TEST(test_exception_wire_images_do_not_mutate_or_enqueue);
  RUN_TEST(test_broadcast_read_is_ignored_and_write_is_durable_without_reply);
  RUN_TEST(test_denied_ordinary_broadcast_is_silent_and_nonmutating);
  RUN_TEST(test_targeted_broadcast_filters_target_and_length_then_snapshots_fire_forget);
  RUN_TEST(test_targeted_broadcast_all_inner_writes_preserve_cross_table_order);
  RUN_TEST(test_unicast_targeted_broadcast_is_illegal_and_has_no_side_effect);
  RUN_TEST(test_targeted_broadcast_denial_is_silent_nonmutating_and_records_ff_drop);
  RUN_TEST(test_admission_precedes_mutation_snapshot_and_ack);
  RUN_TEST(test_admission_denial_prevents_mutation_and_returns_device_failure);
  RUN_TEST(test_local_write_bypasses_queue_and_notifies_before_ack);
  RUN_TEST(test_bridge_defaults_admit_without_product_policy_or_reserved_coils);
  RUN_TEST(test_combined_source_queue_preserves_cross_table_order_and_exact_commit);
  RUN_TEST(test_large_write_chunks_snapshots_without_reordering_or_flag_duplication);
  RUN_TEST(test_source_queue_saturation_rejects_before_mutation_and_records_overflow);
  RUN_TEST(test_multi_chunk_capacity_is_reserved_before_any_mutation);
  RUN_TEST(test_overflow_tokens_and_newest_retention_are_stable);
  RUN_TEST(test_poll_and_tx_pump_operation_counts_are_exact);
  RUN_TEST(test_tx_pump_due_boundary_preserves_nonblocking_de_order);
  RUN_TEST(test_frame_and_tx_deadlines_are_rollover_safe);
  RUN_TEST(test_work_pending_tracks_serial_rx_frame_and_tx_lifecycle);
  RUN_TEST(test_upstream_tx_diagnostics_preserve_queue_pump_done_order_and_buckets);
  RUN_TEST(test_idle_poll_and_request_throughput_stay_within_characterized_budgets);
  // Keep the long token churn after same-host timings so the additional Stage
  // B oracle does not thermally perturb candidate-only performance samples.
  RUN_TEST(test_source_and_overflow_tokens_skip_zero_at_uint16_rollover);
  RUN_TEST(test_bridge_mode_native_object_footprint_stays_bounded);
  return UNITY_END();
}
