#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <type_traits>

#include "Comms/ModbusRTUIngressJournal.h"
#include "LegacyModbusIngressJournalReference.h"

namespace AllocationProbe {
static bool enabled = false;
static std::size_t calls = 0U;
}  // namespace AllocationProbe

void* operator new(std::size_t size) {
  if (AllocationProbe::enabled) ++AllocationProbe::calls;
  void* const memory = std::malloc(size);
  if (!memory) throw std::bad_alloc();
  return memory;
}

void* operator new[](std::size_t size) {
  if (AllocationProbe::enabled) ++AllocationProbe::calls;
  void* const memory = std::malloc(size);
  if (!memory) throw std::bad_alloc();
  return memory;
}

void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }

namespace {

static unsigned gChecks = 0U;
static unsigned gTests = 0U;

void fail(const char* expression, const char* file, int line) {
  std::fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
  std::exit(1);
}

#define CHECK(expression)                                      \
  do {                                                         \
    ++gChecks;                                                 \
    if (!(expression)) fail(#expression, __FILE__, __LINE__); \
  } while (false)

typedef ModbusRTU::FixedCapacityIngressJournal<4U, 3U, 5U>
    NeutralJournal;
typedef LegacyModbusIngressJournalReference::Journal<4U, 3U, 5U>
    LegacyJournal;
typedef ModbusRTU::FixedCapacityIngressJournal<64U, 32U, 50U>
    ProductionShapeJournal;

// Exercise the zero-copy compatibility seam used by ModbusRTUSlave without
// importing Arduino, OGM register definitions, or the production facade.
struct CompatEntry {
  uint16_t start;
  uint16_t count;
  uint16_t sessionGeneration;
  uint16_t sourceToken;
  uint8_t ops;
  uint8_t meta;
  uint8_t snapshotCount;
  ModbusRTU::FixedCapacityIngressSnapshot<4U, 3U> snapshot;
};

struct CompatEntryTraits {
  typedef CompatEntry Entry;
  static uint16_t sourceToken(const Entry& entry) { return entry.sourceToken; }
  static void initialize(Entry& entry, uint16_t start, uint16_t count,
                         uint16_t context, uint16_t sourceToken,
                         uint8_t units, uint8_t entryAttributes,
                         uint8_t snapshotCount) {
    entry.start = start;
    entry.count = count;
    entry.sessionGeneration = context;
    entry.sourceToken = sourceToken;
    entry.ops = units;
    entry.meta = entryAttributes;
    entry.snapshotCount = snapshotCount;
  }
  static void copyCoils(Entry& entry, const bool* values, uint16_t count) {
    memcpy(entry.snapshot.coils, values,
           static_cast<std::size_t>(count) * sizeof(bool));
  }
  static void copyHolding(Entry& entry, const uint16_t* values,
                          uint16_t count) {
    memcpy(entry.snapshot.holding, values,
           static_cast<std::size_t>(count) * sizeof(uint16_t));
  }
};

typedef ModbusRTU::FixedCapacityIngressJournal<4U, 3U, 5U,
                                                CompatEntryTraits>
    CompatJournal;

enum class OracleTable : uint8_t { Coils = 0U, Holding = 1U };

uint8_t attributes(bool fireForget, bool responseDebt) {
  return static_cast<uint8_t>((fireForget ? 0x80U : 0U) |
                              (responseDebt ? 0x40U : 0U));
}

struct NormalizedEntry {
  OracleTable table;
  uint16_t start;
  uint16_t count;
  uint16_t context;
  uint16_t sourceToken;
  uint8_t units;
  uint8_t attributes;
  uint8_t snapshotCount;
  bool coils[4];
  uint16_t holding[3];

  NormalizedEntry()
      : table(OracleTable::Coils),
        start(0U), count(0U), context(0U), sourceToken(0U), units(0U),
        attributes(0U), snapshotCount(0U), coils(), holding() {}
};

bool entriesEqual(const NormalizedEntry& lhs, const NormalizedEntry& rhs) {
  if (lhs.table != rhs.table || lhs.start != rhs.start ||
      lhs.count != rhs.count || lhs.context != rhs.context ||
      lhs.sourceToken != rhs.sourceToken || lhs.units != rhs.units ||
      lhs.attributes != rhs.attributes ||
      lhs.snapshotCount != rhs.snapshotCount) {
    return false;
  }
  if (lhs.table == OracleTable::Coils) {
    for (uint8_t index = 0U; index < lhs.snapshotCount; ++index) {
      if (lhs.coils[index] != rhs.coils[index]) return false;
    }
  } else {
    for (uint8_t index = 0U; index < lhs.snapshotCount; ++index) {
      if (lhs.holding[index] != rhs.holding[index]) return false;
    }
  }
  return true;
}

struct NeutralAdapter {
  typedef NeutralJournal::Reservation Reservation;
  NeutralJournal journal;

  static NeutralJournal::Table table(OracleTable value) {
    return value == OracleTable::Coils ? NeutralJournal::Table::Coils
                                       : NeutralJournal::Table::HoldingRegisters;
  }

  bool reserve(OracleTable tableValue, uint16_t start, uint16_t count,
               uint16_t context, uint8_t entryAttributes, Reservation& out) {
    return journal.reserve(table(tableValue), start, count, context, 1U,
                           entryAttributes, out);
  }
  bool publishCoils(const Reservation& reservation, const bool* values) {
    return journal.publishCoils(reservation, values);
  }
  bool publishHolding(const Reservation& reservation, const uint16_t* values) {
    return journal.publishHolding(reservation, values);
  }
  bool peek(OracleTable tableValue, NormalizedEntry& out) const {
    NeutralJournal::Entry entry;
    if (!journal.peek(table(tableValue), entry)) return false;
    normalize(entry, tableValue, out);
    return true;
  }
  bool peekNext(NormalizedEntry& out, OracleTable& tableValue) const {
    NeutralJournal::Entry entry;
    NeutralJournal::Table selected = NeutralJournal::Table::Coils;
    if (!journal.peekNext(entry, selected)) return false;
    tableValue = selected == NeutralJournal::Table::Coils
        ? OracleTable::Coils : OracleTable::Holding;
    normalize(entry, tableValue, out);
    return true;
  }
  bool retire(OracleTable tableValue, uint16_t token) {
    return journal.retire(table(tableValue), token);
  }
  uint8_t freeSlots(OracleTable tableValue) const {
    return journal.freeSlots(table(tableValue));
  }
  static uint8_t requiredSlots(OracleTable tableValue, uint16_t count) {
    return NeutralJournal::requiredSlots(table(tableValue), count);
  }
  static uint16_t operationCount(OracleTable tableValue, uint16_t count) {
    return NeutralJournal::operationCount(table(tableValue), count);
  }

  static void normalize(const NeutralJournal::Entry& entry,
                        OracleTable tableValue, NormalizedEntry& out) {
    out = NormalizedEntry();
    out.table = tableValue;
    out.start = entry.start;
    out.count = entry.count;
    out.context = entry.context;
    out.sourceToken = entry.sourceToken;
    out.units = entry.units;
    out.attributes = entry.attributes;
    out.snapshotCount = entry.snapshotCount;
    if (tableValue == OracleTable::Coils) {
      for (uint8_t index = 0U; index < entry.snapshotCount; ++index) {
        out.coils[index] = entry.snapshot.coils[index];
      }
    } else {
      for (uint8_t index = 0U; index < entry.snapshotCount; ++index) {
        out.holding[index] = entry.snapshot.holding[index];
      }
    }
  }
};

struct LegacyAdapter {
  typedef LegacyJournal::Reservation Reservation;
  LegacyJournal journal;

  static LegacyModbusIngressJournalReference::Table table(OracleTable value) {
    return value == OracleTable::Coils
        ? LegacyModbusIngressJournalReference::Table::Coils
        : LegacyModbusIngressJournalReference::Table::Holding;
  }
  bool reserve(OracleTable tableValue, uint16_t start, uint16_t count,
               uint16_t context, uint8_t entryAttributes, Reservation& out) {
    return journal.reserve(table(tableValue), start, count,
                           (entryAttributes & 0x80U) != 0U,
                           (entryAttributes & 0x40U) != 0U, context, out);
  }
  bool publishCoils(const Reservation& reservation, const bool* values) {
    return journal.append(reservation, values);
  }
  bool publishHolding(const Reservation& reservation, const uint16_t* values) {
    return journal.append(reservation, values);
  }
  bool peek(OracleTable tableValue, NormalizedEntry& out) const {
    LegacyJournal::Entry entry;
    if (!journal.peek(table(tableValue), entry)) return false;
    normalize(entry, tableValue, out);
    return true;
  }
  bool peekNext(NormalizedEntry& out, OracleTable& tableValue) const {
    LegacyJournal::Entry entry;
    LegacyModbusIngressJournalReference::Table selected =
        LegacyModbusIngressJournalReference::Table::Coils;
    if (!journal.peekNext(entry, selected)) return false;
    tableValue = selected == LegacyModbusIngressJournalReference::Table::Coils
        ? OracleTable::Coils : OracleTable::Holding;
    normalize(entry, tableValue, out);
    return true;
  }
  bool retire(OracleTable tableValue, uint16_t token) {
    return journal.commit(table(tableValue), token);
  }
  uint8_t freeSlots(OracleTable tableValue) const {
    return static_cast<uint8_t>(4U - journal.sourceSize(table(tableValue)));
  }
  uint8_t requiredSlots(OracleTable tableValue, uint16_t count) const {
    return journal.requiredSlots(table(tableValue), count);
  }
  uint16_t operationCount(OracleTable tableValue, uint16_t count) const {
    const uint8_t slots = requiredSlots(tableValue, count);
    return slots == 0U ? uint16_t(1U) : static_cast<uint16_t>(slots);
  }

  static void normalize(const LegacyJournal::Entry& entry,
                        OracleTable tableValue, NormalizedEntry& out) {
    out = NormalizedEntry();
    out.table = tableValue;
    out.start = entry.start;
    out.count = entry.count;
    out.context = entry.sessionGeneration;
    out.sourceToken = entry.sourceToken;
    out.units = entry.ops;
    out.attributes = entry.meta;
    out.snapshotCount = entry.snapshotCount;
    if (tableValue == OracleTable::Coils) {
      for (uint8_t index = 0U; index < entry.snapshotCount; ++index) {
        out.coils[index] = entry.snapshot.coils[index];
      }
    } else {
      for (uint8_t index = 0U; index < entry.snapshotCount; ++index) {
        out.holding[index] = entry.snapshot.holding[index];
      }
    }
  }
};

struct CompatAdapter {
  typedef CompatJournal::Reservation Reservation;
  CompatJournal journal;

  static CompatJournal::Table table(OracleTable value) {
    return value == OracleTable::Coils ? CompatJournal::Table::Coils
                                       : CompatJournal::Table::HoldingRegisters;
  }
  bool reserve(OracleTable tableValue, uint16_t start, uint16_t count,
               uint16_t context, uint8_t entryAttributes, Reservation& out) {
    return journal.reserve(table(tableValue), start, count, context, 1U,
                           entryAttributes, out);
  }
  bool publishCoils(const Reservation& reservation, const bool* values) {
    return journal.publishCoils(reservation, values);
  }
  bool publishHolding(const Reservation& reservation, const uint16_t* values) {
    return journal.publishHolding(reservation, values);
  }
  bool peek(OracleTable tableValue, NormalizedEntry& out) const {
    CompatJournal::Entry entry;
    if (!journal.peek(table(tableValue), entry)) return false;
    normalize(entry, tableValue, out);
    return true;
  }
  bool peekNext(NormalizedEntry& out, OracleTable& tableValue) const {
    CompatJournal::Entry entry;
    CompatJournal::Table selected = CompatJournal::Table::Coils;
    if (!journal.peekNext(entry, selected)) return false;
    tableValue = selected == CompatJournal::Table::Coils
        ? OracleTable::Coils : OracleTable::Holding;
    normalize(entry, tableValue, out);
    return true;
  }
  bool retire(OracleTable tableValue, uint16_t token) {
    return journal.retire(table(tableValue), token);
  }
  uint8_t freeSlots(OracleTable tableValue) const {
    return journal.freeSlots(table(tableValue));
  }

  static void normalize(const CompatJournal::Entry& entry,
                        OracleTable tableValue, NormalizedEntry& out) {
    out = NormalizedEntry();
    out.table = tableValue;
    out.start = entry.start;
    out.count = entry.count;
    out.context = entry.sessionGeneration;
    out.sourceToken = entry.sourceToken;
    out.units = entry.ops;
    out.attributes = entry.meta;
    out.snapshotCount = entry.snapshotCount;
    if (tableValue == OracleTable::Coils) {
      for (uint8_t index = 0U; index < entry.snapshotCount; ++index) {
        out.coils[index] = entry.snapshot.coils[index];
      }
    } else {
      for (uint8_t index = 0U; index < entry.snapshotCount; ++index) {
        out.holding[index] = entry.snapshot.holding[index];
      }
    }
  }
};

enum class EventKind : uint8_t {
  Admission = 0U, Reservation, InvisibleBeforePublish, Mutation, Published,
  AckQueued, BroadcastSilent, SourceHead, WrongRetireRejected, ExactRetire
};

struct Event {
  EventKind kind;
  OracleTable table;
  uint16_t start;
  uint16_t count;
  uint16_t token;
  uint16_t valueFirst;
  uint16_t valueLast;
  uint8_t attributes;
  uint8_t snapshotCount;
  bool result;
};

struct Trace {
  Event events[24];
  uint8_t count;
  Trace() : events(), count(0U) {}

  void push(EventKind kind, OracleTable table, uint16_t start,
            uint16_t entryCount, bool result, uint16_t token = 0U,
            uint16_t valueFirst = 0U, uint16_t valueLast = 0U,
            uint8_t entryAttributes = 0U, uint8_t snapshotCount = 0U) {
    CHECK(count < static_cast<uint8_t>(sizeof(events) / sizeof(events[0])));
    Event& event = events[count++];
    event.kind = kind;
    event.table = table;
    event.start = start;
    event.count = entryCount;
    event.result = result;
    event.token = token;
    event.valueFirst = valueFirst;
    event.valueLast = valueLast;
    event.attributes = entryAttributes;
    event.snapshotCount = snapshotCount;
  }
};

bool eventsEqual(const Event& lhs, const Event& rhs) {
  return lhs.kind == rhs.kind && lhs.table == rhs.table &&
      lhs.start == rhs.start && lhs.count == rhs.count &&
      lhs.token == rhs.token && lhs.valueFirst == rhs.valueFirst &&
      lhs.valueLast == rhs.valueLast &&
      lhs.attributes == rhs.attributes &&
      lhs.snapshotCount == rhs.snapshotCount && lhs.result == rhs.result;
}

void checkTraceEqual(const Trace& legacy, const Trace& neutral) {
  CHECK(legacy.count == neutral.count);
  for (uint8_t index = 0U; index < legacy.count; ++index) {
    CHECK(eventsEqual(legacy.events[index], neutral.events[index]));
  }
}

template <typename Adapter>
Trace runAcceptedTrace() {
  Adapter adapter;
  Trace trace;
  uint16_t holding[4] = {0xAAAAU, 0xBBBBU, 0xCCCCU, 0xDDDDU};
  bool coils[3] = {false, false, false};
  NormalizedEntry entry;

  trace.push(EventKind::Admission, OracleTable::Holding, 10U, 4U, true,
             0U, holding[0], holding[3], attributes(false, true));
  typename Adapter::Reservation holdingReservation;
  const bool holdingReserved = adapter.reserve(
      OracleTable::Holding, 10U, 4U, 7U, attributes(false, true),
      holdingReservation);
  trace.push(EventKind::Reservation, OracleTable::Holding, 10U, 4U,
             holdingReserved);
  trace.push(EventKind::InvisibleBeforePublish, OracleTable::Holding, 10U, 4U,
             !adapter.peek(OracleTable::Holding, entry));
  holding[0] = 0x1001U;
  holding[1] = 0x1002U;
  holding[2] = 0x1003U;
  holding[3] = 0x1004U;
  trace.push(EventKind::Mutation, OracleTable::Holding, 10U, 4U, true,
             0U, holding[0], holding[3]);
  const bool holdingPublished =
      adapter.publishHolding(holdingReservation, holding);
  trace.push(EventKind::Published, OracleTable::Holding, 10U, 4U,
             holdingPublished);
  trace.push(EventKind::AckQueued, OracleTable::Holding, 10U, 4U,
             holdingPublished, 0U, 16U, 0U);

  trace.push(EventKind::Admission, OracleTable::Coils, 20U, 3U, true,
             0U, 0U, 0U, attributes(true, false));
  typename Adapter::Reservation coilReservation;
  const bool coilReserved = adapter.reserve(
      OracleTable::Coils, 20U, 3U, 8U, attributes(true, false),
      coilReservation);
  trace.push(EventKind::Reservation, OracleTable::Coils, 20U, 3U,
             coilReserved);
  trace.push(EventKind::InvisibleBeforePublish, OracleTable::Coils, 20U, 3U,
             !adapter.peek(OracleTable::Coils, entry));
  coils[0] = true;
  coils[1] = false;
  coils[2] = true;
  trace.push(EventKind::Mutation, OracleTable::Coils, 20U, 3U, true,
             0U, 1U, 1U);
  const bool coilsPublished = adapter.publishCoils(coilReservation, coils);
  trace.push(EventKind::Published, OracleTable::Coils, 20U, 3U,
             coilsPublished);
  trace.push(EventKind::BroadcastSilent, OracleTable::Coils, 20U, 3U,
             coilsPublished, 0U, 15U, 0U);

  // Producer-side values are mutable; the published journal image is not.
  holding[0] = 0x9001U;
  holding[1] = 0x9002U;
  holding[2] = 0x9003U;
  holding[3] = 0x9004U;
  coils[0] = false;
  coils[1] = true;
  coils[2] = false;

  OracleTable selected = OracleTable::Coils;
  while (adapter.peekNext(entry, selected)) {
    const uint16_t first = selected == OracleTable::Coils
        ? static_cast<uint16_t>(entry.coils[0] ? 1U : 0U)
        : entry.holding[0];
    const uint16_t last = selected == OracleTable::Coils
        ? static_cast<uint16_t>(
              entry.coils[entry.snapshotCount - 1U] ? 1U : 0U)
        : entry.holding[entry.snapshotCount - 1U];
    trace.push(EventKind::SourceHead, selected, entry.start, entry.count, true,
               entry.sourceToken, first, last, entry.attributes,
               entry.snapshotCount);
    const bool wrongRejected = !adapter.retire(
        selected, static_cast<uint16_t>(entry.sourceToken + 1U));
    trace.push(EventKind::WrongRetireRejected, selected, entry.start,
               entry.count, wrongRejected, entry.sourceToken);
    const bool retired = adapter.retire(selected, entry.sourceToken);
    trace.push(EventKind::ExactRetire, selected, entry.start, entry.count,
               retired, entry.sourceToken);
  }
  return trace;
}

void checkAcceptedGolden(const Trace& trace) {
  CHECK(trace.count == 21U);
  CHECK(trace.events[0].kind == EventKind::Admission);
  CHECK(trace.events[1].kind == EventKind::Reservation);
  CHECK(trace.events[2].kind == EventKind::InvisibleBeforePublish);
  CHECK(trace.events[3].kind == EventKind::Mutation);
  CHECK(trace.events[4].kind == EventKind::Published);
  CHECK(trace.events[5].kind == EventKind::AckQueued);
  CHECK(trace.events[5].valueFirst == 16U);
  CHECK(trace.events[6].kind == EventKind::Admission);
  CHECK(trace.events[11].kind == EventKind::BroadcastSilent);
  CHECK(trace.events[11].valueFirst == 15U);

  const Event& first = trace.events[12];
  CHECK(first.kind == EventKind::SourceHead);
  CHECK(first.table == OracleTable::Holding);
  CHECK(first.start == 10U && first.count == 3U);
  CHECK(first.token == 2U);
  CHECK(first.attributes == attributes(false, true));
  CHECK(first.snapshotCount == 3U);
  CHECK(first.valueFirst == 0x1001U && first.valueLast == 0x1003U);

  const Event& second = trace.events[15];
  CHECK(second.kind == EventKind::SourceHead);
  CHECK(second.table == OracleTable::Holding);
  CHECK(second.start == 13U && second.count == 1U);
  CHECK(second.token == 3U);
  CHECK(second.valueFirst == 0x1004U && second.valueLast == 0x1004U);

  const Event& third = trace.events[18];
  CHECK(third.kind == EventKind::SourceHead);
  CHECK(third.table == OracleTable::Coils);
  CHECK(third.start == 20U && third.count == 3U);
  CHECK(third.token == 4U);
  CHECK(third.attributes == attributes(true, false));
  CHECK(third.valueFirst == 1U && third.valueLast == 1U);
  for (uint8_t index = 12U; index < 21U; ++index) {
    CHECK(trace.events[index].result);
  }
}

void test_normalized_accepted_trace_matches_legacy_and_golden() {
  const Trace legacy = runAcceptedTrace<LegacyAdapter>();
  const Trace neutral = runAcceptedTrace<NeutralAdapter>();
  checkTraceEqual(legacy, neutral);
  checkAcceptedGolden(neutral);
}

template <typename Adapter>
void checkAllOrNothingReservation() {
  Adapter adapter;
  const uint16_t value = 0x7000U;
  for (uint16_t index = 0U; index < 2U; ++index) {
    typename Adapter::Reservation fill;
    CHECK(adapter.reserve(OracleTable::Holding,
                          static_cast<uint16_t>(40U + index), 1U, 3U,
                          attributes(true, false), fill));
    CHECK(adapter.publishHolding(fill, &value));
  }
  CHECK(adapter.freeSlots(OracleTable::Holding) == 2U);
  CHECK(adapter.freeSlots(OracleTable::Coils) == 4U);

  uint16_t image[7] = {0xAAAAU, 0xAAAAU, 0xAAAAU, 0xAAAAU,
                       0xAAAAU, 0xAAAAU, 0xAAAAU};
  typename Adapter::Reservation denied;
  const bool reserved = adapter.reserve(
      OracleTable::Holding, 60U, 7U, 9U, attributes(false, true), denied);
  CHECK(!reserved);
  if (reserved) {
    for (uint8_t index = 0U; index < 7U; ++index) image[index] = 0xBBBBU;
  }
  CHECK(image[0] == 0xAAAAU && image[6] == 0xAAAAU);
  CHECK(adapter.freeSlots(OracleTable::Holding) == 2U);

  NormalizedEntry first;
  CHECK(adapter.peek(OracleTable::Holding, first));
  CHECK(first.start == 40U && first.sourceToken == 2U);
  CHECK(adapter.retire(OracleTable::Holding, first.sourceToken));
  CHECK(adapter.freeSlots(OracleTable::Holding) == 3U);
  CHECK(adapter.reserve(OracleTable::Holding, 60U, 7U, 9U,
                        attributes(false, true), denied));
  CHECK(adapter.publishHolding(denied, image));
}

void test_pre_admission_is_all_or_nothing_and_table_local() {
  checkAllOrNothingReservation<LegacyAdapter>();
  checkAllOrNothingReservation<NeutralAdapter>();
}

void test_foreign_stale_and_invalid_reservations_fail_closed() {
  NeutralJournal first;
  NeutralJournal second;
  NeutralJournal::Reservation foreign;
  CHECK(first.reserve(NeutralJournal::Table::HoldingRegisters,
                      5U, 1U, 11U, 1U, attributes(false, true), foreign));
  const uint16_t value = 0xCAFEU;
  CHECK(!second.publishHolding(foreign, &value));
  NeutralJournal::Entry entry;
  CHECK(!second.peek(NeutralJournal::Table::HoldingRegisters, entry));
  CHECK(first.publishHolding(foreign, &value));
  CHECK(first.peek(NeutralJournal::Table::HoldingRegisters, entry));
  CHECK(entry.context == 11U && entry.snapshot.holding[0] == value);

  NeutralJournal staleJournal;
  NeutralJournal::Reservation stale;
  NeutralJournal::Reservation current;
  CHECK(staleJournal.reserve(NeutralJournal::Table::Coils,
                             1U, 1U, 12U, 1U, 0U, stale));
  CHECK(staleJournal.reserve(NeutralJournal::Table::Coils,
                             2U, 1U, 13U, 1U, 0U, current));
  const bool coil = true;
  CHECK(staleJournal.publishCoils(current, &coil));
  CHECK(!staleJournal.publishCoils(stale, &coil));

  NeutralJournal invalid;
  NeutralJournal::Reservation reservation;
  CHECK(!invalid.reserve(NeutralJournal::Table::Coils,
                         0U, 0U, 0U, 1U, 0U, reservation));
  CHECK(!invalid.reserve(NeutralJournal::Table::Coils,
                         0xFFFFU, 2U, 0U, 1U, 0U, reservation));
  CHECK(invalid.reserve(NeutralJournal::Table::Coils,
                        1U, 1U, 1U, 1U, 0U, reservation));
  CHECK(!invalid.publishCoils(reservation, 0));
  CHECK(!invalid.publishHolding(reservation, &value));
  CHECK(!invalid.peek(NeutralJournal::Table::Coils, entry));
  CHECK(invalid.publishCoils(reservation, &coil));
  CHECK(!invalid.retire(NeutralJournal::Table::Coils, 0U));
  CHECK(!invalid.retire(NeutralJournal::Table::Coils, 3U));
  CHECK(!invalid.retire(NeutralJournal::Table::HoldingRegisters, 2U));
  CHECK(invalid.peek(NeutralJournal::Table::Coils, entry));
  CHECK(entry.sourceToken == 2U);
  CHECK(invalid.retire(NeutralJournal::Table::Coils, 2U));
}

void test_chunk_counts_and_capacity_are_exact() {
  CHECK(NeutralAdapter::requiredSlots(OracleTable::Coils, 0U) == 0U);
  CHECK(NeutralAdapter::requiredSlots(OracleTable::Coils, 1U) == 1U);
  CHECK(NeutralAdapter::requiredSlots(OracleTable::Coils, 4U) == 1U);
  CHECK(NeutralAdapter::requiredSlots(OracleTable::Coils, 5U) == 2U);
  CHECK(NeutralAdapter::requiredSlots(OracleTable::Holding, 3U) == 1U);
  CHECK(NeutralAdapter::requiredSlots(OracleTable::Holding, 4U) == 2U);
  CHECK(NeutralAdapter::requiredSlots(OracleTable::Holding, 12U) == 4U);
  CHECK(NeutralAdapter::requiredSlots(OracleTable::Holding, 13U) == 5U);
  CHECK(NeutralAdapter::operationCount(OracleTable::Holding, 0U) == 1U);
  CHECK(NeutralAdapter::operationCount(OracleTable::Holding, 7U) == 3U);
  NeutralAdapter adapter;
  CHECK(adapter.freeSlots(OracleTable::Coils) == 4U);
  CHECK(adapter.freeSlots(OracleTable::Holding) == 4U);
}

void test_custom_entry_traits_preserve_opaque_units_and_attributes() {
  CompatJournal journal;
  CompatJournal::Reservation reservation;
  CHECK(journal.reserve(CompatJournal::Table::HoldingRegisters,
                        30U, 4U, 0x1234U, 7U, 0xA5U, reservation));
  const uint16_t values[4] = {0x1010U, 0x2020U, 0x3030U, 0x4040U};
  CHECK(journal.publishHolding(reservation, values));

  CompatEntry first;
  CHECK(journal.peek(CompatJournal::Table::HoldingRegisters, first));
  CHECK(first.start == 30U && first.count == 3U);
  CHECK(first.sessionGeneration == 0x1234U);
  CHECK(first.ops == 7U && first.meta == 0xA5U);
  CHECK(first.sourceToken == 2U && first.snapshotCount == 3U);
  CHECK(first.snapshot.holding[0] == 0x1010U);
  CHECK(first.snapshot.holding[2] == 0x3030U);
  CHECK(journal.retire(CompatJournal::Table::HoldingRegisters,
                       first.sourceToken));

  CompatEntry second;
  CHECK(journal.peek(CompatJournal::Table::HoldingRegisters, second));
  CHECK(second.start == 33U && second.count == 1U);
  CHECK(second.sessionGeneration == 0x1234U);
  CHECK(second.ops == 7U && second.meta == 0xA5U);
  CHECK(second.sourceToken == 3U && second.snapshotCount == 1U);
  CHECK(second.snapshot.holding[0] == 0x4040U);
}

template <typename Adapter>
void advanceSourceToWrap(Adapter& adapter) {
  const uint16_t value = 0x1234U;
  for (uint32_t index = 0U; index < 65533UL; ++index) {
    typename Adapter::Reservation reservation;
    CHECK(adapter.reserve(OracleTable::Holding, 1U, 1U, 1U,
                          attributes(true, false), reservation));
    CHECK(adapter.publishHolding(reservation, &value));
    NormalizedEntry entry;
    CHECK(adapter.peek(OracleTable::Holding, entry));
    CHECK(entry.sourceToken != 0U);
    CHECK(adapter.retire(OracleTable::Holding, entry.sourceToken));
  }
}

template <typename Adapter>
void checkSourceRollover(NormalizedEntry& first, NormalizedEntry& second) {
  Adapter adapter;
  advanceSourceToWrap(adapter);
  const bool coil = true;
  const uint16_t holding = 0xBEEFU;
  typename Adapter::Reservation coilReservation;
  typename Adapter::Reservation holdingReservation;
  CHECK(adapter.reserve(OracleTable::Coils, 10U, 1U, 5U,
                        attributes(true, false), coilReservation));
  CHECK(adapter.publishCoils(coilReservation, &coil));
  CHECK(adapter.reserve(OracleTable::Holding, 11U, 1U, 6U,
                        attributes(false, true), holdingReservation));
  CHECK(adapter.publishHolding(holdingReservation, &holding));
  OracleTable selected = OracleTable::Holding;
  CHECK(adapter.peekNext(first, selected));
  CHECK(selected == OracleTable::Coils);
  CHECK(first.sourceToken == 0xFFFFU);
  CHECK(adapter.retire(selected, first.sourceToken));
  CHECK(adapter.peekNext(second, selected));
  CHECK(selected == OracleTable::Holding);
  CHECK(second.sourceToken == 1U);
}

void test_source_token_rollover_matches_legacy() {
  NormalizedEntry legacyFirst;
  NormalizedEntry legacySecond;
  NormalizedEntry neutralFirst;
  NormalizedEntry neutralSecond;
  checkSourceRollover<LegacyAdapter>(legacyFirst, legacySecond);
  checkSourceRollover<NeutralAdapter>(neutralFirst, neutralSecond);
  CHECK(entriesEqual(legacyFirst, neutralFirst));
  CHECK(entriesEqual(legacySecond, neutralSecond));
}

void test_fixed_footprint_and_operations_allocate_nothing() {
  static_assert(std::is_standard_layout<NeutralJournal::Entry>::value,
                "journal entries must remain simple fixed records");
  static_assert(std::is_trivially_copyable<NeutralJournal::Entry>::value,
                "default entries must be safe for snapshot memcpy");
  static_assert(std::is_standard_layout<CompatEntry>::value,
                "compatibility entries must remain standard-layout");
  static_assert(std::is_trivially_copyable<CompatEntry>::value,
                "compatibility entries must be safe for snapshot memcpy");
  static_assert(std::is_trivially_destructible<NeutralJournal>::value,
                "journal teardown must not own dynamic resources");
  static_assert(!std::is_copy_constructible<NeutralJournal>::value,
                "copying a journal would duplicate token and queue state");
  static_assert(!std::is_move_constructible<NeutralJournal>::value,
                "moving a journal would invalidate reservation ownership");
  CHECK(sizeof(NeutralJournal::Entry) == 18U);
  CHECK(sizeof(NeutralJournal) == 186U);
  CHECK(sizeof(NeutralJournal) == sizeof(LegacyJournal));
  CHECK(sizeof(CompatEntry) == sizeof(NeutralJournal::Entry));
  CHECK(sizeof(CompatJournal) == sizeof(NeutralJournal));
  CHECK(sizeof(ProductionShapeJournal::Entry) == 76U);
  CHECK(sizeof(ProductionShapeJournal) == 7606U);

  AllocationProbe::calls = 0U;
  AllocationProbe::enabled = true;
  {
    NeutralAdapter adapter;
    const bool coils[4] = {true, false, true, false};
    const uint16_t holding[4] = {1U, 2U, 3U, 4U};
    NeutralAdapter::Reservation coilReservation;
    NeutralAdapter::Reservation holdingReservation;
    CHECK(adapter.reserve(OracleTable::Coils, 0U, 4U, 1U,
                          attributes(true, false), coilReservation));
    CHECK(adapter.publishCoils(coilReservation, coils));
    CHECK(adapter.reserve(OracleTable::Holding, 0U, 4U, 1U,
                          attributes(false, true), holdingReservation));
    CHECK(adapter.publishHolding(holdingReservation, holding));
    NormalizedEntry entry;
    OracleTable table = OracleTable::Coils;
    while (adapter.peekNext(entry, table)) {
      CHECK(adapter.retire(table, entry.sourceToken));
    }
  }
  AllocationProbe::enabled = false;
  CHECK(AllocationProbe::calls == 0U);
}

template <typename Adapter>
int64_t benchmarkForwardedJournal(uint32_t operations,
                                  volatile uint32_t& checksum) {
  Adapter adapter;
  const uint16_t value = 0x5A5AU;
  const std::chrono::steady_clock::time_point started =
      std::chrono::steady_clock::now();
  for (uint32_t index = 0U; index < operations; ++index) {
    typename Adapter::Reservation reservation;
    if (!adapter.reserve(OracleTable::Holding,
                         static_cast<uint16_t>(index), 1U, 1U,
                         attributes(false, true), reservation)) return -1;
    if (!adapter.publishHolding(reservation, &value)) return -1;
    NormalizedEntry entry;
    if (!adapter.peek(OracleTable::Holding, entry)) return -1;
    checksum += entry.sourceToken;
    if (!adapter.retire(OracleTable::Holding, entry.sourceToken)) return -1;
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now() - started).count();
}

int64_t median(int64_t* values, uint8_t count) {
  for (uint8_t outer = 1U; outer < count; ++outer) {
    const int64_t value = values[outer];
    uint8_t inner = outer;
    while (inner > 0U && values[inner - 1U] > value) {
      values[inner] = values[inner - 1U];
      --inner;
    }
    values[inner] = value;
  }
  return values[count / 2U];
}

void test_forwarded_journal_performance_lane() {
  const bool strict = std::getenv("OGM_STRICT_MODBUS_PERF") != 0;
  const uint8_t samples = strict ? 7U : 1U;
  const uint32_t operations = strict ? 1000000UL : 250000UL;
  int64_t legacySamples[7] = {};
  int64_t neutralSamples[7] = {};
  int64_t compatSamples[7] = {};
  volatile uint32_t legacyChecksum = 0U;
  volatile uint32_t neutralChecksum = 0U;
  volatile uint32_t compatChecksum = 0U;

  for (uint8_t sample = 0U; sample < samples; ++sample) {
    if ((sample & 1U) == 0U) {
      legacySamples[sample] = benchmarkForwardedJournal<LegacyAdapter>(
          operations, legacyChecksum);
      neutralSamples[sample] = benchmarkForwardedJournal<NeutralAdapter>(
          operations, neutralChecksum);
      compatSamples[sample] = benchmarkForwardedJournal<CompatAdapter>(
          operations, compatChecksum);
    } else {
      compatSamples[sample] = benchmarkForwardedJournal<CompatAdapter>(
          operations, compatChecksum);
      neutralSamples[sample] = benchmarkForwardedJournal<NeutralAdapter>(
          operations, neutralChecksum);
      legacySamples[sample] = benchmarkForwardedJournal<LegacyAdapter>(
          operations, legacyChecksum);
    }
    CHECK(legacySamples[sample] > 0);
    CHECK(neutralSamples[sample] > 0);
    CHECK(compatSamples[sample] > 0);
  }
  CHECK(legacyChecksum == neutralChecksum);
  CHECK(legacyChecksum == compatChecksum);

  const int64_t legacyNs = median(legacySamples, samples);
  const int64_t neutralNs = median(neutralSamples, samples);
  const int64_t compatNs = median(compatSamples, samples);
  std::printf("ingress_journal_perf: legacy_median_ns=%lld "
              "neutral_median_ns=%lld compat_median_ns=%lld "
              "ops=%u samples=%u strict=%u\n",
              static_cast<long long>(legacyNs),
              static_cast<long long>(neutralNs),
              static_cast<long long>(compatNs), operations, samples,
              strict ? 1U : 0U);
  CHECK(legacyNs < 2000000000LL);
  CHECK(neutralNs < 2000000000LL);
  CHECK(compatNs < 2000000000LL);
  if (strict) {
    // Same-host migration gate against the frozen OGM behavior. The existing
    // whole-slave absolute ceilings remain an independent strict gate.
    CHECK(neutralNs <= legacyNs + (legacyNs / 20LL));
    CHECK(compatNs <= legacyNs + (legacyNs / 20LL));
  }
}

typedef void (*TestFunction)();
void runTest(const char* name, TestFunction function) {
  function();
  ++gTests;
  std::printf("PASS %s\n", name);
}

}  // namespace

int main() {
  runTest("normalized accepted trace",
          &test_normalized_accepted_trace_matches_legacy_and_golden);
  runTest("all-or-nothing pre-admission",
          &test_pre_admission_is_all_or_nothing_and_table_local);
  runTest("foreign stale and invalid reservations",
          &test_foreign_stale_and_invalid_reservations_fail_closed);
  runTest("chunk counts and capacity",
          &test_chunk_counts_and_capacity_are_exact);
  runTest("custom entry opaque fields",
          &test_custom_entry_traits_preserve_opaque_units_and_attributes);
  runTest("source token rollover",
          &test_source_token_rollover_matches_legacy);
  runTest("fixed footprint and no allocation",
          &test_fixed_footprint_and_operations_allocate_nothing);
  runTest("forwarded journal performance",
          &test_forwarded_journal_performance_lane);
  std::printf("Modbus RTU ingress journal oracle: %u tests, %u checks\n",
              gTests, gChecks);
  return 0;
}
