#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Test-only executable specification of the pre-extraction bridge ingress
// rings. Keep this independent of ModbusRTUIngressJournal so the Stage B suite
// can compare the neutral implementation with the frozen legacy semantics.
namespace LegacyModbusIngressJournalReference {

enum class Table : uint8_t {
  Coils = 0U,
  Holding = 1U,
};

static const uint8_t kFlagFireForget = 0x80U;
static const uint8_t kFlagPublicDebt = 0x40U;

template <size_t CoilSnapshotMax,
          size_t HoldingSnapshotMax,
          size_t RingCapacity>
class Journal {
 public:
  static_assert(CoilSnapshotMax > 0U,
                "legacy reference requires a nonzero coil chunk");
  static_assert(HoldingSnapshotMax > 0U,
                "legacy reference requires a nonzero holding chunk");
  static_assert(RingCapacity > 1U,
                "legacy reference ring needs a sentinel slot");
  static_assert(RingCapacity <= 255U,
                "legacy reference indexes must fit uint8_t");
  static_assert(CoilSnapshotMax <= 255U,
                "legacy snapshot count must fit uint8_t");
  static_assert(HoldingSnapshotMax <= 255U,
                "legacy snapshot count must fit uint8_t");

  union Snapshot {
    bool coils[CoilSnapshotMax];
    uint16_t holding[HoldingSnapshotMax];

    Snapshot() {}
  };

  struct Entry {
    uint16_t start;
    uint16_t count;
    uint16_t sessionGeneration;
    uint16_t sourceToken;
    uint8_t ops;
    uint8_t meta;
    uint8_t snapshotCount;
    Snapshot snapshot;

    Entry()
        : start(0U),
          count(0U),
          sessionGeneration(0U),
          sourceToken(0U),
          ops(1U),
          meta(0U),
          snapshotCount(0U),
          snapshot() {}
  };

  struct Reservation {
    Table table;
    uint16_t start;
    uint16_t count;
    uint16_t sessionGeneration;
    uint8_t requiredSlots;
    bool fireForget;
    bool publicDebt;
    bool accepted;

    Reservation()
        : table(Table::Coils),
          start(0U),
          count(0U),
          sessionGeneration(0U),
          requiredSlots(0U),
          fireForget(false),
          publicDebt(false),
          accepted(false) {}
  };

  Journal()
      : coilHead_(0U),
        coilTail_(0U),
        holdingHead_(0U),
        holdingTail_(0U),
        sourceToken_(1U) {}

  uint8_t requiredSlots(Table table, uint16_t count) const {
    const size_t chunk = table == Table::Coils
        ? CoilSnapshotMax
        : HoldingSnapshotMax;
    const uint32_t slots =
        (static_cast<uint32_t>(count) + static_cast<uint32_t>(chunk - 1U)) /
        static_cast<uint32_t>(chunk);
    return static_cast<uint8_t>(slots == 0U ? 1U
        : slots > 255U ? 255U
        : slots);
  }

  bool reserve(Table table,
               uint16_t start,
               uint16_t count,
               bool fireForget,
               bool publicDebt,
               uint16_t sessionGeneration,
               Reservation& out) const {
    out.table = table;
    out.start = start;
    out.count = count;
    out.sessionGeneration = sessionGeneration;
    out.requiredSlots = requiredSlots(table, count);
    out.fireForget = fireForget;
    out.publicDebt = publicDebt;
    out.accepted = freeSlots(table) >= out.requiredSlots;
    return out.accepted;
  }

  bool append(const Reservation& reservation, const bool* values) {
    if (reservation.table != Table::Coils || !reservation.accepted ||
        values == 0 || freeSlots(Table::Coils) < reservation.requiredSlots) {
      return false;
    }
    return appendCoils(reservation, values);
  }

  bool append(const Reservation& reservation, const uint16_t* values) {
    if (reservation.table != Table::Holding || !reservation.accepted ||
        values == 0 || freeSlots(Table::Holding) < reservation.requiredSlots) {
      return false;
    }
    return appendHolding(reservation, values);
  }

  bool peek(Table table, Entry& out) const {
    const uint8_t head = table == Table::Coils ? coilHead_ : holdingHead_;
    const uint8_t tail = table == Table::Coils ? coilTail_ : holdingTail_;
    if (head == tail) {
      return false;
    }
    memcpy(&out, &sourceRing(table)[head], sizeof(out));
    return out.sourceToken != 0U;
  }

  bool peekNext(Entry& out, Table& table) const {
    const bool coilPresent = coilHead_ != coilTail_;
    const bool holdingPresent = holdingHead_ != holdingTail_;
    if (!coilPresent && !holdingPresent) {
      return false;
    }

    const uint16_t coilToken = coilPresent
        ? coilSource_[coilHead_].sourceToken
        : 0U;
    const uint16_t holdingToken = holdingPresent
        ? holdingSource_[holdingHead_].sourceToken
        : 0U;
    if (!holdingPresent ||
        (coilPresent && !tokenBefore(holdingToken, coilToken))) {
      table = Table::Coils;
      memcpy(&out, &coilSource_[coilHead_], sizeof(out));
    } else {
      table = Table::Holding;
      memcpy(&out, &holdingSource_[holdingHead_], sizeof(out));
    }
    return out.sourceToken != 0U;
  }

  bool commit(Table table, uint16_t sourceToken) {
    volatile uint8_t& head =
        table == Table::Coils ? coilHead_ : holdingHead_;
    const uint8_t tail = table == Table::Coils ? coilTail_ : holdingTail_;
    Entry* const ring = sourceRing(table);
    if (head == tail || sourceToken == 0U ||
        ring[head].sourceToken != sourceToken) {
      return false;
    }
    head = incrementIndex(head);
    return true;
  }

  uint8_t sourceSize(Table table) const {
    return usedSlots(table == Table::Coils ? coilHead_ : holdingHead_,
                     table == Table::Coils ? coilTail_ : holdingTail_);
  }

 private:
  static bool tokenBefore(uint16_t lhs, uint16_t rhs) {
    return lhs != 0U &&
        (rhs == 0U || static_cast<int16_t>(lhs - rhs) < 0);
  }

  static uint16_t advanceToken(uint16_t& token) {
    token = static_cast<uint16_t>(token + 1U);
    if (token == 0U) {
      token = 1U;
    }
    return token;
  }

  static uint8_t incrementIndex(uint8_t index) {
    return static_cast<uint8_t>((index + 1U) % RingCapacity);
  }

  static uint8_t usedSlots(uint8_t head, uint8_t tail) {
    return tail >= head
        ? static_cast<uint8_t>(tail - head)
        : static_cast<uint8_t>(RingCapacity - (head - tail));
  }

  uint8_t freeSlots(Table table) const {
    return static_cast<uint8_t>(
        (RingCapacity - 1U) - sourceSize(table));
  }

  Entry* sourceRing(Table table) {
    return table == Table::Coils ? coilSource_ : holdingSource_;
  }

  const Entry* sourceRing(Table table) const {
    return table == Table::Coils ? coilSource_ : holdingSource_;
  }

  bool appendCoils(const Reservation& reservation, const bool* values) {
    uint16_t remaining = reservation.count;
    uint16_t offset = 0U;
    while (remaining != 0U) {
      const uint16_t chunk = remaining > CoilSnapshotMax
          ? static_cast<uint16_t>(CoilSnapshotMax)
          : remaining;
      Entry& entry = coilSource_[coilTail_];
      fillHeader(entry, reservation,
                 static_cast<uint16_t>(reservation.start + offset), chunk);
      for (uint16_t index = 0U; index < chunk; ++index) {
        entry.snapshot.coils[index] = values[offset + index];
      }
      coilTail_ = incrementIndex(coilTail_);
      remaining = static_cast<uint16_t>(remaining - chunk);
      offset = static_cast<uint16_t>(offset + chunk);
    }
    return true;
  }

  bool appendHolding(const Reservation& reservation, const uint16_t* values) {
    uint16_t remaining = reservation.count;
    uint16_t offset = 0U;
    while (remaining != 0U) {
      const uint16_t chunk = remaining > HoldingSnapshotMax
          ? static_cast<uint16_t>(HoldingSnapshotMax)
          : remaining;
      Entry& entry = holdingSource_[holdingTail_];
      fillHeader(entry, reservation,
                 static_cast<uint16_t>(reservation.start + offset), chunk);
      for (uint16_t index = 0U; index < chunk; ++index) {
        entry.snapshot.holding[index] = values[offset + index];
      }
      holdingTail_ = incrementIndex(holdingTail_);
      remaining = static_cast<uint16_t>(remaining - chunk);
      offset = static_cast<uint16_t>(offset + chunk);
    }
    return true;
  }

  void fillHeader(Entry& entry,
                  const Reservation& reservation,
                  uint16_t start,
                  uint16_t count) {
    entry.start = start;
    entry.count = count;
    entry.sessionGeneration = reservation.sessionGeneration;
    entry.sourceToken = advanceToken(sourceToken_);
    entry.ops = 1U;
    entry.meta = static_cast<uint8_t>(
        (reservation.fireForget ? kFlagFireForget : 0U) |
        (reservation.publicDebt ? kFlagPublicDebt : 0U));
    entry.snapshotCount = static_cast<uint8_t>(count);
  }

  Entry coilSource_[RingCapacity];
  Entry holdingSource_[RingCapacity];
  volatile uint8_t coilHead_;
  volatile uint8_t coilTail_;
  volatile uint8_t holdingHead_;
  volatile uint8_t holdingTail_;
  uint16_t sourceToken_;
};

}  // namespace LegacyModbusIngressJournalReference
