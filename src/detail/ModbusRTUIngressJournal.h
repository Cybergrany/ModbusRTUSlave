// Copyright (c) OpenGameMaster contributors.
// SPDX-License-Identifier: MIT
#ifndef MODBUS_RTU_INGRESS_JOURNAL_H
#define MODBUS_RTU_INGRESS_JOURNAL_H

#include <stdint.h>
#include <string.h>

// This journal sits on the acknowledged-write hot path. Keep its small leaf
// seams zero-cost in diagnostic/native builds that deliberately compile
// without optimisation. Optimized firmware builds retain ordinary inline
// semantics so the target compiler remains free to balance speed and size.
#if (defined(__GNUC__) || defined(__clang__)) && !defined(__OPTIMIZE__)
#define MODBUS_RTU_INGRESS_INLINE inline __attribute__((always_inline))
#else
#define MODBUS_RTU_INGRESS_INLINE inline
#endif

// Keep the acknowledged one-entry path in its caller even under -Os, while
// keeping the uncommon multi-entry implementation out of that caller. This
// pairing avoids a second large register-save frame for FC5/FC6 without
// cloning the multi-chunk loop into every protocol handler.
#if defined(__GNUC__) || defined(__clang__)
#define MODBUS_RTU_INGRESS_HOT_INLINE inline __attribute__((always_inline))
#define MODBUS_RTU_INGRESS_SLOW_PATH __attribute__((noinline))
#else
#define MODBUS_RTU_INGRESS_HOT_INLINE inline
#define MODBUS_RTU_INGRESS_SLOW_PATH
#endif

/**
 * Allocation-free transactional write journal for Modbus-style table writes.
 *
 * This file deliberately has no Arduino, RTOS, serial, pin-map, or OGM
 * dependencies.  A protocol parser can therefore reserve journal capacity,
 * mutate its register image, capture an immutable post-mutation snapshot, and
 * publish the write before acknowledging it without coupling this mechanism to
 * a particular firmware product.
 *
 * Concurrency contract
 * --------------------
 * Methods do not lock.  The owner must serialize calls that touch a journal.
 * This is a one-producer journal (the normal Modbus RTU slave arrangement).
 * If an application has multiple possible producer contexts, one owner lock
 * must serialize the complete reserve -> mutation -> publish interval. Merely
 * locking individual calls is too late to prevent two producers from mutating
 * against the same tail. Consumers may advance a head between reserve and
 * publish; that only creates more capacity and is safe.
 *
 * Transaction contract
 * --------------------
 *   1. reserve() proves that every required chunk fits and records the tail.
 *   2. The caller mutates its register image under its own table lock.
 *   3. publishCoils()/publishHolding() copies the values into entries.
 *   4. The tail is published once, after every entry is complete.
 *   5. Only then may the caller queue a Modbus acknowledgement.
 *
 * If the producer tail changed after reserve(), publish fails without exposing
 * a partial journal transaction. The caller must still treat that as a fatal
 * producer-contract error because its register mutation has already occurred.
 */
namespace ModbusRTU {

template <uint16_t MaxCoilSnapshot, uint16_t MaxHoldingSnapshot>
union FixedCapacityIngressSnapshot {
  bool coils[MaxCoilSnapshot];
  uint16_t holding[MaxHoldingSnapshot];

  // Do not clear the largest union member for every queue slot at startup.
  // snapshotCount identifies the initialized prefix after an entry publishes.
  FixedCapacityIngressSnapshot() {}
};

/** Default opaque record used by a standalone neutral journal. */
template <uint16_t MaxCoilSnapshot, uint16_t MaxHoldingSnapshot>
struct FixedCapacityIngressEntry {
  using Snapshot = FixedCapacityIngressSnapshot<MaxCoilSnapshot,
                                                 MaxHoldingSnapshot>;

  uint16_t start = 0U;
  uint16_t count = 0U;
  // Opaque owner-supplied correlation value, for example an admission epoch.
  uint16_t context = 0U;
  uint16_t sourceToken = 0U;
  // Opaque logical unit count and attribute bits. Their interpretation stays
  // in the product adapter rather than becoming journal business logic.
  uint8_t units = 1U;
  uint8_t attributes = 0U;
  uint8_t snapshotCount = 0U;
  Snapshot snapshot{};
};

/**
 * Default field adapter. Products with an established public record can supply
 * an equivalent traits type and let the journal store that record directly,
 * without unsafe type punning or a second copy.
 */
template <uint16_t MaxCoilSnapshot, uint16_t MaxHoldingSnapshot>
struct FixedCapacityIngressEntryTraits {
  using Entry = FixedCapacityIngressEntry<MaxCoilSnapshot,
                                          MaxHoldingSnapshot>;

  static MODBUS_RTU_INGRESS_INLINE uint16_t sourceToken(const Entry& entry) {
    return entry.sourceToken;
  }

  static MODBUS_RTU_INGRESS_INLINE void initialize(
      Entry& entry, uint16_t start, uint16_t count, uint16_t context,
      uint16_t sourceToken, uint8_t units, uint8_t attributes,
      uint8_t snapshotCount) {
    entry.start = start;
    entry.count = count;
    entry.context = context;
    entry.sourceToken = sourceToken;
    entry.units = units;
    entry.attributes = attributes;
    entry.snapshotCount = snapshotCount;
  }

  static MODBUS_RTU_INGRESS_INLINE void copyCoils(
      Entry& entry, const bool* values, uint16_t count) {
    if (count == 1U) {
      entry.snapshot.coils[0] = values[0];
      return;
    }
    memcpy(entry.snapshot.coils, values,
           static_cast<size_t>(count) * sizeof(bool));
  }

  static MODBUS_RTU_INGRESS_INLINE void copyHolding(
      Entry& entry, const uint16_t* values, uint16_t count) {
    if (count == 1U) {
      entry.snapshot.holding[0] = values[0];
      return;
    }
    memcpy(entry.snapshot.holding, values,
           static_cast<size_t>(count) * sizeof(uint16_t));
  }
};

template <uint16_t MaxCoilSnapshot,
          uint16_t MaxHoldingSnapshot,
          uint8_t RingCapacity,
          typename EntryTraits = FixedCapacityIngressEntryTraits<
              MaxCoilSnapshot, MaxHoldingSnapshot> >
class FixedCapacityIngressJournal {
 public:
  // EntryTraits is an optional zero-cost compatibility seam. It must provide
  // Entry plus sourceToken(), initialize(), copyCoils(), and copyHolding().
  // Entry must be standard-layout and trivially copyable because peek()
  // snapshots it with memcpy. These requirements are tested by host compile
  // gates rather than <type_traits> here because the supported AVR library
  // lacks that file.
  static_assert(MaxCoilSnapshot > 0U,
                "A coil journal needs a non-zero snapshot capacity");
  static_assert(MaxHoldingSnapshot > 0U,
                "A holding-register journal needs a non-zero snapshot capacity");
  static_assert(MaxCoilSnapshot <= 0xFFU,
                "Entry::snapshotCount is an 8-bit wire-adjacent field");
  static_assert(MaxHoldingSnapshot <= 0xFFU,
                "Entry::snapshotCount is an 8-bit wire-adjacent field");
  static_assert(RingCapacity > 1U,
                "A sentinel-slot ring needs at least two entries");

  enum class Table : uint8_t {
    Coils = 0U,
    HoldingRegisters = 1U,
  };

  using Snapshot = FixedCapacityIngressSnapshot<MaxCoilSnapshot,
                                                 MaxHoldingSnapshot>;
  using Entry = typename EntryTraits::Entry;

  FixedCapacityIngressJournal() = default;
  FixedCapacityIngressJournal(const FixedCapacityIngressJournal&) = delete;
  FixedCapacityIngressJournal& operator=(
      const FixedCapacityIngressJournal&) = delete;
  FixedCapacityIngressJournal(FixedCapacityIngressJournal&&) = delete;
  FixedCapacityIngressJournal& operator=(
      FixedCapacityIngressJournal&&) = delete;

  /**
   * Stack-owned proof that an entire write fitted before mutation began.
   *
   * Reservations do not allocate and do not make entries visible.  They are
   * valid only for the journal instance that produced them and while the
   * recorded producer tail remains unchanged. A reservation is a one-shot,
   * immediate producer hand-off: only one may be outstanding, and callers must
   * not retain it or attempt to publish it again after a successful publish.
   */
  class Reservation {
   public:
    // A failed reserve() only needs an invalid entry marker. Leaving the
    // remaining ticket fields unspecified until reserve succeeds avoids
    // clearing the whole stack object on every acknowledged single write.
    // Accessors are therefore valid only after reserve() returns true.
    Reservation() : entry_(nullptr) {}

    MODBUS_RTU_INGRESS_INLINE uint16_t start() const { return start_; }
    MODBUS_RTU_INGRESS_INLINE uint16_t count() const { return count_; }
    MODBUS_RTU_INGRESS_INLINE uint16_t context() const { return context_; }
    MODBUS_RTU_INGRESS_INLINE uint8_t units() const { return units_; }
    MODBUS_RTU_INGRESS_INLINE uint8_t attributes() const {
      return attributes_;
    }

   private:
    friend class FixedCapacityIngressJournal;

    const Entry* entry_;
    uint16_t start_;
    uint16_t count_;
    uint16_t context_;
    uint8_t units_;
    uint8_t attributes_;
  };

  static MODBUS_RTU_INGRESS_INLINE constexpr uint16_t maxChunk(Table table) {
    return table == Table::Coils ? MaxCoilSnapshot : MaxHoldingSnapshot;
  }

  static MODBUS_RTU_INGRESS_INLINE uint8_t requiredSlots(
      Table table, uint16_t count) {
    if (count == 0U) return 0U;
    const uint16_t chunk = maxChunk(table);
    if (count <= chunk) return 1U;
    const uint32_t slots =
        (static_cast<uint32_t>(count) + static_cast<uint32_t>(chunk - 1U)) /
        static_cast<uint32_t>(chunk);
    return slots > 0xFFU ? 0xFFU : static_cast<uint8_t>(slots);
  }

  static MODBUS_RTU_INGRESS_INLINE uint16_t operationCount(
      Table table, uint16_t count) {
    const uint8_t slots = requiredSlots(table, count);
    return slots == 0U ? uint16_t(1U) : static_cast<uint16_t>(slots);
  }

  MODBUS_RTU_INGRESS_INLINE uint8_t freeSlots(Table table) const {
    const Ring& ring = ringFor(table);
    const uint8_t head = ring.head;
    const uint8_t tail = ring.tail;
    const uint8_t used = tail >= head
        ? static_cast<uint8_t>(tail - head)
        : static_cast<uint8_t>(RingCapacity - (head - tail));
    return static_cast<uint8_t>((RingCapacity - 1U) - used);
  }

  /**
   * Prepare an all-or-nothing write.  Failure leaves both rings unchanged.
   * The owner may publish its own overflow/rejection diagnostic afterwards.
   */
  bool reserve(
      Table table, uint16_t start, uint16_t count, uint16_t context,
      uint8_t units, uint8_t attributes, Reservation& out) const {
    const uint8_t slots = requiredSlots(table, count);
    const uint32_t endExclusive =
        static_cast<uint32_t>(start) + static_cast<uint32_t>(count);
    if (slots == 0U || endExclusive > 0x10000UL ||
        freeSlots(table) < slots) {
      // Clear only the identity marker. All other fields are deliberately
      // unspecified after a failed reservation and cannot pass publication.
      out.entry_ = nullptr;
      return false;
    }

    const Ring& ring = ringFor(table);
    // The exact reserved slot identifies journal, table, and producer tail at
    // once. Publication can therefore reject foreign/stale tickets with one
    // comparison and use the slot directly on the one-entry hot path.
    out.entry_ = &ring.entries[ring.tail];
    out.start_ = start;
    out.count_ = count;
    out.context_ = context;
    out.units_ = units;
    out.attributes_ = attributes;
    return true;
  }

  /** Publish a coil reservation from values[0..reservation.count()). */
  MODBUS_RTU_INGRESS_HOT_INLINE bool publishCoils(
      const Reservation& reservation, const bool* values) {
    if (!values || !reservationMatches(reservation, Table::Coils)) return false;
    return reservation.count_ <= MaxCoilSnapshot
        ? publishOne<Table::Coils>(reservation, values)
        : publishMultiple<Table::Coils>(reservation, values);
  }

  /** Publish a holding reservation from values[0..reservation.count()). */
  MODBUS_RTU_INGRESS_HOT_INLINE bool publishHolding(
      const Reservation& reservation, const uint16_t* values) {
    if (!values ||
        !reservationMatches(reservation, Table::HoldingRegisters)) return false;
    return reservation.count_ <= MaxHoldingSnapshot
        ? publishOne<Table::HoldingRegisters>(reservation, values)
        : publishMultiple<Table::HoldingRegisters>(reservation, values);
  }

  MODBUS_RTU_INGRESS_HOT_INLINE bool peek(Table table, Entry& out) const {
    const Ring& ring = ringFor(table);
    const uint8_t head = ring.head;
    if (head == ring.tail) return false;
    memcpy(&out, &ring.entries[head], sizeof(out));
    return EntryTraits::sourceToken(out) != 0U;
  }

  /**
   * Return the oldest head across the two table rings.
   *
   * Ordering uses 16-bit serial-number arithmetic. Outstanding entries must
   * remain within half the token range (32767); a bounded ring naturally does
   * so when consumers drain through peekNext() plus exact-token retire(). Do
   * not indefinitely drain only one table while retaining an ancient head in
   * the other table.
   */
  MODBUS_RTU_INGRESS_HOT_INLINE bool peekNext(
      Entry& out, Table& table) const {
    const uint8_t coilHead = coils_.head;
    const uint8_t holdingHead = holding_.head;
    const bool coilPresent = coilHead != coils_.tail;
    const bool holdingPresent = holdingHead != holding_.tail;
    if (!coilPresent && !holdingPresent) return false;

    const uint16_t coilToken =
        coilPresent ? EntryTraits::sourceToken(coils_.entries[coilHead]) : 0U;
    const uint16_t holdingToken =
        holdingPresent
            ? EntryTraits::sourceToken(holding_.entries[holdingHead])
            : 0U;
    table = (!holdingPresent ||
             (coilPresent && !tokenBefore(holdingToken, coilToken)))
        ? Table::Coils
        : Table::HoldingRegisters;
    return peek(table, out);
  }

  /** Advance only when token still identifies the exact visible head. */
  MODBUS_RTU_INGRESS_HOT_INLINE bool retire(Table table, uint16_t token) {
    Ring& ring = ringFor(table);
    const uint8_t head = ring.head;
    if (head == ring.tail || token == 0U ||
        EntryTraits::sourceToken(ring.entries[head]) != token) {
      return false;
    }
    ring.head = advance(head);
    return true;
  }

  static MODBUS_RTU_INGRESS_INLINE constexpr bool tokenBefore(
      uint16_t lhs, uint16_t rhs) {
    // Compare the forward distance without converting an out-of-range unsigned
    // value to int16_t (which is implementation-defined in C++11). Exactly a
    // half-range apart is deliberately unordered; the documented bounded-ring
    // contract prevents that ambiguous state.
    return lhs != 0U &&
        (rhs == 0U ||
         (static_cast<uint16_t>(rhs - lhs) != 0U &&
          static_cast<uint16_t>(rhs - lhs) < 0x8000U));
  }

 private:
  struct Ring {
    Entry entries[RingCapacity];
    volatile uint8_t head = 0U;
    volatile uint8_t tail = 0U;
  };

  // Default-initialize rather than value-initialize the rings. Entry payloads
  // are deliberately unspecified until their tail is published, so clearing
  // every snapshot here would add boot work and target flash with no safety
  // benefit. Ring's head/tail member initializers still establish emptiness.
  Ring coils_;
  Ring holding_;
  uint16_t nextToken_ = 1U;

  static MODBUS_RTU_INGRESS_INLINE uint8_t advance(uint8_t index) {
    return static_cast<uint8_t>((static_cast<uint16_t>(index) + 1U) %
                                RingCapacity);
  }

  MODBUS_RTU_INGRESS_INLINE Ring& ringFor(Table table) {
    return table == Table::Coils ? coils_ : holding_;
  }

  MODBUS_RTU_INGRESS_INLINE const Ring& ringFor(Table table) const {
    return table == Table::Coils ? coils_ : holding_;
  }

  MODBUS_RTU_INGRESS_HOT_INLINE bool reservationMatches(
      const Reservation& reservation, Table expectedTable) const {
    const Ring& ring = ringFor(expectedTable);
    // A matching slot identity can only be written by a successful reserve(),
    // which already rejected a zero count. Avoid rechecking that invariant on
    // every acknowledged write.
    return reservation.entry_ == &ring.entries[ring.tail];
  }

  MODBUS_RTU_INGRESS_INLINE uint16_t nextToken() {
    uint16_t token = static_cast<uint16_t>(nextToken_ + 1U);
    if (token == 0U) token = 1U;
    nextToken_ = token;
    return token;
  }

  template <Table ExpectedTable, typename Value>
  MODBUS_RTU_INGRESS_HOT_INLINE bool publishOne(
      const Reservation& reservation, const Value* values) {
    Ring& ring = ringFor(ExpectedTable);
    const uint8_t tail = ring.tail;
    Entry& entry = *const_cast<Entry*>(reservation.entry_);
    EntryTraits::initialize(
        entry, reservation.start_, reservation.count_, reservation.context_,
        nextToken(), reservation.units_, reservation.attributes_,
        static_cast<uint8_t>(reservation.count_));
    copySnapshot(entry, values, reservation.count_);
    ring.tail = advance(tail);
    return true;
  }

  template <Table ExpectedTable, typename Value>
  MODBUS_RTU_INGRESS_SLOW_PATH bool publishMultiple(
      const Reservation& reservation, const Value* values) {
    Ring& ring = ringFor(ExpectedTable);
    uint8_t tail = ring.tail;
    uint16_t remaining = reservation.count_;
    const uint16_t chunkLimit = maxChunk(ExpectedTable);
    uint16_t valueOffset = 0U;
    uint16_t entryStart = reservation.start_;

    // Complete every entry using a local tail.  The consumer cannot observe a
    // transaction until the single tail publication after this loop.
    while (remaining != 0U) {
      const uint16_t chunk = remaining > chunkLimit ? chunkLimit : remaining;
      Entry& entry = ring.entries[tail];
      EntryTraits::initialize(
          entry, entryStart, chunk, reservation.context_, nextToken(),
          reservation.units_, reservation.attributes_,
          static_cast<uint8_t>(chunk));
      copySnapshot(entry, values + valueOffset, chunk);

      tail = advance(tail);
      remaining = static_cast<uint16_t>(remaining - chunk);
      valueOffset = static_cast<uint16_t>(valueOffset + chunk);
      entryStart = static_cast<uint16_t>(entryStart + chunk);
    }

    ring.tail = tail;
    return true;
  }

  static MODBUS_RTU_INGRESS_HOT_INLINE void copySnapshot(
      Entry& entry, const bool* values, uint16_t count) {
    EntryTraits::copyCoils(entry, values, count);
  }

  static MODBUS_RTU_INGRESS_HOT_INLINE void copySnapshot(
      Entry& entry, const uint16_t* values, uint16_t count) {
    EntryTraits::copyHolding(entry, values, count);
  }
};

}  // namespace ModbusRTU

#undef MODBUS_RTU_INGRESS_INLINE
#undef MODBUS_RTU_INGRESS_HOT_INLINE
#undef MODBUS_RTU_INGRESS_SLOW_PATH

#endif  // MODBUS_RTU_INGRESS_JOURNAL_H
