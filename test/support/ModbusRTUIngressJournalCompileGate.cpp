#include <stdint.h>

#include "detail/ModbusRTUIngressJournal.h"

// Compile-only fixture: unlike an include-only smoke test, this instantiates
// and exercises the default journal's producer and consumer templates. The
// validation gate builds it with strict host C++11 and the AVR toolchain.
uint8_t exerciseModbusRTUIngressJournalCompileGate() {
  typedef ModbusRTU::FixedCapacityIngressJournal<4U, 3U, 5U> Journal;
  static_assert(
      sizeof(Journal::Reservation) ==
          (sizeof(void*) == 8U ? 16U : sizeof(void*) == 4U ? 12U : 10U),
      "Reservation gained fields or target padding on the write hot path");
  Journal journal;
  Journal::Reservation reservation;
  const uint16_t values[4] = {1U, 2U, 3U, 4U};
  if (!journal.reserve(Journal::Table::HoldingRegisters,
                       10U, 4U, 12U, 2U, 0x40U, reservation)) {
    return 1U;
  }
  if (!journal.publishHolding(reservation, values)) return 2U;

  Journal::Entry entry;
  Journal::Table table = Journal::Table::Coils;
  if (!journal.peekNext(entry, table)) return 3U;
  if (table != Journal::Table::HoldingRegisters) return 4U;
  if (entry.context != 12U || entry.units != 2U ||
      entry.attributes != 0x40U || entry.snapshot.holding[0] != 1U) {
    return 5U;
  }
  return journal.retire(table, entry.sourceToken) ? 0U : 6U;
}
