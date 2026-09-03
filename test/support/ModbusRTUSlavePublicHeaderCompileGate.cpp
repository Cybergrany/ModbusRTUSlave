#include <ModbusRTUSlave.h>

// Compile-only API smoke test. The validation script builds this same source
// once as a standalone Arduino library and once with the bridge profile.
static_assert(ModbusRTUSlave::FC69_OFFSET_INNER_FC == 3U,
              "the public targeted-broadcast API changed");

bool modbusRTUSlaveCustomCandidate(const uint8_t*, uint16_t) {
  return false;
}

void configureModbusRTUSlaveCustomCandidate(ModbusRTUSlave& slave) {
  slave.setAdditionalFrameCandidateFn(&modbusRTUSlaveCustomCandidate);
}

ModbusRTUSlave* modbusRTUSlavePublicApiPointer() {
  return nullptr;
}
