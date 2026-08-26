#include <ModbusRTUSlave.h>

// Compile-only API smoke test. The validation script builds this same source
// once as a standalone Arduino library and once with the bridge profile.
static_assert(ModbusRTUSlave::FC69_OFFSET_INNER_FC == 3U,
              "the public targeted-broadcast API changed");

ModbusRTUSlave* modbusRTUSlavePublicApiPointer() {
  return nullptr;
}
