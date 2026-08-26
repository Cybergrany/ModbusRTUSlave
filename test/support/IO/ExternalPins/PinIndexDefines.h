#pragma once

#include <cstdint>

// Production defaults from OGM_Portable. Keeping the native fixture aligned
// makes queue chunking and object-footprint characterisation meaningful.
namespace PinIndexDefines {
static constexpr uint16_t MAX_MULTI_COILS = 64;
static constexpr uint16_t MAX_MULTI_HRS = 32;
} // namespace PinIndexDefines
