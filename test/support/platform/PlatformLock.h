#pragma once

// Test-only alternate adapter name. The public library deliberately does not
// own this path; applications select whatever header provides the three mutex
// types documented in src/README.md.
#include "PlatformMutex.h"
