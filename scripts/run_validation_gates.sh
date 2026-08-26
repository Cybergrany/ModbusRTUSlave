#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$script_dir/.." && pwd)
pio_core_dir=${PLATFORMIO_CORE_DIR:-$repo_root/../.platformio_core_portable}
host_cxx=${CXX:-g++}
avr_cxx=${AVR_CXX:-$pio_core_dir/packages/toolchain-atmelavr/bin/avr-g++}
strict_requested=${MBUS_RTU_SLAVE_STRICT_PERFORMANCE:-}
temporary_dir=$(mktemp -d /tmp/modbus-rtu-slave-gates.XXXXXX)

cleanup() {
  case "$temporary_dir" in
    /tmp/modbus-rtu-slave-gates.*) rm -rf "$temporary_dir" ;;
    *) echo "refusing to remove unexpected temporary path: $temporary_dir" >&2 ;;
  esac
}
trap cleanup EXIT

cd "$repo_root"

common_host_flags=(
  -std=gnu++17 -Os -Wall -Wextra -Werror
  -I src -I test/support
)
bridge_profile_flags=(
  -DMBUS_RTU_SLAVE_BRIDGE_MODE -DMBUS_RTU_SLAVE_USE_MUTEX
  '-DMBUS_RTU_SLAVE_MUTEX_HEADER="platform/PlatformLock.h"'
  -DMBUS_RTU_SLAVE_WORK_ACCESSORS -DMBUS_RTU_SLAVE_DIAGNOSTICS
  -DMBUS_RTU_SLAVE_EVENT_CALLBACKS
  -DMBUS_RTU_SLAVE_BRIDGE_TX_DIAGNOSTICS=1
)

"$host_cxx" "${common_host_flags[@]}" \
  -c test/support/ModbusRTUSlavePublicHeaderCompileGate.cpp \
  -o "$temporary_dir/public_header_standalone.o"
"$host_cxx" "${common_host_flags[@]}" "${bridge_profile_flags[@]}" \
  -c test/support/ModbusRTUSlavePublicHeaderCompileGate.cpp \
  -o "$temporary_dir/public_header_bridge.o"
"$host_cxx" "${common_host_flags[@]}" \
  -c src/ModbusRTUSlave.cpp \
  -o "$temporary_dir/package_slave_standalone.o"
"$host_cxx" "${common_host_flags[@]}" -DMBUS_RTU_SLAVE_EVENT_CALLBACKS \
  -c src/ModbusRTUSlave.cpp \
  -o "$temporary_dir/package_slave_event_callbacks.o"
"$host_cxx" "${common_host_flags[@]}" "${bridge_profile_flags[@]}" \
  -c src/ModbusRTUSlave.cpp \
  -o "$temporary_dir/package_slave_bridge.o"
echo "public header: standalone, event-callback, and bridge profiles compile"

env -u MBUS_RTU_SLAVE_STRICT_PERFORMANCE \
  -u MBUS_RTU_SLAVE_PERFORMANCE_ONLY \
  PLATFORMIO_CORE_DIR="$pio_core_dir" \
  pio test -e native_modbus_tests

"$host_cxx" -std=c++11 -O2 -Wall -Wextra -Werror -pedantic-errors \
  -I src -I test/support \
  test/oracles/ModbusRTUIngressJournalOracle.cpp \
  -o "$temporary_dir/modbus_ingress_oracle"
env -u MBUS_RTU_SLAVE_STRICT_PERFORMANCE \
  "$temporary_dir/modbus_ingress_oracle"

"$host_cxx" -std=c++11 -Os -Wall -Wextra -Werror -pedantic-errors \
  -I src -c test/support/ModbusRTUIngressJournalCompileGate.cpp \
  -o "$temporary_dir/modbus_ingress_host.o"

if [[ ! -x "$avr_cxx" ]]; then
  echo "AVR compiler is unavailable: $avr_cxx" >&2
  exit 2
fi
"$avr_cxx" -mmcu=atmega328p -std=c++11 -Os \
  -Wall -Wextra -Werror -pedantic-errors -I src \
  -c test/support/ModbusRTUIngressJournalCompileGate.cpp \
  -o "$temporary_dir/modbus_ingress_avr.o"

scripts/compare_modbus_native_perf.sh --self-test

if [[ -n "$strict_requested" ]]; then
  MBUS_RTU_SLAVE_STRICT_PERFORMANCE="$strict_requested" \
    "$temporary_dir/modbus_ingress_oracle"

  strict_runner=()
  if command -v taskset >/dev/null 2>&1; then
    strict_cpu=${MBUS_RTU_SLAVE_PERF_CPU:-}
    if [[ -z "$strict_cpu" ]]; then
      allowed_cpus=$(taskset -pc $$ | sed 's/^.*: //')
      strict_cpu=${allowed_cpus%%,*}
      strict_cpu=${strict_cpu%%-*}
    fi
    if [[ ! "$strict_cpu" =~ ^[0-9]+$ ]] ||
       ! taskset -c "$strict_cpu" true >/dev/null 2>&1; then
      echo "MBUS_RTU_SLAVE_PERF_CPU is not available to this process" >&2
      exit 2
    fi
    strict_runner=(taskset -c "$strict_cpu")
    echo "strict Modbus performance affinity: CPU $strict_cpu"
  elif [[ -n ${MBUS_RTU_SLAVE_PERF_CPU:-} ]]; then
    echo "MBUS_RTU_SLAVE_PERF_CPU requires taskset" >&2
    exit 2
  fi

  env -u MBUS_RTU_SLAVE_PERFORMANCE_ONLY \
    MBUS_RTU_SLAVE_STRICT_PERFORMANCE="$strict_requested" \
    "${strict_runner[@]}" .pio/build/native_modbus_tests/program
fi

echo "ModbusRTUSlave validation gates: PASS"
