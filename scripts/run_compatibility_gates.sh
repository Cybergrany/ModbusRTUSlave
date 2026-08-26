#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$script_dir/.." && pwd)
pio_core_dir=${PLATFORMIO_CORE_DIR:-$repo_root/../.platformio_core_portable}
host_cxx=${CXX:-g++}
avr_cxx=${AVR_CXX:-$pio_core_dir/packages/toolchain-atmelavr/bin/avr-g++}
strict_requested=${OGM_STRICT_MODBUS_PERF:-}
temporary_dir=$(mktemp -d /tmp/ogm-modbus-ingress-gates.XXXXXX)

cleanup() {
  case "$temporary_dir" in
    /tmp/ogm-modbus-ingress-gates.*) rm -rf "$temporary_dir" ;;
    *) echo "refusing to remove unexpected temporary path: $temporary_dir" >&2 ;;
  esac
}
trap cleanup EXIT

cd "$repo_root"

scripts/check_ogm_source_parity.sh ${OGM_SLAVE_CORE_REFERENCE:+"$OGM_SLAVE_CORE_REFERENCE"}

if [[ -n ${OGM_SLAVE_CORE_REFERENCE:-} ]]; then
  "$host_cxx" -std=gnu++17 -Os -Wall -Wextra -Werror \
    -I "$OGM_SLAVE_CORE_REFERENCE/include" \
    -I "$OGM_SLAVE_CORE_REFERENCE/test/support" \
    -DOGM_BRIDGE_MODE -DOGM_USE_MUTEX -DOGM_MODBUS_MT_ACCESSORS \
    -DUSB_DEBUG -DBRIDGE_UPSTREAM_TX_DIAG=1 \
    -c "$OGM_SLAVE_CORE_REFERENCE/src/Comms/ModbusRTUSlave.cpp" \
    -o "$temporary_dir/embedded_slave.o"
  "$host_cxx" -std=gnu++17 -Os -Wall -Wextra -Werror \
    -I src -I test/support \
    -DOGM_BRIDGE_MODE -DOGM_USE_MUTEX -DOGM_MODBUS_MT_ACCESSORS \
    -DUSB_DEBUG -DBRIDGE_UPSTREAM_TX_DIAG=1 \
    -c src/Comms/ModbusRTUSlave.cpp \
    -o "$temporary_dir/package_slave.o"
  cmp "$temporary_dir/embedded_slave.o" "$temporary_dir/package_slave.o"
  echo "optimized bridge-mode object parity: exact"
fi

env -u OGM_STRICT_MODBUS_PERF -u OGM_MODBUS_PERF_ONLY \
  PLATFORMIO_CORE_DIR="$pio_core_dir" \
  pio test -e native_modbus_tests

"$host_cxx" -std=c++11 -O2 -Wall -Wextra -Werror -pedantic-errors \
  -I src -I test/support \
  test/oracles/ModbusRTUIngressJournalOracle.cpp \
  -o "$temporary_dir/modbus_ingress_oracle"
env -u OGM_STRICT_MODBUS_PERF "$temporary_dir/modbus_ingress_oracle"

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
scripts/compare_modbus_forwarded_pio_perf.sh --self-test

if [[ -n "$strict_requested" ]]; then
  OGM_STRICT_MODBUS_PERF="$strict_requested" \
    "$temporary_dir/modbus_ingress_oracle"

  strict_runner=()
  if command -v taskset >/dev/null 2>&1; then
    strict_cpu=${OGM_MODBUS_PERF_CPU:-}
    if [[ -z "$strict_cpu" ]]; then
      allowed_cpus=$(taskset -pc $$ | sed 's/^.*: //')
      strict_cpu=${allowed_cpus%%,*}
      strict_cpu=${strict_cpu%%-*}
    fi
    if [[ ! "$strict_cpu" =~ ^[0-9]+$ ]] ||
       ! taskset -c "$strict_cpu" true >/dev/null 2>&1; then
      echo "OGM_MODBUS_PERF_CPU is not available to this process" >&2
      exit 2
    fi
    strict_runner=(taskset -c "$strict_cpu")
    echo "strict Modbus performance affinity: CPU $strict_cpu"
  elif [[ -n ${OGM_MODBUS_PERF_CPU:-} ]]; then
    echo "OGM_MODBUS_PERF_CPU requires taskset" >&2
    exit 2
  fi

  env -u OGM_MODBUS_PERF_ONLY \
    OGM_STRICT_MODBUS_PERF="$strict_requested" \
    "${strict_runner[@]}" \
    .pio/build/native_modbus_tests/program
fi

if [[ -n ${OGM_MODBUS_BASELINE_WORKTREE:-} ]]; then
  PLATFORMIO_CORE_DIR="$pio_core_dir" \
    scripts/compare_modbus_forwarded_pio_perf.sh \
      "$OGM_MODBUS_BASELINE_WORKTREE" "$repo_root"
fi

echo "ModbusRTUSlave compatibility gates: PASS"
