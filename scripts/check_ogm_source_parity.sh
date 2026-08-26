#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$script_dir/.." && pwd)
reference=${1:-}

expected_source_commit=73925642c29a0f419b2b3cb160647dee71f4c078

check_hash() {
  local relative_path=$1
  local expected=$2
  local actual
  actual=$(sha256sum "$repo_root/$relative_path" | awk '{print $1}')
  if [[ "$actual" != "$expected" ]]; then
    echo "$relative_path differs from the frozen OGM source: $actual" >&2
    exit 1
  fi
  printf '%s  %s\n' "$actual" "$relative_path"
}

check_hash src/Comms/ModbusRTUSlave.h \
  cc6145185e889a344bfabd7dbb3a591fb5e67cc4f0490b02091e2006ff576c3d
check_hash src/Comms/ModbusRTUSlave.cpp \
  7844e6f6a1f8a8818f3044af5db07847dc244aa5ae4f2cfa46b73538b68c18aa
check_hash src/Comms/ModbusRTUIngressJournal.h \
  f9d08a82db5f349610db43115089c9e916e3a59024d0d189ebcbbe9e8eec381e
check_hash src/ModbusRTUSlave.h \
  9b862a9ab11cea6856715885dc9bb69375c35bae87672d3e027e79a8e58d6c14

if [[ -n "$reference" ]]; then
  reference=$(cd "$reference" && pwd)
  actual_commit=$(git -C "$reference" rev-parse HEAD)
  if [[ "$actual_commit" != "$expected_source_commit" ]]; then
    echo "reference must be OGM_slave_core $expected_source_commit; got $actual_commit" >&2
    exit 2
  fi

  cmp "$repo_root/src/Comms/ModbusRTUSlave.h" \
      "$reference/include/Comms/ModbusRTUSlave.h"
  cmp "$repo_root/src/Comms/ModbusRTUSlave.cpp" \
      "$reference/src/Comms/ModbusRTUSlave.cpp"
  cmp "$repo_root/src/Comms/ModbusRTUIngressJournal.h" \
      "$reference/include/Comms/ModbusRTUIngressJournal.h"
  echo "reference source parity: exact ($expected_source_commit)"
fi

echo "frozen OGM source parity: PASS"
