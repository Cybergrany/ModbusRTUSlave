#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$script_dir/.." && pwd)
reference=${1:-}

expected_source_commit=73925642c29a0f419b2b3cb160647dee71f4c078
rx_framing_todo_commit=0ad325f333204225468d1f7c4ae65a408a1bf87b

check_hash() {
  local relative_path=$1
  local expected=$2
  local actual
  actual=$(sha256sum "$repo_root/$relative_path" | awk '{print $1}')
  if [[ "$actual" != "$expected" ]]; then
    echo "$relative_path differs from the reviewed public-layout candidate: $actual" >&2
    exit 1
  fi
  printf '%s  %s\n' "$actual" "$relative_path"
}

check_hash src/ModbusRTUSlave.h \
  c53562fe91a353b4bbbbf7febd65f826d700d3900f6e6431cc5ec57fa054e687
check_hash src/ModbusRTUSlave.cpp \
  506257c11f891e5a839f177dda96b92ffd439232336e082ced3a444486d4c30f
check_hash src/detail/ModbusRTUIngressJournal.h \
  f9d08a82db5f349610db43115089c9e916e3a59024d0d189ebcbbe9e8eec381e

if find "$repo_root/src/Comms" -type f -print -quit 2>/dev/null | grep -q .; then
  echo "legacy src/Comms compatibility files remain" >&2
  exit 1
fi

if ! grep -Fq 'TODO(rx-framing): Stream exposes queued bytes, not their physical arrival times.' \
    "$repo_root/src/ModbusRTUSlave.cpp"; then
  echo "RX framing TODO from $rx_framing_todo_commit is missing" >&2
  exit 1
fi

if [[ -n "$reference" ]]; then
  reference=$(cd "$reference" && pwd)
  actual_commit=$(git -C "$reference" rev-parse HEAD)
  if [[ "$actual_commit" != "$expected_source_commit" ]]; then
    echo "reference must be OGM_slave_core $expected_source_commit; got $actual_commit" >&2
    exit 2
  fi

  temporary_dir=$(mktemp -d /tmp/ogm-modbus-slave-layout.XXXXXX)
  cleanup() {
    case "$temporary_dir" in
      /tmp/ogm-modbus-slave-layout.*) rm -rf "$temporary_dir" ;;
      *) echo "refusing to remove unexpected temporary path: $temporary_dir" >&2 ;;
    esac
  }
  trap cleanup EXIT

  sed \
    -e 's|#include "detail/ModbusRTUIngressJournal.h"|#include "Comms/ModbusRTUIngressJournal.h"|' \
    -e 's|PlatformMutex\* diMut,$|PlatformMutex* diMut, |' \
    -e 's|void poll();$|void poll(); |' \
    "$repo_root/src/ModbusRTUSlave.h" > "$temporary_dir/ModbusRTUSlave.h"
  sed \
    -e 's|#include "ModbusRTUSlave.h"|#include "Comms/ModbusRTUSlave.h"|' \
    -e '/^  \/\/ TODO(rx-framing):/,/^  \/\/ suppress the CRC statistic\.$/d' \
    "$repo_root/src/ModbusRTUSlave.cpp" > "$temporary_dir/ModbusRTUSlave.cpp"

  cmp "$temporary_dir/ModbusRTUSlave.h" \
      "$reference/include/Comms/ModbusRTUSlave.h"
  cmp "$temporary_dir/ModbusRTUSlave.cpp" \
      "$reference/src/Comms/ModbusRTUSlave.cpp"
  cmp "$repo_root/src/detail/ModbusRTUIngressJournal.h" \
      "$reference/include/Comms/ModbusRTUIngressJournal.h"
  echo "normalized source delta: canonical paths, whitespace cleanup and RX TODO only"
fi

echo "reviewed OGM public-layout migration: PASS"
