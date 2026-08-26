#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 BASELINE_PROGRAM CANDIDATE_PROGRAM [ODD_SAMPLE_COUNT]" >&2
  echo "       $0 --self-test" >&2
  echo "compares interleaved same-host medians; default regression limit is 5%" >&2
}

parse_measurement() {
  local label=$1
  local output=$2
  local perf_line legacy_line
  local format=legacy
  local idle_total=0 idle_ops=0 request_total=0 request_ops=0
  local idle_per_op= request_per_op=

  perf_line=$(printf '%s\n' "$output" | sed -n '/^modbus_perf: /p' | tail -n 1)
  if [[ -n "$perf_line" ]]; then
    if [[ "$perf_line" =~ idle_median_ns=([0-9]+)[[:space:]]+idle_ops=([0-9]+)[[:space:]]+request_median_ns=([0-9]+)[[:space:]]+request_ops=([0-9]+) ]]; then
      format=raw
      idle_total=${BASH_REMATCH[1]}
      idle_ops=${BASH_REMATCH[2]}
      request_total=${BASH_REMATCH[3]}
      request_ops=${BASH_REMATCH[4]}
    else
      echo "$label emitted a malformed modbus_perf line" >&2
      return 2
    fi
    if (( idle_ops == 0 || request_ops == 0 )); then
      echo "$label emitted zero operation count in modbus_perf output" >&2
      return 2
    fi
  fi

  legacy_line=$(printf '%s\n' "$output" | sed -n '/^characterization: /p' | tail -n 1)
  if [[ "$legacy_line" =~ idle=([0-9]+)ns/op[[:space:]]+requests=([0-9]+)ns/op ]]; then
    idle_per_op=${BASH_REMATCH[1]}
    request_per_op=${BASH_REMATCH[2]}
  elif [[ "$format" == raw ]]; then
    # Raw-only future programs can still participate in a mixed-format
    # comparison. Match the legacy executable's integer truncation.
    idle_per_op=$((idle_total / idle_ops))
    request_per_op=$((request_total / request_ops))
  else
    echo "$label emitted neither modbus_perf nor characterization performance output" >&2
    return 2
  fi

  printf '%s %s %s %s %s %s %s\n' \
    "$format" "$idle_total" "$idle_ops" "$request_total" "$request_ops" \
    "$idle_per_op" "$request_per_op"
}

measure() {
  local label=$1
  local program=$2
  local output parsed
  local status
  set +e
  output=$(env -u OGM_STRICT_MODBUS_PERF "$program" 2>&1)
  status=$?
  set -e
  if (( status != 0 )); then
    echo "$label test program failed with status $status" >&2
    echo "$output" >&2
    return "$status"
  fi

  if ! parsed=$(parse_measurement "$label" "$output"); then
    echo "$label did not emit parseable Modbus performance output" >&2
    echo "$output" >&2
    return 2
  fi
  printf '%s\n' "$parsed"
}

comparison_mode() {
  local baseline_format=$1
  local candidate_format=$2
  if [[ "$baseline_format" == raw && "$candidate_format" == raw ]]; then
    printf 'raw\n'
  else
    printf 'legacy\n'
  fi
}

raw_counts_match() {
  [[ "$1" == "$3" && "$2" == "$4" ]]
}

median() {
  local position=$((($# + 1) / 2))
  printf '%s\n' "$@" | sort -n | sed -n "${position}p"
}

report_raw_ratio() {
  local name=$1
  local baseline=$2
  local candidate=$3
  awk -v name="$name" -v baseline="$baseline" -v candidate="$candidate" \
    'BEGIN {
       delta = ((candidate / baseline) - 1.0) * 100.0;
       printf "%s: baseline_total=%dns candidate_total=%dns delta=%+.2f%%\n",
              name, baseline, candidate, delta;
     }'
}

report_legacy_ratio() {
  local name=$1
  local baseline=$2
  local candidate=$3
  awk -v name="$name" -v baseline="$baseline" -v candidate="$candidate" \
    'BEGIN {
       delta = ((candidate / baseline) - 1.0) * 100.0;
       printf "%s: baseline=%dns/op candidate=%dns/op delta=%+.2f%%\n",
              name, baseline, candidate, delta;
     }'
}

assert_equal() {
  local expected=$1
  local actual=$2
  local label=$3
  if [[ "$actual" != "$expected" ]]; then
    echo "self-test failed ($label): expected '$expected', got '$actual'" >&2
    exit 1
  fi
}

run_self_test() {
  local raw_both legacy_only raw_only parsed
  raw_both=$'noise\nmodbus_perf: idle_median_ns=165 idle_ops=10 request_median_ns=6655 request_ops=10\ncharacterization: idle=16ns/op requests=665ns/op object=1B pending=1B'
  legacy_only='characterization: idle=17ns/op requests=681ns/op object=1B pending=1B'
  raw_only='modbus_perf: idle_median_ns=199 idle_ops=10 request_median_ns=7019 request_ops=10'

  parsed=$(parse_measurement raw-both "$raw_both")
  assert_equal 'raw 165 10 6655 10 16 665' "$parsed" 'raw plus legacy parse'
  parsed=$(parse_measurement legacy-only "$legacy_only")
  assert_equal 'legacy 0 0 0 0 17 681' "$parsed" 'legacy parse'
  parsed=$(parse_measurement raw-only "$raw_only")
  assert_equal 'raw 199 10 7019 10 19 701' "$parsed" 'raw-only legacy derivation'
  assert_equal raw "$(comparison_mode raw raw)" 'raw mode selection'
  assert_equal legacy "$(comparison_mode legacy raw)" 'mixed mode selection'
  assert_equal legacy "$(comparison_mode raw legacy)" 'reverse mixed mode selection'
  if ! raw_counts_match 10 20 10 20; then
    echo 'self-test failed: equal raw operation counts were rejected' >&2
    exit 1
  fi
  if raw_counts_match 10 20 11 20; then
    echo 'self-test failed: unequal raw operation counts were accepted' >&2
    exit 1
  fi
  if parse_measurement malformed 'no performance output' >/dev/null 2>&1; then
    echo 'self-test failed: malformed output was accepted' >&2
    exit 1
  fi
  if parse_measurement malformed-raw \
      $'modbus_perf: malformed\ncharacterization: idle=17ns/op requests=681ns/op' \
      >/dev/null 2>&1; then
    echo 'self-test failed: malformed raw output silently fell back to legacy' >&2
    exit 1
  fi
  echo 'compare_modbus_native_perf self-test: PASS'
}

if [[ ${1:-} == --self-test ]]; then
  if [[ $# -ne 1 ]]; then
    usage
    exit 2
  fi
  run_self_test
  exit 0
fi

if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage
  exit 2
fi

baseline_program=$1
candidate_program=$2
sample_count=${3:-7}
max_regression_pct=${OGM_MODBUS_PERF_MAX_REGRESSION_PCT:-5}

if [[ ! -x "$baseline_program" ]]; then
  echo "baseline program is not executable: $baseline_program" >&2
  exit 2
fi
if [[ ! -x "$candidate_program" ]]; then
  echo "candidate program is not executable: $candidate_program" >&2
  exit 2
fi
if [[ ! "$sample_count" =~ ^[0-9]+$ ]] || (( sample_count < 3 || sample_count % 2 == 0 )); then
  echo "ODD_SAMPLE_COUNT must be an odd integer of at least 3" >&2
  exit 2
fi
if [[ ! "$max_regression_pct" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "OGM_MODBUS_PERF_MAX_REGRESSION_PCT must be a non-negative number" >&2
  exit 2
fi

baseline_idle_raw=()
baseline_request_raw=()
candidate_idle_raw=()
candidate_request_raw=()
baseline_idle_legacy=()
baseline_request_legacy=()
candidate_idle_legacy=()
candidate_request_legacy=()
baseline_format=
candidate_format=
baseline_idle_ops=
baseline_request_ops=
candidate_idle_ops=
candidate_request_ops=

record_measurement() {
  local label=$1
  local program=$2
  local measurement format idle_total idle_ops request_total request_ops
  local idle_per_op request_per_op
  measurement=$(measure "$label" "$program")
  read -r format idle_total idle_ops request_total request_ops \
    idle_per_op request_per_op <<<"$measurement"

  if [[ "$label" == baseline ]]; then
    if [[ -z "$baseline_format" ]]; then
      baseline_format=$format
    elif [[ "$format" != "$baseline_format" ]]; then
      echo 'baseline output format changed between samples' >&2
      exit 2
    fi
    if [[ "$format" == raw ]]; then
      if [[ -z "$baseline_idle_ops" ]]; then
        baseline_idle_ops=$idle_ops
        baseline_request_ops=$request_ops
      elif [[ "$idle_ops" != "$baseline_idle_ops" ||
              "$request_ops" != "$baseline_request_ops" ]]; then
        echo 'baseline raw operation counts changed between samples' >&2
        exit 2
      fi
    fi
    baseline_idle_raw+=("$idle_total")
    baseline_request_raw+=("$request_total")
    baseline_idle_legacy+=("$idle_per_op")
    baseline_request_legacy+=("$request_per_op")
  else
    if [[ -z "$candidate_format" ]]; then
      candidate_format=$format
    elif [[ "$format" != "$candidate_format" ]]; then
      echo 'candidate output format changed between samples' >&2
      exit 2
    fi
    if [[ "$format" == raw ]]; then
      if [[ -z "$candidate_idle_ops" ]]; then
        candidate_idle_ops=$idle_ops
        candidate_request_ops=$request_ops
      elif [[ "$idle_ops" != "$candidate_idle_ops" ||
              "$request_ops" != "$candidate_request_ops" ]]; then
        echo 'candidate raw operation counts changed between samples' >&2
        exit 2
      fi
    fi
    candidate_idle_raw+=("$idle_total")
    candidate_request_raw+=("$request_total")
    candidate_idle_legacy+=("$idle_per_op")
    candidate_request_legacy+=("$request_per_op")
  fi
}

for ((sample = 0; sample < sample_count; ++sample)); do
  echo "performance pair $((sample + 1))/$sample_count" >&2
  if (( sample % 2 == 0 )); then
    record_measurement baseline "$baseline_program"
    record_measurement candidate "$candidate_program"
  else
    record_measurement candidate "$candidate_program"
    record_measurement baseline "$baseline_program"
  fi
done

mode=$(comparison_mode "$baseline_format" "$candidate_format")
if [[ "$mode" == raw ]]; then
  if ! raw_counts_match \
      "$baseline_idle_ops" "$baseline_request_ops" \
      "$candidate_idle_ops" "$candidate_request_ops"; then
    echo 'raw operation counts differ between baseline and candidate executables' >&2
    echo "baseline idle/request ops: $baseline_idle_ops/$baseline_request_ops" >&2
    echo "candidate idle/request ops: $candidate_idle_ops/$candidate_request_ops" >&2
    exit 2
  fi
  baseline_idle_median=$(median "${baseline_idle_raw[@]}")
  baseline_request_median=$(median "${baseline_request_raw[@]}")
  candidate_idle_median=$(median "${candidate_idle_raw[@]}")
  candidate_request_median=$(median "${candidate_request_raw[@]}")
else
  echo 'WARNING: raw modbus_perf totals are not available from both executables.' >&2
  echo 'Falling back to integer-truncated legacy ns/op summaries; sub-ns changes are quantized.' >&2
  echo 'Operation counts are intentionally not compared because legacy output does not expose them.' >&2
  baseline_idle_median=$(median "${baseline_idle_legacy[@]}")
  baseline_request_median=$(median "${baseline_request_legacy[@]}")
  candidate_idle_median=$(median "${candidate_idle_legacy[@]}")
  candidate_request_median=$(median "${candidate_request_legacy[@]}")
  if (( baseline_idle_median == 0 || baseline_request_median == 0 )); then
    echo 'legacy baseline truncated to 0ns/op and cannot produce a meaningful ratio' >&2
    exit 2
  fi
fi

within_limit() {
  local baseline=$1
  local candidate=$2
  awk -v baseline="$baseline" -v candidate="$candidate" \
      -v limit="$max_regression_pct" \
      'BEGIN { exit !(baseline > 0 && candidate * 100.0 <= baseline * (100.0 + limit)) }'
}

echo "same-host interleaved median comparison ($sample_count pairs, ${max_regression_pct}% limit, mode=$mode)"
if [[ "$mode" == raw ]]; then
  echo "raw operation counts: idle=$baseline_idle_ops request=$baseline_request_ops"
  report_raw_ratio idle "$baseline_idle_median" "$candidate_idle_median"
  report_raw_ratio request "$baseline_request_median" "$candidate_request_median"
else
  report_legacy_ratio idle "$baseline_idle_median" "$candidate_idle_median"
  report_legacy_ratio request "$baseline_request_median" "$candidate_request_median"
fi

failed=0
if ! within_limit "$baseline_idle_median" "$candidate_idle_median"; then
  echo "idle poll regression exceeds ${max_regression_pct}%" >&2
  failed=1
fi
if ! within_limit "$baseline_request_median" "$candidate_request_median"; then
  echo "request/TX regression exceeds ${max_regression_pct}%" >&2
  failed=1
fi
exit "$failed"
