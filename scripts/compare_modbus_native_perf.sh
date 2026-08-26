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
  local local_line forwarded_line

  local_line=$(printf '%s\n' "$output" | sed -n '/^modbus_perf: /p' | tail -n 1)
  forwarded_line=$(printf '%s\n' "$output" | sed -n '/^modbus_forwarded_perf: /p' | tail -n 1)

  if [[ ! "$local_line" =~ idle_median_ns=([0-9]+)[[:space:]]+idle_ops=([0-9]+)[[:space:]]+request_median_ns=([0-9]+)[[:space:]]+request_ops=([0-9]+) ]]; then
    echo "$label emitted a missing or malformed modbus_perf line" >&2
    return 2
  fi
  local idle_total=${BASH_REMATCH[1]}
  local idle_ops=${BASH_REMATCH[2]}
  local request_total=${BASH_REMATCH[3]}
  local request_ops=${BASH_REMATCH[4]}

  if [[ ! "$forwarded_line" =~ forwarded_median_ns=([0-9]+)[[:space:]]+forwarded_ops=([0-9]+)[[:space:]]+max_median_ns=([0-9]+)[[:space:]]+max_ops=([0-9]+)[[:space:]]+multichunk_median_ns=([0-9]+)[[:space:]]+multichunk_ops=([0-9]+) ]]; then
    echo "$label emitted a missing or malformed modbus_forwarded_perf line" >&2
    return 2
  fi
  local forwarded_total=${BASH_REMATCH[1]}
  local forwarded_ops=${BASH_REMATCH[2]}
  local maximum_total=${BASH_REMATCH[3]}
  local maximum_ops=${BASH_REMATCH[4]}
  local multichunk_total=${BASH_REMATCH[5]}
  local multichunk_ops=${BASH_REMATCH[6]}

  local operation_count
  for operation_count in "$idle_ops" "$request_ops" "$forwarded_ops" \
      "$maximum_ops" "$multichunk_ops"; do
    if (( operation_count == 0 )); then
      echo "$label emitted a zero operation count" >&2
      return 2
    fi
  done

  printf '%s %s %s %s %s %s %s %s %s %s\n' \
    "$idle_total" "$idle_ops" "$request_total" "$request_ops" \
    "$forwarded_total" "$forwarded_ops" "$maximum_total" "$maximum_ops" \
    "$multichunk_total" "$multichunk_ops"
}

measure() {
  local label=$1
  local program=$2
  local output status parsed
  set +e
  output=$(env -u MBUS_RTU_SLAVE_STRICT_PERFORMANCE "$program" 2>&1)
  status=$?
  set -e
  if (( status != 0 )); then
    echo "$label test program failed with status $status" >&2
    echo "$output" >&2
    return "$status"
  fi
  if ! parsed=$(parse_measurement "$label" "$output"); then
    echo "$output" >&2
    return 2
  fi
  printf '%s\n' "$parsed"
}

assert_equal() {
  if [[ "$2" != "$1" ]]; then
    echo "self-test failed ($3): expected '$1', got '$2'" >&2
    exit 1
  fi
}

run_self_test() {
  local sample parsed
  sample=$'noise\nmodbus_perf: idle_median_ns=165 idle_ops=10 request_median_ns=6655 request_ops=20\nmodbus_forwarded_perf: forwarded_median_ns=7000 forwarded_ops=30 max_median_ns=8000 max_ops=40 multichunk_median_ns=9000 multichunk_ops=50'
  parsed=$(parse_measurement self-test "$sample")
  assert_equal '165 10 6655 20 7000 30 8000 40 9000 50' "$parsed" parse
  if parse_measurement malformed 'no performance output' >/dev/null 2>&1; then
    echo 'self-test failed: malformed output was accepted' >&2
    exit 1
  fi
  echo 'compare_modbus_native_perf self-test: PASS'
}

if [[ ${1:-} == --self-test ]]; then
  [[ $# -eq 1 ]] || { usage; exit 2; }
  run_self_test
  exit 0
fi

[[ $# -ge 2 && $# -le 3 ]] || { usage; exit 2; }
baseline_program=$1
candidate_program=$2
sample_count=${3:-7}
max_regression_pct=${MBUS_RTU_SLAVE_MAX_REGRESSION_PCT:-5}

[[ -x "$baseline_program" ]] || { echo "baseline program is not executable: $baseline_program" >&2; exit 2; }
[[ -x "$candidate_program" ]] || { echo "candidate program is not executable: $candidate_program" >&2; exit 2; }
if [[ ! "$sample_count" =~ ^[0-9]+$ ]] || (( sample_count < 3 || sample_count % 2 == 0 )); then
  echo "ODD_SAMPLE_COUNT must be an odd integer of at least 3" >&2
  exit 2
fi
if [[ ! "$max_regression_pct" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "MBUS_RTU_SLAVE_MAX_REGRESSION_PCT must be a non-negative number" >&2
  exit 2
fi

baseline_rows=()
candidate_rows=()
for ((sample = 0; sample < sample_count; ++sample)); do
  echo "performance pair $((sample + 1))/$sample_count" >&2
  if (( sample % 2 == 0 )); then
    baseline_rows+=("$(measure baseline "$baseline_program")")
    candidate_rows+=("$(measure candidate "$candidate_program")")
  else
    candidate_rows+=("$(measure candidate "$candidate_program")")
    baseline_rows+=("$(measure baseline "$baseline_program")")
  fi
done

value_at() {
  local row=$1
  local column=$2
  local values
  read -r -a values <<<"$row"
  printf '%s\n' "${values[$column]}"
}

median_column() {
  local column=$1
  shift
  local position=$((($# + 1) / 2))
  local row
  for row in "$@"; do value_at "$row" "$column"; done | \
    sort -n | sed -n "${position}p"
}

within_limit() {
  awk -v baseline="$1" -v candidate="$2" -v limit="$max_regression_pct" \
    'BEGIN { exit !(baseline > 0 && candidate * 100.0 <= baseline * (100.0 + limit)) }'
}

lane_names=(idle request forwarded forwarded_max forwarded_multichunk)
failed=0
echo "same-host interleaved median comparison ($sample_count pairs, ${max_regression_pct}% limit)"
for lane_index in "${!lane_names[@]}"; do
  total_column=$((lane_index * 2))
  ops_column=$((total_column + 1))
  baseline_ops=$(value_at "${baseline_rows[0]}" "$ops_column")
  candidate_ops=$(value_at "${candidate_rows[0]}" "$ops_column")
  if [[ "$baseline_ops" != "$candidate_ops" ]]; then
    echo "${lane_names[$lane_index]} operation counts differ: $baseline_ops vs $candidate_ops" >&2
    exit 2
  fi
  for row in "${baseline_rows[@]}" "${candidate_rows[@]}"; do
    if [[ $(value_at "$row" "$ops_column") != "$baseline_ops" ]]; then
      echo "${lane_names[$lane_index]} operation count changed between samples" >&2
      exit 2
    fi
  done
  baseline_median=$(median_column "$total_column" "${baseline_rows[@]}")
  candidate_median=$(median_column "$total_column" "${candidate_rows[@]}")
  awk -v name="${lane_names[$lane_index]}" -v ops="$baseline_ops" \
      -v baseline="$baseline_median" -v candidate="$candidate_median" \
    'BEGIN {
       delta = ((candidate / baseline) - 1.0) * 100.0;
       printf "%s: ops=%d baseline_total=%dns candidate_total=%dns delta=%+.2f%%\n",
              name, ops, baseline, candidate, delta;
     }'
  if ! within_limit "$baseline_median" "$candidate_median"; then
    echo "${lane_names[$lane_index]} regression exceeds ${max_regression_pct}%" >&2
    failed=1
  fi
done
exit "$failed"
