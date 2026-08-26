#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 BASELINE_WORKTREE CANDIDATE_WORKTREE [ODD_SAMPLE_COUNT]" >&2
  echo "       $0 --self-test" >&2
  echo "runs three native forwarded-write lanes alternately; default limit is 5%" >&2
}

median() {
  local position=$((($# + 1) / 2))
  printf '%s\n' "$@" | sort -n | sed -n "${position}p"
}

ratio_ppm() {
  awk -v baseline="$1" -v candidate="$2" \
    'BEGIN { printf "%.0f", (candidate / baseline) * 1000000.0 }'
}

paired_within_limit() {
  awk -v ratio="$1" -v limit="$2" \
    'BEGIN { delta = ((ratio / 1000000.0) - 1.0) * 100.0;
             exit !(delta <= limit); }'
}

run_self_test() {
  # Three large candidate outliers make the independent raw medians diverge,
  # while four of seven paired ratios remain +4%. The paired statistic must
  # pass this host-drift case.
  local drift_baseline=(100 100 100 100 1000 1000 1000)
  local drift_candidate=(104 104 104 150 1040 1500 1500)
  local drift_ratios=()
  local index
  for index in 0 1 2 3 4 5 6; do
    drift_ratios+=("$(ratio_ppm "${drift_baseline[index]}" \
                                 "${drift_candidate[index]}")")
  done
  local drift_median
  drift_median=$(median "${drift_ratios[@]}")
  if [[ "$drift_median" != 1040000 ]] ||
     ! paired_within_limit "$drift_median" 5; then
    echo "self-test failed: paired drift cancellation was rejected" >&2
    exit 1
  fi

  local regression_ratios=(1060000 1060000 1060000 1060000 \
                           1060000 1060000 1060000)
  local regression_median
  regression_median=$(median "${regression_ratios[@]}")
  if paired_within_limit "$regression_median" 5; then
    echo "self-test failed: consistent 6% regression was accepted" >&2
    exit 1
  fi
  echo "compare_modbus_forwarded_pio_perf self-test: PASS"
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

baseline_arg=$1
candidate_arg=$2
sample_count=${3:-7}
max_regression_pct=${OGM_MODBUS_PERF_MAX_REGRESSION_PCT:-5}

if [[ ! -d "$baseline_arg" || ! -d "$candidate_arg" ]]; then
  echo "baseline and candidate must both be worktree directories" >&2
  exit 2
fi
baseline_dir=$(cd "$baseline_arg" && pwd)
candidate_dir=$(cd "$candidate_arg" && pwd)
pio_core_dir=${PLATFORMIO_CORE_DIR:-$candidate_dir/../.platformio_core_portable}
default_perf_build_flags='-std=gnu++17 -Os -Iinclude -Isrc -Itest/support '
default_perf_build_flags+='-DOGM_BRIDGE_MODE -DOGM_USE_MUTEX '
default_perf_build_flags+='-DOGM_MODBUS_MT_ACCESSORS -DUSB_DEBUG '
default_perf_build_flags+='-DBRIDGE_UPSTREAM_TX_DIAG=1'
perf_build_flags=${OGM_MODBUS_PERF_BUILD_FLAGS:-$default_perf_build_flags}
perf_runner=()
if command -v taskset >/dev/null 2>&1; then
  perf_cpu=${OGM_MODBUS_PERF_CPU:-}
  if [[ -z "$perf_cpu" ]]; then
    allowed_cpus=$(taskset -pc $$ | sed 's/^.*: //')
    perf_cpu=${allowed_cpus%%,*}
    perf_cpu=${perf_cpu%%-*}
  fi
  if [[ ! "$perf_cpu" =~ ^[0-9]+$ ]] ||
     ! taskset -c "$perf_cpu" true >/dev/null 2>&1; then
    echo "OGM_MODBUS_PERF_CPU is not available to this process" >&2
    exit 2
  fi
  perf_runner=(taskset -c "$perf_cpu")
  echo "forwarded performance affinity: CPU $perf_cpu"
elif [[ -n ${OGM_MODBUS_PERF_CPU:-} ]]; then
  echo "OGM_MODBUS_PERF_CPU requires taskset" >&2
  exit 2
fi
if [[ ! "$sample_count" =~ ^[0-9]+$ ]] ||
   (( sample_count < 3 || sample_count % 2 == 0 )); then
  echo "ODD_SAMPLE_COUNT must be an odd integer of at least 3" >&2
  exit 2
fi
if [[ ! "$max_regression_pct" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "OGM_MODBUS_PERF_MAX_REGRESSION_PCT must be non-negative" >&2
  exit 2
fi

build_lane() {
  local label=$1
  local worktree=$2
  local output
  if ! output=$(cd "$worktree" &&
      env -u OGM_STRICT_MODBUS_PERF \
        OGM_MODBUS_PERF_ONLY=1 \
        PLATFORMIO_BUILD_FLAGS="$perf_build_flags" \
        PLATFORMIO_CORE_DIR="$pio_core_dir" \
        pio test -e native_modbus_tests -v 2>&1); then
    echo "$label native Modbus suite failed" >&2
    echo "$output" >&2
    return 1
  fi
  if [[ ! -x "$worktree/.pio/build/native_modbus_tests/program" ]]; then
    echo "$label produced no native Modbus test executable" >&2
    return 2
  fi
}

measure() {
  local label=$1
  local worktree=$2
  local output line
  if ! output=$(cd "$worktree" &&
      env -u OGM_STRICT_MODBUS_PERF \
        OGM_MODBUS_PERF_ONLY=1 \
        "${perf_runner[@]}" \
        .pio/build/native_modbus_tests/program 2>&1); then
    echo "$label native Modbus executable failed" >&2
    echo "$output" >&2
    return 1
  fi
  line=$(printf '%s\n' "$output" |
      sed -n '/^modbus_forwarded_perf: /p' | tail -n 1)
  if [[ ! "$line" =~ forwarded_median_ns=([0-9]+)[[:space:]]+forwarded_ops=([0-9]+)[[:space:]]+max_median_ns=([0-9]+)[[:space:]]+max_ops=([0-9]+)[[:space:]]+multichunk_median_ns=([0-9]+)[[:space:]]+multichunk_ops=([0-9]+) ]]; then
    echo "$label emitted no parseable modbus_forwarded_perf line" >&2
    echo "$output" >&2
    return 2
  fi
  printf '%s %s %s %s %s %s\n' \
    "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}" \
    "${BASH_REMATCH[3]}" "${BASH_REMATCH[4]}" \
    "${BASH_REMATCH[5]}" "${BASH_REMATCH[6]}"
}

# Compile once, then measure the executables directly. Re-running PlatformIO
# before every sub-microsecond sample lets dependency scanning and unequal tree
# sizes perturb CPU frequency, which can dwarf the migration delta itself.
build_lane baseline "$baseline_dir"
build_lane candidate "$candidate_dir"
measure baseline "$baseline_dir" >/dev/null
measure candidate "$candidate_dir" >/dev/null

baseline_samples=()
candidate_samples=()
baseline_max_samples=()
candidate_max_samples=()
baseline_multichunk_samples=()
candidate_multichunk_samples=()
single_pair_ratios=()
max_pair_ratios=()
multichunk_pair_ratios=()
expected_single_ops=
expected_max_ops=
expected_multichunk_ops=
for ((sample = 0; sample < sample_count; ++sample)); do
  if (( sample % 2 == 0 )); then
    order=(baseline candidate)
  else
    order=(candidate baseline)
  fi
  for label in "${order[@]}"; do
    if [[ "$label" == baseline ]]; then
      measurement=$(measure baseline "$baseline_dir")
      read -r total_ns operations max_ns max_operations \
        multichunk_ns multichunk_operations <<<"$measurement"
      baseline_samples+=("$total_ns")
      baseline_max_samples+=("$max_ns")
      baseline_multichunk_samples+=("$multichunk_ns")
      pair_baseline_single=$total_ns
      pair_baseline_max=$max_ns
      pair_baseline_multichunk=$multichunk_ns
    else
      measurement=$(measure candidate "$candidate_dir")
      read -r total_ns operations max_ns max_operations \
        multichunk_ns multichunk_operations <<<"$measurement"
      candidate_samples+=("$total_ns")
      candidate_max_samples+=("$max_ns")
      candidate_multichunk_samples+=("$multichunk_ns")
      pair_candidate_single=$total_ns
      pair_candidate_max=$max_ns
      pair_candidate_multichunk=$multichunk_ns
    fi
    if [[ -z "$expected_single_ops" ]]; then
      expected_single_ops=$operations
      expected_max_ops=$max_operations
      expected_multichunk_ops=$multichunk_operations
    elif [[ "$operations" != "$expected_single_ops" ||
            "$max_operations" != "$expected_max_ops" ||
            "$multichunk_operations" != "$expected_multichunk_ops" ]]; then
      echo "forwarded operation counts changed between samples" >&2
      exit 2
    fi
    printf 'forwarded sample %d %s: single=%dns/%s max=%dns/%s ' \
      "$((sample + 1))" "$label" "$total_ns" "$operations" \
      "$max_ns" "$max_operations"
    printf 'multichunk=%dns/%s ops\n' \
      "$multichunk_ns" "$multichunk_operations"
  done
  single_pair_ratio=$(ratio_ppm "$pair_baseline_single" \
    "$pair_candidate_single")
  max_pair_ratio=$(ratio_ppm "$pair_baseline_max" "$pair_candidate_max")
  multichunk_pair_ratio=$(ratio_ppm "$pair_baseline_multichunk" \
    "$pair_candidate_multichunk")
  single_pair_ratios+=("$single_pair_ratio")
  max_pair_ratios+=("$max_pair_ratio")
  multichunk_pair_ratios+=("$multichunk_pair_ratio")
  awk -v sample="$((sample + 1))" -v single="$single_pair_ratio" \
      -v max="$max_pair_ratio" -v multichunk="$multichunk_pair_ratio" \
    'BEGIN {
       printf "forwarded pair %d deltas: single=%+.2f%% max=%+.2f%% " \
              "multichunk=%+.2f%%\n", sample,
              ((single / 1000000.0) - 1.0) * 100.0,
              ((max / 1000000.0) - 1.0) * 100.0,
              ((multichunk / 1000000.0) - 1.0) * 100.0;
     }'
done

baseline_median=$(median "${baseline_samples[@]}")
candidate_median=$(median "${candidate_samples[@]}")
baseline_max_median=$(median "${baseline_max_samples[@]}")
candidate_max_median=$(median "${candidate_max_samples[@]}")
baseline_multichunk_median=$(median "${baseline_multichunk_samples[@]}")
candidate_multichunk_median=$(median "${candidate_multichunk_samples[@]}")
single_pair_ratio_median=$(median "${single_pair_ratios[@]}")
max_pair_ratio_median=$(median "${max_pair_ratios[@]}")
multichunk_pair_ratio_median=$(median "${multichunk_pair_ratios[@]}")

assert_lane() {
  local name=$1
  local baseline=$2
  local candidate=$3
  local operations=$4
  local paired_ratio=$5
  awk -v name="$name" -v baseline="$baseline" -v candidate="$candidate" \
      -v paired_ratio="$paired_ratio" -v limit="$max_regression_pct" \
      -v operations="$operations" \
  'BEGIN {
     raw_delta = ((candidate / baseline) - 1.0) * 100.0;
     paired_delta = ((paired_ratio / 1000000.0) - 1.0) * 100.0;
     printf "%s: baseline_median=%dns candidate_median=%dns " \
            "ops=%d raw_delta=%+.2f%% paired_delta=%+.2f%% limit=%.2f%%\n",
            name, baseline, candidate, operations, raw_delta, paired_delta,
            limit;
   }'
  paired_within_limit "$paired_ratio" "$max_regression_pct"
}

comparison_failed=0
assert_lane "forwarded FC6" "$baseline_median" "$candidate_median" \
  "$expected_single_ops" "$single_pair_ratio_median" || comparison_failed=1
assert_lane "forwarded FC16 max" "$baseline_max_median" \
  "$candidate_max_median" "$expected_max_ops" "$max_pair_ratio_median" || \
  comparison_failed=1
assert_lane "forwarded FC16 multichunk" "$baseline_multichunk_median" \
  "$candidate_multichunk_median" "$expected_multichunk_ops" \
  "$multichunk_pair_ratio_median" || \
  comparison_failed=1
exit "$comparison_failed"
