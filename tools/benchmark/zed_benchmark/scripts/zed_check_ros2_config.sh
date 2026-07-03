#!/usr/bin/env bash
#
# Copyright 2025 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# -----------------------------------------------------------------------------
# ZED ROS 2 configuration check
#
# Runs the topic benchmark against a real ZED node for a set of representative
# topics, in both "standard" (separate process) and "IPC" (composition) mode,
# and prints a report for each test. It is meant to verify that the user's
# ROS 2 / DDS / system configuration is able to deliver the camera data at the
# expected rate and bandwidth.
#
# Topics tested:
#   * /zed/zed_node/rgb/color/rect/image          (depth mode NONE)
#   * /zed/zed_node/depth/depth_registered        (depth mode NEURAL_LIGHT)
#   * /zed/zed_node/point_cloud/cloud_registered  (depth mode NEURAL_LIGHT)
#
# Usage:
#   ./zed_check_ros2_config.sh [camera_model] [duration_sec]
#
# Environment overrides:
#   CAMERA_MODEL   ZED camera model (default: zed2i)
#   DURATION       Seconds of measurement per test (default: 15)
#   WIN_SIZE       Benchmark averaging window size (default: 100)
#   TOPIC_TIMEOUT  Max seconds to wait for a topic to appear (default: 120)
#   SETTLE         Seconds to wait between tests (default: 5)
#   OUTPUT_DIR     Where to store the reports (default: ./zed_config_reports/<timestamp>)
# -----------------------------------------------------------------------------

set -u

# ----> Configuration
CAMERA_MODEL="${1:-${CAMERA_MODEL:-zed2i}}"
DURATION="${2:-${DURATION:-15}}"
WIN_SIZE="${WIN_SIZE:-100}"
TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-120}"
SETTLE="${SETTLE:-5}"
OUTPUT_DIR="${OUTPUT_DIR:-$(pwd)/zed_config_reports/$(date +%Y%m%d_%H%M%S)}"

CONTAINER="/zed/zed_container"   # default ZED node component container
BENCH_PKG="zed_topic_benchmark"
BENCH_EXE="zed_topic_benchmark"
BENCH_COMP_PKG="zed_topic_benchmark_component"
BENCH_PLUGIN="stereolabs::TopicBenchmarkComponent"

# Test matrix: "name|topic|depth_mode"
SCENARIOS=(
  "image|/zed/zed_node/rgb/color/rect/image|NONE"
  "depth|/zed/zed_node/depth/depth_registered|NEURAL_LIGHT"
  "cloud|/zed/zed_node/point_cloud/cloud_registered|NEURAL_LIGHT"
)
MODES=("standard" "ipc")
# <---- Configuration

ZED_PID=""   # PID of the currently running ZED launch (process group leader)

# ----> Helpers
err() { echo "[ERROR] $*" >&2; }
info() { echo "[INFO ] $*"; }

stop_zed() {
  # Stop the ZED launch (and every process in its group) if still running.
  if [[ -n "${ZED_PID}" ]] && kill -0 "${ZED_PID}" 2>/dev/null; then
    kill -INT -- "-${ZED_PID}" 2>/dev/null || kill -INT "${ZED_PID}" 2>/dev/null
    for _ in $(seq 1 20); do
      kill -0 "${ZED_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -9 -- "-${ZED_PID}" 2>/dev/null
  fi
  ZED_PID=""
}

cleanup() {
  stop_zed
  # Best-effort cleanup of any stray benchmark/container processes.
  pkill -f "${BENCH_EXE}" 2>/dev/null
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  # Wait until <topic> is advertised, up to TOPIC_TIMEOUT seconds.
  local topic="$1"
  local waited=0
  while ! ros2 topic list 2>/dev/null | grep -qx "${topic}"; do
    sleep 2
    waited=$((waited + 2))
    if (( waited >= TOPIC_TIMEOUT )); then
      return 1
    fi
  done
  return 0
}

run_test() {
  local mode="$1" name="$2" topic="$3" depth_mode="$4"
  local enable_ipc report zed_log

  [[ "${mode}" == "ipc" ]] && enable_ipc="true" || enable_ipc="false"
  report="${OUTPUT_DIR}/report_${mode}_${name}.txt"
  zed_log="${OUTPUT_DIR}/zed_${mode}_${name}.log"

  echo
  echo "==================================================================="
  info "TEST: topic='${topic}' | mode='${mode}' | depth_mode='${depth_mode}'"
  echo "==================================================================="

  # ----> Start the ZED node (composable node inside its container)
  setsid ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:="${CAMERA_MODEL}" \
    enable_ipc:="${enable_ipc}" \
    param_overrides:="depth.depth_mode:=${depth_mode}" \
    > "${zed_log}" 2>&1 &
  ZED_PID=$!
  info "ZED node launching (pid ${ZED_PID}), waiting for topic '${topic}'..."

  if ! wait_for_topic "${topic}"; then
    err "Topic '${topic}' not available after ${TOPIC_TIMEOUT}s. Skipping. (see ${zed_log})"
    stop_zed
    sleep "${SETTLE}"
    return 1
  fi
  info "Topic available. Letting the node stabilize..."
  sleep 3
  # <---- Start the ZED node

  # ----> Run the benchmark
  if [[ "${mode}" == "standard" ]]; then
    # Separate process: blocks until the benchmark self-terminates.
    ros2 run "${BENCH_PKG}" "${BENCH_EXE}" --ros-args \
      -p topic_name:="${topic}" \
      -p test_duration_sec:="${DURATION}.0" \
      -p avg_win_size:="${WIN_SIZE}" \
      -p use_ros_log:=true \
      -p log_file_path:="${report}"
  else
    # IPC: load the benchmark as a component into the ZED container, with
    # intra-process communication enabled. On completion the component shuts
    # the container down (and the report is written before that happens).
    ros2 component load "${CONTAINER}" "${BENCH_COMP_PKG}" "${BENCH_PLUGIN}" \
      -e use_intra_process_comms:=true \
      -p topic_name:="${topic}" \
      -p test_duration_sec:="${DURATION}.0" \
      -p avg_win_size:="${WIN_SIZE}" \
      -p use_ros_log:=true \
      -p log_file_path:="${report}" || \
      err "Failed to load the benchmark component into '${CONTAINER}'."
    # 'component load' returns immediately: wait for the measurement to finish.
    sleep "$((DURATION + 8))"
  fi
  # <---- Run the benchmark

  if [[ -f "${report}" ]]; then
    info "Report saved: ${report}"
  else
    err "No report produced for ${mode}/${name} (see ${zed_log})."
  fi

  stop_zed
  info "Waiting ${SETTLE}s for the camera to be released..."
  sleep "${SETTLE}"
}

print_summary() {
  echo
  echo "############################# SUMMARY #############################"
  printf "%-8s %-9s %-9s %-12s %-14s\n" "TOPIC" "MODE" "MSGS" "FREQ[Hz]" "BW[Mbps]"
  echo "-------------------------------------------------------------------"
  local s name topic mode report msgs freq bw
  for mode in "${MODES[@]}"; do
    for s in "${SCENARIOS[@]}"; do
      IFS='|' read -r name topic _ <<< "${s}"
      report="${OUTPUT_DIR}/report_${mode}_${name}.txt"
      if [[ -f "${report}" ]]; then
        msgs=$(grep -E "Messages received:" "${report}" | grep -oE "[0-9]+" | head -1)
        freq=$(grep -E "Frequency \[Hz\]" "${report}" | sed -E 's/.*mean:[[:space:]]*([0-9.]+).*/\1/')
        bw=$(grep -E "Bandwidth \[Mbps\]" "${report}" | sed -E 's/.*mean:[[:space:]]*([0-9.]+).*/\1/')
      else
        msgs="-"; freq="NO DATA"; bw="-"
      fi
      printf "%-8s %-9s %-9s %-12s %-14s\n" "${name}" "${mode}" "${msgs:-?}" "${freq:-?}" "${bw:-?}"
    done
  done
  echo "-------------------------------------------------------------------"
  echo "Full reports in: ${OUTPUT_DIR}"
  echo "###################################################################"
}
# <---- Helpers

# ----> Main
command -v ros2 >/dev/null 2>&1 || { err "'ros2' not found. Source your ROS 2 / workspace setup first."; exit 1; }

mkdir -p "${OUTPUT_DIR}"
info "ZED ROS 2 configuration check"
info "Camera model: ${CAMERA_MODEL} | Duration: ${DURATION}s | Window: ${WIN_SIZE}"
info "Reports directory: ${OUTPUT_DIR}"

for mode in "${MODES[@]}"; do
  for s in "${SCENARIOS[@]}"; do
    IFS='|' read -r name topic depth_mode <<< "${s}"
    run_test "${mode}" "${name}" "${topic}" "${depth_mode}"
  done
done

print_summary
# <---- Main
