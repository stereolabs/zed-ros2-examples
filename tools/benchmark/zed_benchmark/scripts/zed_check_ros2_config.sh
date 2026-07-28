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
# topics and prints a report for each test. It is meant to verify that the
# user's ROS 2 / DDS / system configuration is able to deliver the camera data
# at the expected rate and bandwidth, and to quantify what composing a
# subscriber in the camera container actually buys.
#
# Each topic is measured twice, always with a *typed* subscription so the two
# runs use the very same size and latency accounting:
#   * interprocess : the benchmark runs as a separate process, so messages are
#                    serialized and routed through the middleware.
#   * ipc          : the benchmark is loaded as a component in the ZED
#                    container with intra-process comms enabled.
#
# The ZED node keeps enable_ipc:=true in both runs, so the publisher is
# identical and the only variable is where the subscriber lives.
#
# What to compare: LATENCY and CPU, not bandwidth. On the intra-process path no
# bytes are transported at all, so its "bandwidth" is a notional payload figure
# and the frequency is simply whatever the publisher produces. Latency and CPU
# are what actually change.
#
# Note: a typed subscription is required because a runtime-typed
# (rclcpp::GenericSubscription) subscriber is never registered with the
# IntraProcessManager and therefore can never take the intra-process path.
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
MODES=("interprocess" "ipc")
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
  local report zed_log

  report="${OUTPUT_DIR}/report_${mode}_${name}.txt"
  zed_log="${OUTPUT_DIR}/zed_${mode}_${name}.log"

  echo
  echo "==================================================================="
  info "TEST: topic='${topic}' | mode='${mode}' | depth_mode='${depth_mode}'"
  echo "==================================================================="

  # ----> Start the ZED node (composable node inside its container)
  # enable_ipc stays true in BOTH modes so the publisher is byte-for-byte the
  # same experiment and the only variable is where the subscriber lives. With
  # IPC enabled the ZED node publishes images through its TypeAdapter, which
  # feeds the intra-process path AND the middleware, so an out-of-process
  # subscriber still receives a normal sensor_msgs/Image.
  setsid ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:="${CAMERA_MODEL}" \
    enable_ipc:=true \
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
  # Both modes use subscription_mode:=typed so the size and latency accounting
  # is identical and the two reports really are comparable.
  if [[ "${mode}" == "interprocess" ]]; then
    # Separate process: blocks until the benchmark self-terminates.
    ros2 run "${BENCH_PKG}" "${BENCH_EXE}" --ros-args \
      -p topic_name:="${topic}" \
      -p subscription_mode:=typed \
      -p test_duration_sec:="${DURATION}.0" \
      -p avg_win_size:="${WIN_SIZE}" \
      -p use_ros_log:=true \
      -p log_file_path:="${report}"
  else
    # Composed in the ZED container with intra-process comms enabled. On
    # completion the component shuts the container down, after writing the
    # report.
    ros2 component load "${CONTAINER}" "${BENCH_COMP_PKG}" "${BENCH_PLUGIN}" \
      -e use_intra_process_comms:=true \
      -p topic_name:="${topic}" \
      -p subscription_mode:=typed \
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
  printf "%-7s %-13s %-7s %-10s %-12s %-9s\n" \
    "TOPIC" "MODE" "MSGS" "FREQ[Hz]" "LATENCY[ms]" "CPU[%]"
  echo "-------------------------------------------------------------------"
  local s name topic mode report msgs freq lat cpu
  for s in "${SCENARIOS[@]}"; do
    IFS='|' read -r name topic _ <<< "${s}"
    for mode in "${MODES[@]}"; do
      report="${OUTPUT_DIR}/report_${mode}_${name}.txt"
      if [[ -f "${report}" ]]; then
        msgs=$(grep -E "Messages received:" "${report}" | grep -oE "[0-9]+" | head -1)
        freq=$(grep -E "Frequency \[Hz\]" "${report}" | sed -E 's/.*mean:[[:space:]]*([0-9.]+).*/\1/')
        lat=$(grep -E "Latency \[ms\]" "${report}" | sed -E 's/.*mean:[[:space:]]*([0-9.]+).*/\1/')
        [[ "${lat}" == *"not available"* ]] && lat="n/a"
        cpu=$(grep -E "Process CPU:" "${report}" | sed -E 's/.*\(([0-9.]+)%.*/\1/')
      else
        msgs="-"; freq="NO DATA"; lat="-"; cpu="-"
      fi
      printf "%-7s %-13s %-7s %-10s %-12s %-9s\n" \
        "${name}" "${mode}" "${msgs:-?}" "${freq:-?}" "${lat:-?}" "${cpu:-?}"
    done
  done
  echo "-------------------------------------------------------------------"
  echo "Compare LATENCY and CPU between the two modes: those are what the"
  echo "intra-process path changes. Frequency is set by the publisher, and the"
  echo "bandwidth of an intra-process run is notional (nothing is transported),"
  echo "so neither of them shows the IPC gain."
  echo "Each report states its own delivery path, and says explicitly when"
  echo "intra-process delivery was confirmed rather than merely possible."
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

for s in "${SCENARIOS[@]}"; do
  IFS='|' read -r name topic depth_mode <<< "${s}"
  for mode in "${MODES[@]}"; do
    run_test "${mode}" "${name}" "${topic}" "${depth_mode}"
  done
done

print_summary
# <---- Main
