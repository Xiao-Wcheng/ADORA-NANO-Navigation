#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ -f "$HOME/.cargo/env" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/.cargo/env"
fi

build_cmake_node() {
  local dir="$1"
  local target="${2:-}"
  echo
  echo "== Building ${dir} =="
  cmake -S "${ROOT}/${dir}" -B "${ROOT}/${dir}/build"
  if [[ -n "${target}" ]]; then
    cmake --build "${ROOT}/${dir}/build" --target "${target}" -j"$(nproc)"
  else
    cmake --build "${ROOT}/${dir}/build" -j"$(nproc)"
  fi
}

build_rust_node() {
  local dir="$1"
  echo
  echo "== Building ${dir} =="
  cargo build --manifest-path "${ROOT}/${dir}/Cargo.toml" --release
}

build_cmake_node "driver/ms200_dora_node" "ms200_dora_node"
build_cmake_node "localization/initial_pose_dora_node" "initial_pose_dora_node"
build_cmake_node "localization/karto_slam_dora_node" "karto_slam_dora_node"
build_cmake_node "planning/pose_goal_dora_node" "pose_goal_dora_node"
build_cmake_node "planning/adora_nano_global_planner_node" "adora_nano_global_planner_node"
build_cmake_node "planning/rpp_local_controller_dora_node" "rpp_local_controller_dora_node"
build_cmake_node "planning/velocity_smoother_dora_node" "velocity_smoother_dora_node"
build_cmake_node "planning/nav_supervisor_dora_node" "nav_supervisor_dora_node"
build_cmake_node "chassis/feetech_kiwi_chassis_dora_node" "keyboard_control_dora_node"
build_rust_node "chassis/feetech_kiwi_chassis_dora_node/sdk_node"

echo
echo "Navigation stack build complete."
