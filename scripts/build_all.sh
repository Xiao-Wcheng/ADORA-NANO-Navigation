#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: $0"
  echo "Builds every production node and retained unit test."
  exit 0
fi

if [[ -f "$HOME/.cargo/env" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/.cargo/env"
fi

build_cmake() {
  local relative="$1"
  echo "== Building $relative =="
  cmake -S "$ROOT/$relative" -B "$ROOT/$relative/build"
  cmake --build "$ROOT/$relative/build" -j"$(nproc)"
}

build_cmake "driver/ms200_dora_node"
build_cmake "chassis/feetech_kiwi_chassis_dora_node"
echo "== Building chassis/feetech_kiwi_chassis_dora_node/sdk_node =="
cargo build --locked --release --manifest-path \
  "$ROOT/chassis/feetech_kiwi_chassis_dora_node/sdk_node/Cargo.toml"
build_cmake "localization/initial_pose_dora_node"
build_cmake "localization/karto_slam_dora_node"
build_cmake "planning/pose_goal_dora_node"
build_cmake "planning/adora_nano_global_planner_node"
build_cmake "planning/rpp_local_controller_dora_node"
build_cmake "planning/velocity_smoother_dora_node"
build_cmake "planning/nav_supervisor_dora_node"

echo "Standalone navigation build complete."
