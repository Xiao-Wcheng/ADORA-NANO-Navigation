#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "$HOME/.cargo/env" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/.cargo/env"
fi

packages=(
  build-essential cmake pkg-config python3 python3-yaml
  libboost-graph-dev libeigen3-dev libceres-dev
  libsuitesparse-dev libyaml-cpp-dev libudev-dev
)

if [[ "${1:-}" == "--check" ]]; then
  missing=0
  for package in "${packages[@]}"; do
    if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q "install ok installed"; then
      echo "MISS $package"
      missing=1
    else
      echo "OK   $package"
    fi
  done
  for command_name in rustc cargo; do
    if command -v "$command_name" >/dev/null 2>&1; then
      echo "OK   $command_name: $(command -v "$command_name")"
    else
      echo "MISS $command_name"
      missing=1
    fi
  done
  for candidate in "$HOME/dora-main/target/release/dora" "$HOME/dora/target/release/dora"; do
    if [[ -x "$candidate" ]]; then
      echo "OK   Dora CLI: $candidate"
      exit "$missing"
    fi
  done
  echo "MISS Dora CLI (install Dora separately)"
  exit 1
fi

if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: $0 [--check|--help]"
  echo "Installs Ubuntu build dependencies. Dora must be installed separately."
  exit 0
fi

echo "Installing dependencies for $ROOT"
sudo apt-get update
sudo apt-get install -y "${packages[@]}" rustc cargo
