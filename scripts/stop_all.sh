#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: $0"
  echo "Stops keyboard control and all Adora Nano navigation flows."
  exit 0
fi

if [[ -p /tmp/feetech_kiwi_keyboard_fifo ]]; then
  timeout 1s bash -c "printf 'stop\n' > /tmp/feetech_kiwi_keyboard_fifo" || true
fi
exec bash "$ROOT/apps/adora_nano_navigation/stop_all_navigation.sh"
