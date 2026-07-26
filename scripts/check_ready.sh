#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: $0 [check_navigation_ready.py options]"
  echo "Example: $0 --mode localize"
  exit 0
fi

exec python3 "$ROOT/apps/adora_nano_navigation/check_navigation_ready.py" "$@"

