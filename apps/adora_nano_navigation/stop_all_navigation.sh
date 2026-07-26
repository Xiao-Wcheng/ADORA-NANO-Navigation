#!/usr/bin/env bash
set -euo pipefail

DORA="${DORA:-$HOME/dora-main/target/release/dora}"

"${DORA}" list | awk 'NR > 1 && $3 == "Running" {print $1}' | while read -r uuid; do
  if [[ -n "${uuid}" ]]; then
    echo "Stopping ${uuid}"
    "${DORA}" stop "${uuid}" || true
  fi
done

echo "Running Dora flows after stop request:"
"${DORA}" list | awk 'NR == 1 || $3 == "Running"'
