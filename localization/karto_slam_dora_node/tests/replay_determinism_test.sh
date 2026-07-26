#!/usr/bin/env bash
set -euo pipefail
replay="$1"
dataset="$2"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
"$replay" "$dataset" "$work/run1/map"
"$replay" "$dataset" "$work/run2/map"
for suffix in posegraph.dora pgm yaml metadata.json; do
  cmp "$work/run1/map.$suffix" "$work/run2/map.$suffix"
done
echo "replay_determinism_test PASS sha256=$(sha256sum "$work/run1/map.posegraph.dora" | cut -d' ' -f1)"
