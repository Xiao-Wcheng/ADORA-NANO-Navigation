#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <binary>" >&2
  exit 2
fi

binary="$1"
if [[ ! -x "$binary" ]]; then
  echo "missing executable: $binary" >&2
  exit 1
fi

links="$(ldd "$binary")"
if grep -E '/opt/ros|lib(rcl|ros|tf2|ament|rcutils)' <<<"$links"; then
  echo "ROS runtime dependency detected" >&2
  exit 1
fi

echo "ROS_FREE=PASS"
