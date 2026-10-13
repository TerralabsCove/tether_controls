#!/usr/bin/env bash
set -euo pipefail
for role in coordinator node1 node2; do
  "$(dirname "$0")/build-role.sh" "$role"
done
