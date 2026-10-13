#!/usr/bin/env bash
set -euo pipefail

role="${1:-}"
case "$role" in
  coordinator)
    if [[ -z "${TETHER_MASTER_MOTOR_SLAVE_ID:-}" ||
          -z "${TETHER_MASTER_MOTOR_FEEDBACK_ID:-}" ]]; then
      echo "set TETHER_MASTER_MOTOR_SLAVE_ID and TETHER_MASTER_MOTOR_FEEDBACK_ID first" >&2
      exit 2
    fi
    build=build-coordinator
    args=(-D TETHER_ROLE=0 -D TETHER_NODE_ID=0
          -D TETHER_MASTER_MOTOR_CONFIGURED=1
          -D TETHER_MOTOR_SLAVE_ID="$TETHER_MASTER_MOTOR_SLAVE_ID"
          -D TETHER_MOTOR_MASTER_ID="$TETHER_MASTER_MOTOR_FEEDBACK_ID")
    ;;
  node1|node2|node3)
    node="${role#node}"
    build="build-${role}"
    if [[ "$node" == "1" ]]; then
      args=(-D TETHER_ROLE=1 -D TETHER_NODE_ID=1
            -D TETHER_MOTOR_SLAVE_ID=0x07 -D TETHER_MOTOR_MASTER_ID=0x17)
    elif [[ "$node" == "2" ]]; then
      args=(-D TETHER_ROLE=1 -D TETHER_NODE_ID=2
            -D TETHER_MOTOR_SLAVE_ID=0x04 -D TETHER_MOTOR_MASTER_ID=0x14)
    else
      echo "only node1 and node2 exist in the three-module design" >&2
      exit 2
    fi
    ;;
  *)
    echo "usage: $0 coordinator|node1|node2" >&2
    exit 2
    ;;
esac

idf.py -B "$build" "${args[@]}" set-target esp32c6
idf.py -B "$build" build
