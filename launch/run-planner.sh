#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
cd "$REPO_DIR"

source /opt/ros/humble/setup.bash

PIDS=()
cleanup() {
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 "$REPO_DIR/mapPublisher.py" &
PIDS+=($!)

rviz2 -d "$REPO_DIR/pathPlanner.rviz" >/dev/null 2>&1 &
PIDS+=($!)

python3 "$REPO_DIR/decisions.py" &
PIDS+=($!)

wait "${PIDS[@]}"
