#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
cd "$REPO_DIR"

GOAL_X="-2.1"
GOAL_Y="1.0"

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

python3 "$REPO_DIR/decisions.py" "$@" &
PIDS+=($!)

for _ in $(seq 1 30); do
    if ros2 topic info /goal_pose 2>/dev/null | grep -q "Subscription count: [1-9]"; then
        break
    fi
    sleep 0.5
done

echo "Publishing fixed goal pose: x=$GOAL_X, y=$GOAL_Y"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {z: 0.976, w: 0.219}}}"

wait "${PIDS[@]}"
