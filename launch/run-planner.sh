#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
cd "$REPO_DIR"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.fastdds.xml}"
if [[ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]]; then
    printf 'Fast DDS profile not found: %s\n' "$FASTRTPS_DEFAULT_PROFILES_FILE" >&2
    exit 1
fi

export QT_QPA_PLATFORM=xcb
QT5CT_PLUGIN_DIR="$HOME/.local/lib/qt5/plugins"
if [[ -f "$QT5CT_PLUGIN_DIR/platformthemes/libqt5ct.so" ]]; then
    export QT_PLUGIN_PATH="$QT5CT_PLUGIN_DIR${QT_PLUGIN_PATH:+:$QT_PLUGIN_PATH}"
    export QT_QPA_PLATFORMTHEME=qt5ct
elif [[ -f /usr/lib/x86_64-linux-gnu/qt5/plugins/platformthemes/libqt5ct.so ]]; then
    export QT_QPA_PLATFORMTHEME=qt5ct
fi

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

rviz2 -d "$REPO_DIR/pathPlanner.rviz" &
PIDS+=($!)

python3 "$REPO_DIR/decisions.py" &
PIDS+=($!)

wait "${PIDS[@]}"
