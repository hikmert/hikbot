#!/usr/bin/env bash
# explore.sh — Autonomous frontier exploration for hikbot
# Usage: ./explore.sh [--no-build]
set -e

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
LOCAL_SETUP="$WORKSPACE/install/setup.bash"

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
RST='\033[0m'

info()  { echo -e "${CYN}[INFO]${RST}  $*"; }
ok()    { echo -e "${GRN}[  OK]${RST}  $*"; }
warn()  { echo -e "${YLW}[WARN]${RST}  $*"; }

# ── 1. Kill stale processes ──────────────────────────────────────────
info "Killing stale processes..."
pkill -9 -f "gz sim"                 2>/dev/null || true
pkill -9 -f "parameter_bridge"      2>/dev/null || true
pkill -9 -f "slam_toolbox"          2>/dev/null || true
pkill -9 -f "static_transform"      2>/dev/null || true
pkill -9 -f "rviz2"                 2>/dev/null || true
pkill -9 -f "bt_navigator"          2>/dev/null || true
pkill -9 -f "controller_server"     2>/dev/null || true
pkill -9 -f "planner_server"        2>/dev/null || true
pkill -9 -f "behavior_server"       2>/dev/null || true
pkill -9 -f "frontier_explorer"     2>/dev/null || true
sleep 2
ok "Old processes killed."

# ── 2. Deactivate venv (NumPy 2.x / headless cv2 breaks builds) ─────
deactivate 2>/dev/null || true

# ── 3. Source ROS ────────────────────────────────────────────────────
source "$ROS_SETUP"
info "ROS 2 Jazzy sourced."

# ── 4. Build ─────────────────────────────────────────────────────────
if [[ "$1" != "--no-build" ]]; then
    info "Building warehouse_sim..."
    cd "$WORKSPACE"
    colcon build --packages-select warehouse_sim --symlink-install
    ok "Build complete."
else
    warn "Skipping build (--no-build)."
fi

# ── 5. Source local workspace ─────────────────────────────────────────
source "$LOCAL_SETUP"
ok "Local workspace sourced."

# ── 6. Launch ─────────────────────────────────────────────────────────
info "Launching autonomous exploration..."
info "  → Gazebo + SLAM + Nav2 + Frontier Explorer"
info "  → Robot will autonomously map the entire warehouse."
info "  → Save map when done:"
info "      ros2 run nav2_map_server map_saver_cli -f ~/hikbot_warehouse_map"
echo ""
ros2 launch warehouse_sim explore.launch.py &
EXPLORE_PID=$!

# ── 7. Cleanup on Ctrl+C ─────────────────────────────────────────────
cleanup() {
    echo ""
    warn "Shutting down..."
    pkill -9 -f "gz sim"             2>/dev/null || true
    pkill -9 -f "parameter_bridge"  2>/dev/null || true
    pkill -9 -f "slam_toolbox"      2>/dev/null || true
    pkill -9 -f "static_transform"  2>/dev/null || true
    pkill -9 -f "rviz2"             2>/dev/null || true
    pkill -9 -f "bt_navigator"      2>/dev/null || true
    pkill -9 -f "controller_server" 2>/dev/null || true
    pkill -9 -f "planner_server"    2>/dev/null || true
    pkill -9 -f "behavior_server"   2>/dev/null || true
    pkill -9 -f "frontier_explorer" 2>/dev/null || true
    ok "All processes stopped. Bye!"
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $EXPLORE_PID
