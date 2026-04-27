#!/usr/bin/env bash
# slam.sh — Fresh-start SLAM mapping for hikbot
# Usage: ./slam.sh [--no-build]
set -e

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
LOCAL_SETUP="$WORKSPACE/install/setup.bash"

# ── Colors ──
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
RST='\033[0m'

info()  { echo -e "${CYN}[INFO]${RST}  $*"; }
ok()    { echo -e "${GRN}[  OK]${RST}  $*"; }
warn()  { echo -e "${YLW}[WARN]${RST}  $*"; }

# ── 1. Kill stale processes ──
info "Killing stale Gazebo / ROS processes..."
pkill -9 -f "gz sim"                 2>/dev/null || true
pkill -9 -f "parameter_bridge"      2>/dev/null || true
pkill -9 -f "slam_toolbox"          2>/dev/null || true
pkill -9 -f "static_transform"      2>/dev/null || true
pkill -9 -f "rviz2"                 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
sleep 2
ok "All old processes killed."

# ── 2. Deactivate any venv (NumPy 2.x / headless cv2 breaks builds) ──
deactivate 2>/dev/null || true

# ── 3. Source ROS ──
source "$ROS_SETUP"
info "ROS 2 Jazzy sourced."

# ── 4. Build (skip with --no-build) ──
if [[ "$1" != "--no-build" ]]; then
    info "Building warehouse_sim..."
    cd "$WORKSPACE"
    colcon build --packages-select warehouse_sim --symlink-install
    ok "Build complete."
else
    warn "Skipping build (--no-build)."
fi

# ── 5. Source local workspace ──
source "$LOCAL_SETUP"
ok "Local workspace sourced."

# ── 6. Launch SLAM (Gazebo + bridges + slam_toolbox + RViz) ──
info "Launching SLAM..."
ros2 launch warehouse_sim slam.launch.py &
SLAM_PID=$!

# ── 7. Wait for Gazebo to be ready, then open teleop in a new terminal ──
info "Waiting for Gazebo to initialize (15s)..."
sleep 15

info "Opening teleop in a new terminal..."
TELEOP_CMD="source $ROS_SETUP && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/hikbot/cmd_vel"

if grep -qi microsoft /proc/version 2>/dev/null; then
    # WSL — use Windows Terminal + helper script (avoids quoting hell)
    wt.exe wsl.exe bash "$WORKSPACE/teleop.sh" &
elif command -v gnome-terminal &>/dev/null; then
    gnome-terminal -- bash -c "$TELEOP_CMD ; exec bash" &
elif command -v xterm &>/dev/null; then
    xterm -hold -e "bash -c '$TELEOP_CMD'" &
else
    warn "No terminal emulator found. Open a new terminal and run:"
    echo "  $TELEOP_CMD"
fi

ok "Everything is up!"
echo ""
echo -e "${CYN}╔══════════════════════════════════════════════════════════╗${RST}"
echo -e "${CYN}║${RST}  ${GRN}SLAM is running.${RST} Drive the robot with teleop.          ${CYN}║${RST}"
echo -e "${CYN}║${RST}  Map grows in RViz as you explore the warehouse.        ${CYN}║${RST}"
echo -e "${CYN}║${RST}                                                          ${CYN}║${RST}"
echo -e "${CYN}║${RST}  ${YLW}Save map:${RST}                                               ${CYN}║${RST}"
echo -e "${CYN}║${RST}    ros2 run nav2_map_server map_saver_cli \\              ${CYN}║${RST}"
echo -e "${CYN}║${RST}        -f ~/hikbot_warehouse_map                         ${CYN}║${RST}"
echo -e "${CYN}║${RST}                                                          ${CYN}║${RST}"
echo -e "${CYN}║${RST}  ${RED}Ctrl+C here to stop everything.${RST}                        ${CYN}║${RST}"
echo -e "${CYN}╚══════════════════════════════════════════════════════════╝${RST}"
echo ""

# ── 8. Wait for SLAM process; cleanup on Ctrl+C ──
cleanup() {
    echo ""
    warn "Shutting down..."
    pkill -9 -f "gz sim"                 2>/dev/null || true
    pkill -9 -f "parameter_bridge"      2>/dev/null || true
    pkill -9 -f "slam_toolbox"          2>/dev/null || true
    pkill -9 -f "static_transform"      2>/dev/null || true
    pkill -9 -f "rviz2"                 2>/dev/null || true
    pkill -9 -f "teleop_twist_keyboard" 2>/dev/null || true
    ok "All processes stopped. Bye!"
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $SLAM_PID
