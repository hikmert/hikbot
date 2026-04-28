#!/usr/bin/env bash
# slam.sh - Fresh-start autonomous SLAM mapping for hikbot
# Usage: ./slam.sh [--no-build] [--teleop]
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

NO_BUILD=0
TELEOP=0
for arg in "$@"; do
    case "$arg" in
        --no-build) NO_BUILD=1 ;;
        --teleop) TELEOP=1 ;;
        *)
            warn "Unknown option: $arg"
            ;;
    esac
done

info "Killing stale Gazebo / ROS processes..."
pkill -9 -f "gz sim"                 2>/dev/null || true
pkill -9 -f "parameter_bridge"      2>/dev/null || true
pkill -9 -f "slam_toolbox"          2>/dev/null || true
pkill -9 -f "autonomous_mapper.py"  2>/dev/null || true
pkill -9 -f "static_transform"      2>/dev/null || true
pkill -9 -f "rviz2"                 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "teleop_twist_keyboard" 2>/dev/null || true
sleep 2
ok "All old processes killed."

deactivate 2>/dev/null || true

source "$ROS_SETUP"
info "ROS 2 Jazzy sourced."

chmod +x "$WORKSPACE/src/warehouse_sim/scripts/autonomous_mapper.py" 2>/dev/null || true

if [[ "$NO_BUILD" != "1" ]]; then
    info "Building warehouse_sim..."
    cd "$WORKSPACE"
    colcon build --packages-select warehouse_sim --symlink-install
    ok "Build complete."
else
    warn "Skipping build (--no-build)."
fi

source "$LOCAL_SETUP"
ok "Local workspace sourced."

info "Launching autonomous SLAM..."
ros2 launch warehouse_sim slam.launch.py autostart_mapper:=true &
SLAM_PID=$!

if [[ "$TELEOP" == "1" ]]; then
    info "Waiting for Gazebo to initialize (15s)..."
    sleep 15

    info "Opening teleop in a new terminal..."
    TELEOP_CMD="source $ROS_SETUP && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/hikbot/cmd_vel"

    if grep -qi microsoft /proc/version 2>/dev/null; then
        wt.exe wsl.exe bash "$WORKSPACE/teleop.sh" &
    elif command -v gnome-terminal &>/dev/null; then
        gnome-terminal -- bash -c "$TELEOP_CMD ; exec bash" &
    elif command -v xterm &>/dev/null; then
        xterm -hold -e "bash -c '$TELEOP_CMD'" &
    else
        warn "No terminal emulator found. Open a new terminal and run:"
        echo "  $TELEOP_CMD"
    fi
fi

ok "Everything is up!"
echo ""
echo "============================================================"
echo "  Autonomous SLAM is running."
echo "  The robot starts with a 360-degree scan spin, then explores"
echo "  frontiers with inflated-grid A* and LiDAR safety. Watch"
echo "  /map and /hikbot/exploration_path live in RViz."
echo ""
echo "  Save map:"
echo "    ros2 run nav2_map_server map_saver_cli -f ~/hikbot_warehouse_map"
echo ""
echo "  Ctrl+C here to stop everything."
echo "============================================================"
echo ""

cleanup() {
    echo ""
    warn "Shutting down..."
    pkill -9 -f "gz sim"                 2>/dev/null || true
    pkill -9 -f "parameter_bridge"      2>/dev/null || true
    pkill -9 -f "slam_toolbox"          2>/dev/null || true
    pkill -9 -f "autonomous_mapper.py"  2>/dev/null || true
    pkill -9 -f "static_transform"      2>/dev/null || true
    pkill -9 -f "rviz2"                 2>/dev/null || true
    pkill -9 -f "teleop_twist_keyboard" 2>/dev/null || true
    ok "All processes stopped. Bye!"
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $SLAM_PID
