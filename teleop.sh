#!/usr/bin/env bash
# teleop.sh — Drive hikbot with keyboard
source /opt/ros/jazzy/setup.bash
echo "=== HIKBOT TELEOP ==="
echo "Use i/j/k/l/u/o/m/,/. to drive. Ctrl+C to quit."
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/hikbot/cmd_vel
