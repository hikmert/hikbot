#!/usr/bin/env bash
# Saves the current slam_toolbox map to maps/warehouse_map
# Run while the simulation is active.

set -e
MAPS_DIR="$(cd "$(dirname "$0")/.." && pwd)/maps"
MAP_FILE="$MAPS_DIR/warehouse_map"

echo "[save_map] Saving slam_toolbox pose graph to: $MAP_FILE"
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_FILE'}"

echo "[save_map] Saving occupancy grid PNG to: $MAP_FILE"
ros2 run nav2_map_server map_saver_cli -f "$MAP_FILE" --ros-args -p use_sim_time:=true

echo "[save_map] Done. Files: $MAP_FILE.posegraph / .data / .pgm / .yaml"
