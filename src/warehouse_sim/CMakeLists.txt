cmake_minimum_required(VERSION 3.8)
project(warehouse_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds rviz models config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/autonomous_mapper.py
    scripts/odom_tf_broadcaster.py
    scripts/map_manager_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
