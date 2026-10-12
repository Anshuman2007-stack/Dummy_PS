cmake_minimum_required(VERSION 3.8)
project(my_bot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME})

# Executable scripts -> `ros2 run my_bot_sim true_pose_gps.py`
install(PROGRAMS scripts/true_pose_gps.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
