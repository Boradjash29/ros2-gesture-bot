cmake_minimum_required(VERSION 3.8)
project(four_wheel_gesture_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Find dependencies ───────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ─── Install Python package (the four_wheel_gesture_control/ module) ─────────
ament_python_install_package(${PROJECT_NAME})

# ─── Install Python executable scripts ───────────────────────────────────────
install(PROGRAMS
  scripts/hand_gesture_node.py
  scripts/differential_drive_node.py
  scripts/robot_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─── Install launch files ─────────────────────────────────────────────────────
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# ─── Install config files ─────────────────────────────────────────────────────
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)



# ─── Linting (optional) ───────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
