cmake_minimum_required(VERSION 3.8)
project(cl_arganello_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Dependencies ───────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ─── Service Interface ───────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetControlMode.srv"
  DEPENDENCIES builtin_interfaces
)

# ─── Install Python Files ────────────────────────────────
install(
  PROGRAMS
  cl_arganello_interface/dongle_node.py
  cl_arganello_interface/arganello_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─── Install Launch Files ────────────────────────────────
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ─── Required for Service Generation ─────────────────────
ament_export_dependencies(rosidl_default_runtime)
ament_package()

