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
find_package(builtin_interfaces REQUIRED)

# ─── Custom Interfaces ───────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetControlMode.srv"
  "msg/ArganelloRawTelemetry.msg"
  "msg/ArganelloEnhancedTelemetry.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# ─── Install Python Files ────────────────────────────────
install(
  PROGRAMS
  cl_arganello_interface/dongle_node.py
  cl_arganello_interface/arganello_node.py
  cl_arganello_interface/arganello_enhancer_node.py
  cl_arganello_interface/friction_estimator.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─── Install Launch Files ────────────────────────────────
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ─── Required for Interface Export ───────────────────────
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  builtin_interfaces
)

ament_package()
