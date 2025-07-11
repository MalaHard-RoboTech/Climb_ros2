cmake_minimum_required(VERSION 3.8)
project(cl_arganello_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Installa i file Python
install(
  PROGRAMS
  cl_arganello_interface/dongle_node.py
  cl_arganello_interface/arganello_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installazione della cartella launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
