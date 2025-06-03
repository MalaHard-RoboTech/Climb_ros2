cmake_minimum_required(VERSION 3.8)
project(cl_action_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cl_interface REQUIRED)


add_library(simple_action_server SHARED src/simple_action_server.cpp)
ament_target_dependencies(simple_action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  cl_interface
)

add_library(simple_action_client SHARED src/simple_action_client.cpp)
ament_target_dependencies(simple_action_client
  rclcpp
  rclcpp_action
  rclcpp_components
  cl_interface
)



# Registrazione del nodo SimpleActionServer
rclcpp_components_register_node(simple_action_server
  PLUGIN "cl_action_service::SimpleActionServer"
  EXECUTABLE simple_action_server_node
)

# Registrazione del nodo SimpleActionClient (se necessario)
rclcpp_components_register_node(simple_action_client
  PLUGIN "cl_action_service::SimpleActionClient" # Assicurati che il namespace e la classe siano corretti
  EXECUTABLE simple_action_client_node
)

install(TARGETS
  # simple_action_client
  simple_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
