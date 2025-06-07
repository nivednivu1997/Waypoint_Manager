cmake_minimum_required(VERSION 3.8)
project(cpp_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
include_directories(/usr/include/websocketpp)



add_executable(waypoint_node src/waypoint.cpp)
target_link_libraries(waypoint_node  yaml-cpp)
ament_target_dependencies(waypoint_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  message_filters
  tf2_geometry_msgs
  nlohmann_json
  yaml-cpp
  rcl_interfaces
  nav2_msgs
  visualization_msgs
  std_srvs
  tf2_geometry_msgs
)

install(TARGETS waypoint_node
  DESTINATION lib/${PROJECT_NAME}
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

