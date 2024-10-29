cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(AMENT_LINT_AUTO_FOUND FALSE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros
)
rosidl_target_interfaces(tortoisebot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  tortoisebot_action_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_waypoints test/test_waypoints.cpp)
  target_link_libraries(test_waypoints ${rclcpp_LIBRARIES})
  ament_target_dependencies(test_waypoints
    rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros
  )
  rosidl_target_interfaces(test_waypoints
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
endif()

ament_package()
