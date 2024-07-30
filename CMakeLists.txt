cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server 
	rclcpp 
	rclcpp_action 
	geometry_msgs 
	nav_msgs 
	tf2
)
rosidl_target_interfaces(tortoisebot_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_add_gtest(test_waypoints_server test/test_waypoints_server.cpp TIMEOUT 120)
target_link_libraries(test_waypoints_server ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(test_waypoints_server
    rclcpp
    rclcpp_action
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
  )
rosidl_target_interfaces(test_waypoints_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS tortoisebot_action_server DESTINATION lib/${PROJECT_NAME})


ament_package()
