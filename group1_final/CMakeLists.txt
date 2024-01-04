cmake_minimum_required(VERSION 3.8)
project(group1_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


include_directories(
  include
)

add_executable(static_broadcaster_node src/tb3_static_broadcaster.cpp)
ament_target_dependencies(static_broadcaster_node rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs)

# add_executable(navigation_node src/navigation_demo.cpp)
# ament_target_dependencies(navigation_node rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs nav2_msgs rclcpp_action)

# add_executable(waypoint_navigation_node src/tb3_waypoint_navigator.cpp)
# ament_target_dependencies(waypoint_navigation_node rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs nav2_msgs rclcpp_action)


add_executable(aruco_detector_node src/tb3_aruco_detector.cpp)
ament_target_dependencies(aruco_detector_node rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs ros2_aruco_interfaces nav2_msgs rclcpp_action)


install(TARGETS
  static_broadcaster_node
  aruco_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
