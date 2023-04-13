cmake_minimum_required(VERSION 3.8)
project(ismr23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories( include )
add_library( ismr23 SHARED src/ismr23.cpp )
ament_target_dependencies( ismr23 rclcpp moveit_core geometry_msgs moveit_ros_planning moveit_ros_move_group control_msgs tf2 tf2_ros )

add_executable( ismr23_node src/node.cpp )
target_link_libraries( ismr23_node ismr23 )
ament_target_dependencies( ismr23_node rclcpp moveit_core geometry_msgs )

install( DIRECTORY launch DESTINATION share/${PROJECT_NAME} )
install( TARGETS ismr23 LIBRARY DESTINATION lib )
install( TARGETS ismr23_node DESTINATION lib/${PROJECT_NAME} )


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

ament_package()
