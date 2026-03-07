cmake_minimum_required(VERSION 3.8)
project(nav2_custom_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(pluginlib REQUIRED)

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

include_directories(include)

add_library(nav2_custom_bt_plugins SHARED
  src/battery_monitor.cpp
  src/crowd_stop.cpp
  src/obstacle_slowdown.cpp
  src/return_to_dock.cpp
)

ament_target_dependencies(nav2_custom_bt_plugins
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_behavior_tree plugin.xml)

install(
  TARGETS nav2_custom_bt_plugins
  LIBRARY DESTINATION lib
)

ament_package()
