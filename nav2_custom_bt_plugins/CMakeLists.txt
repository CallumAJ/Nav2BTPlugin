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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_plugins test/test_plugins.cpp)
  target_link_libraries(test_plugins nav2_custom_bt_plugins)
  ament_target_dependencies(test_plugins
    rclcpp
    behaviortree_cpp_v3
    sensor_msgs
    geometry_msgs
    nav2_msgs
  )
endif()

include_directories(include)

add_library(nav2_custom_bt_plugins SHARED
  src/battery_monitor.cpp
  src/crowd_stop.cpp
  src/obstacle_slowdown.cpp
  src/return_to_dock.cpp
  src/register_nodes.cpp
)

target_compile_definitions(nav2_custom_bt_plugins PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(nav2_custom_bt_plugins
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp_v3
  pluginlib
  sensor_msgs
  geometry_msgs
  nav2_msgs
)

pluginlib_export_plugin_description_file(nav2_behavior_tree plugin.xml)

install(
  TARGETS nav2_custom_bt_plugins
  LIBRARY DESTINATION lib
)

install(
  FILES custom_nav_tree.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
