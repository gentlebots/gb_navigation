cmake_minimum_required(VERSION 3.5)
project(gb_navigation)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_executor
    ament_index_cpp
    plansys2_bt_actions
    behaviortree_cpp_v3
)

include_directories(include)

add_library(gb_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs gb_move_bt_node)

add_library(gb_approach_object_bt_node SHARED src/behavior_tree_nodes/ApproachObject.cpp)
list(APPEND plugin_libs gb_approach_object_bt_node)

add_library(gb_follow_bt_node SHARED src/behavior_tree_nodes/Follow.cpp)
list(APPEND plugin_libs gb_follow_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(dummy_controller_node src/dummy_controller_node.cpp)
ament_target_dependencies(dummy_controller_node ${dependencies})

install(DIRECTORY rviz maps params launch config behavior_trees_xml DESTINATION share/${PROJECT_NAME})

install(TARGETS
  dummy_controller_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()