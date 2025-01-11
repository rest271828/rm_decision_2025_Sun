cmake_minimum_required(VERSION 3.5)
project(information_widgets)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_decision_interfaces REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(chessboard_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories( 
  include 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

add_library(information_widgets
  src/chessboard_handle.cpp
  src/prism_handle.cpp
)

ament_target_dependencies(information_widgets
  rclcpp
  std_msgs
  geometry_msgs
  rm_decision_interfaces
  auto_aim_interfaces
  chessboard_interfaces
  nav2_msgs
  rclcpp_action
)

install(DIRECTORY include/information_widgets/
  DESTINATION include/information_widgets
)

install(TARGETS information_widgets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
