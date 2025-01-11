cmake_minimum_required(VERSION 3.5)
project(information_widgets)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include/information_widgets
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
