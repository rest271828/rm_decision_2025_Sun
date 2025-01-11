cmake_minimum_required(VERSION 3.8)
project(terminal_listener)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_decision_interfaces REQUIRED)

add_executable(${PROJECT_NAME} src/terminal_listener.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  rm_decision_interfaces
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
