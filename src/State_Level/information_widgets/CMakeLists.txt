cmake_minimum_required(VERSION 3.8)
project(information_widgets)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(SRC_LIST
  src/chessboard_def.cpp
  src/prism_def.cpp
)

set(HEAD_LIST
  include/information_widgets/chessboard_def.hpp
  include/information_widgets/prism_def.hpp
  include/information_widgets/rm_decision_defs.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SRC_LIST}
  ${HEAD_LIST}
)

ament_auto_package()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(rm_decision_interfaces REQUIRED)
# find_package(iw_interfaces REQUIRED)

# add_library(information_widgets SHARED
#   include/information_widgets/chessboard_def.hpp
#   include/information_widgets/prism_def.hpp
#   include/information_widgets/rm_decision_defs.hpp
#   src/chessboard_def.cpp
#   src/prism_def.cpp
#   )

# target_include_directories(information_widgets PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   )

# ament_target_dependencies(information_widgets PUBLIC 
#   rclcpp
#   std_msgs
#   geometry_msgs
#   rm_decision_interfaces
#   iw_interfaces
#   )

# ament_export_targets(information_widgets HAS_LIBRARY_TARGET)
# ament_export_dependencies(
#   rclcpp
#   std_msgs
#   geometry_msgs
#   rm_decision_interfaces
#   iw_interfaces
# )

# install(
#         DIRECTORY include/
#         DESTINATION include
# )

# install(
#         TARGETS information_widgets
#         EXPORT information_widgets
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
#         INCLUDES DESTINATION include
# )
# ament_package()