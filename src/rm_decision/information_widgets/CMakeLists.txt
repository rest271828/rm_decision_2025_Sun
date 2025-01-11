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
  src/chessboard_handle.cpp
  src/chessboard_handle.cpp
)

set(HEAD_LIST
  include/information_widgets/chessboard_handle.hpp
  include/information_widgets/prism_handle.hpp
  include/information_widgets/rm_decision_defs.hpp
  include/information_widgets/rm_msg_includes.hpp
)

# 库名不能和可执行文件重名
# 要先生成库才能被别的功能包链接和使用头文件
ament_auto_add_library(los_controller_lib SHARED
  ${SRC_LIST}
  ${HEAD_LIST}
)

ament_auto_package()