cmake_minimum_required(VERSION 3.12)
project(rmcs_msgs LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB RMCS_MSGS_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "msg/*.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${RMCS_MSGS_MSGS}
  DEPENDENCIES geometry_msgs builtin_interfaces
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
