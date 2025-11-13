cmake_minimum_required(VERSION 3.12)
project(rmcs_tongji_auto_aim VERSION 1.0 LANGUAGES CXX)

# C++20 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# === ROS2 依赖 ===
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# === 外部库依赖 ===
find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# === alliance_auto_aim 路径 ===
set(ALLIANCE_AUTO_AIM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../alliance_auto_aim)

# 检查 alliance_auto_aim 是否存在
if(NOT EXISTS ${ALLIANCE_AUTO_AIM_DIR})
    message(FATAL_ERROR
        "alliance_auto_aim not found at ${ALLIANCE_AUTO_AIM_DIR}\n"
        "Please ensure it's available as a git submodule:\n"
        "  cd ${CMAKE_CURRENT_SOURCE_DIR}/..\n"
        "  git submodule update --init --recursive alliance_auto_aim")
endif()

message(STATUS "Found alliance_auto_aim at: ${ALLIANCE_AUTO_AIM_DIR}")

# === 添加 alliance_auto_aim 的 include 路径 ===
include_directories(
    ${ALLIANCE_AUTO_AIM_DIR}/include
    ${ALLIANCE_AUTO_AIM_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# === 收集 alliance_auto_aim 源文件 ===
# 只包含需要的模块:tongji, v1, util, core
file(GLOB_RECURSE ALLIANCE_TONGJI_SOURCES
    ${ALLIANCE_AUTO_AIM_DIR}/src/tongji/*.cpp
)

file(GLOB_RECURSE ALLIANCE_V1_SOURCES
    ${ALLIANCE_AUTO_AIM_DIR}/src/v1/*.cpp
)

file(GLOB_RECURSE ALLIANCE_UTIL_SOURCES
    ${ALLIANCE_AUTO_AIM_DIR}/src/util/*.cpp
)

file(GLOB_RECURSE ALLIANCE_CORE_SOURCES
    ${ALLIANCE_AUTO_AIM_DIR}/src/core/*.cpp
)

# 合并所有 alliance 源文件
set(ALLIANCE_SOURCES
    ${ALLIANCE_TONGJI_SOURCES}
    ${ALLIANCE_V1_SOURCES}
    ${ALLIANCE_UTIL_SOURCES}
    ${ALLIANCE_CORE_SOURCES}
)

# 排除测试文件和 playground
list(FILTER ALLIANCE_SOURCES EXCLUDE REGEX ".*/tests/.*")
list(FILTER ALLIANCE_SOURCES EXCLUDE REGEX ".*/playground/.*")
list(FILTER ALLIANCE_SOURCES EXCLUDE REGEX ".*test.*\\.cpp$")

message(STATUS "Alliance sources count: ${ALLIANCE_SOURCES}")

# === 本项目源文件 ===
file(GLOB_RECURSE PROJECT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

message(STATUS "Project sources: ${PROJECT_SOURCES}")

# === 构建共享库 ===
ament_auto_add_library(${PROJECT_NAME} SHARED
    ${PROJECT_SOURCES}
    ${ALLIANCE_SOURCES}
)

# === 链接库 ===
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp
    -lpthread
)

# === Pluginlib 导出 ===
pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

# === 安装配置文件 ===
install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# === 如果有模型文件,也需要安装 ===
# install(
#     DIRECTORY models/
#     DESTINATION share/${PROJECT_NAME}/models
# )

ament_auto_package()
