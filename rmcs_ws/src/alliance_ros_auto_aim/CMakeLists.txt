cmake_minimum_required(VERSION 3.22)
project(alliance_ros_auto_aim)

# ============================================================================
# 编译选项
# ============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# 查找依赖
# ============================================================================
find_package(ament_cmake REQUIRED)

# ROS2 核心依赖
find_package(rclcpp REQUIRED)
find_package(rmcs_executor REQUIRED)
find_package(pluginlib REQUIRED)

# ROS2 消息依赖
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 图像处理依赖
find_package(OpenCV REQUIRED)

# RMCS 框架依赖
find_package(rmcs_description REQUIRED)
find_package(fast_tf REQUIRED)

# 硬件依赖
find_package(hikcamera REQUIRED)

# 数学库
find_package(Eigen3 REQUIRED)

# ============================================================================
# alliance_auto_aim 核心库（子模块）
# ============================================================================
add_subdirectory(alliance_auto_aim)

# ============================================================================
# 主库：RMCS Component 插件
# ============================================================================
set(COMPONENT_SOURCES
  src/alliance_adapter.cpp
  src/alliance_initializer.cpp
  src/alliance_camera.cpp
  src/alliance_controller.cpp
)

# 添加共享库
add_library(${PROJECT_NAME} SHARED ${COMPONENT_SOURCES})

# 设置头文件路径
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/alliance_auto_aim/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/alliance_auto_aim/src>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖库
target_link_libraries(${PROJECT_NAME}
  alliance_auto_aim
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

# 声明 ament 依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rmcs_executor
  pluginlib
  sensor_msgs
  std_msgs
  rmcs_description
  fast_tf
  hikcamera
)

# ============================================================================
# 插件导出
# ============================================================================
pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

# ============================================================================
# 安装规则
# ============================================================================
# 安装库文件
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# ============================================================================
# ament 包配置
# ============================================================================
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rmcs_executor
  pluginlib
  sensor_msgs
  std_msgs
  rmcs_description
  fast_tf
  hikcamera
  Eigen3
)

ament_package()
