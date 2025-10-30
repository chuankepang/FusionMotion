cmake_minimum_required(VERSION 3.16)
project(rokae_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Using static/shared libraries option
option(XCORE_LINK_SHARED_LIBS "Example executables link shared library" OFF)
# Compile option: whether using xMateModel library, supports Linux x86_64 and Windows 64bit
option(XCORE_USE_XMATE_MODEL "The library of kinematics and dynamics calculation for cobot" OFF)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cust_msgs REQUIRED)
find_package(sixd_interfaces REQUIRED)

add_subdirectory(lib)

# === 包含目录 ===
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${rokae_INCLUDE_DIRS}
)

# === 编译库（每个模块独立）===
## 1. 力缓存库
add_library(force_buffer_lib SHARED src/force_node.cpp)
ament_target_dependencies(force_buffer_lib rclcpp cust_msgs std_msgs sixd_interfaces)
target_link_libraries(force_buffer_lib Eigen3::Eigen)
target_include_directories(force_buffer_lib PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

## 2. 阻抗控制库
add_library(admittance_ctrl_lib SHARED src/admittance_node.cpp)
ament_target_dependencies(admittance_ctrl_lib rclcpp std_msgs)
target_link_libraries(admittance_ctrl_lib Rokae::Rokae)
target_include_directories(admittance_ctrl_lib PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

## 3. Rokae 运动控制库
add_library(rokae_node_lib SHARED src/rokae_node.cpp)
ament_target_dependencies(rokae_node_lib rclcpp cust_msgs std_msgs sixd_interfaces)
target_link_libraries(rokae_node_lib Rokae::Rokae Eigen3::Eigen)
target_include_directories(rokae_node_lib PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

## 4. 手爪控制库
# add_library(inspire_node_lib SHARED src/inspire_node.cpp)
# ament_target_dependencies(inspire_node_lib rclcpp std_msgs)

# === 编译可执行文件 ===
add_executable(main_node src/main_node.cpp)
ament_target_dependencies(main_node rclcpp std_msgs)
target_link_libraries(main_node
  force_buffer_lib
  admittance_ctrl_lib
  rokae_node_lib
  # inspire_node_lib
)

# === 安装 ===
install(TARGETS
  # 库
  force_buffer_lib
  admittance_ctrl_lib
  rokae_node_lib
  # inspire_node_lib
  # 可执行
  main_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装 launch 和 config（如果有）
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()