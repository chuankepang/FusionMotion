cmake_minimum_required(VERSION 3.16)
project(rokae_admittance)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cust_msgs REQUIRED)
find_package(sixd_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(cartesian_admittance
    src/cartesian_admittance.cpp
    src/force_node.cpp
)

target_link_libraries(cartesian_admittance
  Rokae::Rokae
  Eigen3::Eigen
)

target_include_directories(cartesian_admittance PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    include
)

ament_target_dependencies(cartesian_admittance
  rclcpp
  std_msgs
  cust_msgs
  Eigen3
  sixd_interfaces
)

add_executable(joint_admittance
    src/joint_admittance.cpp
    src/force_node.cpp
)

target_link_libraries(joint_admittance
  Rokae::Rokae
  Eigen3::Eigen
)

target_include_directories(joint_admittance PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    include
)

ament_target_dependencies(joint_admittance
  rclcpp
  std_msgs
  cust_msgs
  Eigen3
  sixd_interfaces
)

install(TARGETS
  cartesian_admittance
  joint_admittance
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
