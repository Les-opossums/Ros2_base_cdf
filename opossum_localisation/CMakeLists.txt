cmake_minimum_required(VERSION 3.8)
project(opossum_localisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(opossum_msgs REQUIRED)
find_package(obstacle_detector REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

#include cpp include directory
include_directories(include)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(beacon_detector src/beacon_detector.cpp src/math_lidar.cpp src/BeaconSorter.cpp src/PositionFinder.cpp src/publisher.cpp)
ament_target_dependencies(beacon_detector rclcpp std_msgs opossum_msgs vision_msgs geometry_msgs obstacle_detector)

target_include_directories(beacon_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Install cpp executables
install(TARGETS
  beacon_detector
  DESTINATION lib/${PROJECT_NAME}
)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/beacon_detector.py
  scripts/tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
