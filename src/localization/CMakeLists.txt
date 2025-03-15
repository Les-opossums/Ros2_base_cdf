cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cdf_msgs REQUIRED)
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

add_library(math_lidar src/math_lidar.cpp)
target_include_directories(math_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(publisher src/publisher.cpp)
target_include_directories(publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(publisher geometry_msgs rclcpp std_msgs cdf_msgs vision_msgs)

add_library(BeaconSorter src/BeaconSorter.cpp)
target_include_directories(BeaconSorter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(BeaconSorter geometry_msgs rclcpp std_msgs cdf_msgs vision_msgs)

#Create cpp executable
# add_executable(beacon_detector src/beacon_detector.cpp)
# ament_target_dependencies(beacon_detector rclcpp std_msgs)



# Install cpp executables
install(TARGETS
  # beacon_detector
  DESTINATION lib/${PROJECT_NAME}
)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/beacon_detector.py
  scripts/on_board_robot_detector.py
  scripts/lidar_simulation.py
  scripts/nav_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # # Unit tests
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(unit_vector test/unit_vector.cpp)
  # # target_include_directories(${PROJECT_NAME} PUBLIC
  # #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # #   $<INSTALL_INTERFACE:include>
  # # )
  
  # if(TARGET unit_vector)
  #   target_include_directories(unit_vector PUBLIC
  #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #     $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  #     $<INSTALL_INTERFACE:include>
  #   )
    
  #   # If the test depends on other packages:
  #   # ament_target_dependencies(unit_vector rclcpp std_msgs ...)
  # endif()
  # if(TARGET test_unit_vector AND TARGET math_lidar)
  #   target_link_libraries(test_unit_vector PRIVATE math_lidar)
  # endif()
endif()

ament_package()