cmake_minimum_required(VERSION 3.8)
project(opossum_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(opossum_msgs REQUIRED)

# Find the C++ serial library
find_package(serial REQUIRED)

include_directories(include)

# 2. Build the C++ Executable
# Make sure your file is located at src/comm_node.cpp
add_executable(comm_node src/comm_node.cpp)

# 3. Link the dependencies to your new C++ node
ament_target_dependencies(comm_node
  rclcpp
  std_msgs
  opossum_msgs
  serial
)

# 4. Install the C++ executable
install(TARGETS
  comm_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Keep the Python stuff in case you still use python nodes in this package
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/comm.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()