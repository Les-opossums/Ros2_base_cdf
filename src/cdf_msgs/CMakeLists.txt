cmake_minimum_required(VERSION 3.8)
project(cdf_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AxResult.msg"
  "msg/Buttons.msg"
  "msg/DebugLidar.msg"
  "msg/Evitement.msg"
  "msg/LidarLoc.msg"
  "msg/MergedData.msg"
  "msg/PicAction.msg"
  "msg/RobotData.msg"
  "msg/ServoDebug.msg"
  "msg/TCPmessage.msg"
  "msg/Trajectoire.msg"
  "msg/Obstacles.msg"
  "msg/CircleObstacle.msg"
  "msg/SegmentObstacle.msg"
  "msg/GoalDetection.msg"
  "msg/CamCommand.msg"
  "action/ActionSequencer.action"
  "srv/Init.srv"
  DEPENDENCIES std_msgs geometry_msgs
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
