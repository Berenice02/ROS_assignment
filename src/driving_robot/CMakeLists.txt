cmake_minimum_required(VERSION 3.8)
project(driving_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

add_executable(drive_robot src/drive_robot.cpp)
ament_target_dependencies(drive_robot rclcpp geometry_msgs sensor_msgs custom_msgs)
target_include_directories(drive_robot PRIVATE include)

add_executable(obstacle_monitor src/obstacle_monitor.cpp)
ament_target_dependencies(obstacle_monitor rclcpp sensor_msgs custom_msgs)
target_include_directories(obstacle_monitor PRIVATE include)

add_executable(velocity_monitor src/velocity_monitor.cpp)
ament_target_dependencies(velocity_monitor rclcpp geometry_msgs custom_msgs)

install(TARGETS drive_robot obstacle_monitor velocity_monitor DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
