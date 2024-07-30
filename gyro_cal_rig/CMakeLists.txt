cmake_minimum_required(VERSION 3.8)
project(gyro_cal_rig)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(riptide_gyro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gyro_cal_rig_msgs REQUIRED)

set(deps
  rclcpp
  riptide_gyro
  gyro_cal_rig_msgs)

add_executable(gyro_rig_node src/gyro_rig_node.cpp)

ament_target_dependencies(gyro_rig_node ${deps})

install(TARGETS gyro_rig_node
  DESTINATION lib/${PROJECT_NAME})

file(GLOB python_programs RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)

install(PROGRAMS 
  ${python_programs}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
