cmake_minimum_required(VERSION 3.8)
project(fullfield_positioning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(bupt_can REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

aux_source_directory (src SRC_LIST)
include_directories (include)

add_executable(odometer_publisher ${SRC_LIST} ~/bupt_code/bupt_can_node/src/bupt_can.cpp)
#add_executable(odometer_publisher src/odometer_publisher.cpp)

ament_target_dependencies(odometer_publisher 
  rclcpp
  nav_msgs 
  bupt_can 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  tf2
)

install(TARGETS
  odometer_publisher
  DESTINATION lib/${PROJECT_NAME}
)
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
