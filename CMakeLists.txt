cmake_minimum_required(VERSION 3.5)
project(lidar_tape_correction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

add_library(intensity_distance_corrector
  src/intensity_distance_corrector.cpp
)

ament_target_dependencies(intensity_distance_corrector
  rclcpp
  sensor_msgs
)

add_executable(calibration_node
  src/calibration_node.cpp
)

target_link_libraries(calibration_node
  intensity_distance_corrector
)

ament_target_dependencies(calibration_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
)

add_executable(correction_node
  src/correction_node.cpp
)

target_link_libraries(correction_node
  intensity_distance_corrector
)

ament_target_dependencies(correction_node
  rclcpp
  sensor_msgs
  visualization_msgs
)

install(TARGETS
  calibration_node
  correction_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  intensity_distance_corrector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()