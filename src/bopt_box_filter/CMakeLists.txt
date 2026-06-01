cmake_minimum_required(VERSION 3.8)
project(bopt_box_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(bopt_box_filter_lib
  src/box_filter.cpp
  src/box_filter_node.cpp)
target_include_directories(bopt_box_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bopt_box_filter_lib
  rclcpp sensor_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs geometry_msgs)

add_executable(bopt_box_filter_node src/main.cpp)
target_link_libraries(bopt_box_filter_node bopt_box_filter_lib)
ament_target_dependencies(bopt_box_filter_node
  rclcpp sensor_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs geometry_msgs)

install(DIRECTORY include/
  DESTINATION include/)

install(TARGETS bopt_box_filter_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS bopt_box_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_box_filter test/test_box_filter.cpp)
  target_link_libraries(test_box_filter bopt_box_filter_lib)
endif()

ament_package()
