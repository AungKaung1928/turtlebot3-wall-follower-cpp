cmake_minimum_required(VERSION 3.8)
project(wall_following_cpp_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/pid_controller.cpp
  src/wall_detector.cpp
  src/wall_follower_controller.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(wall_follower_cpp src/main.cpp)
target_link_libraries(wall_follower_cpp ${PROJECT_NAME}_core)
ament_target_dependencies(wall_follower_cpp rclcpp)

install(TARGETS wall_follower_cpp DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz worlds description models
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
