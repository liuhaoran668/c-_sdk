cmake_minimum_required(VERSION 3.16)
project(linkerhand_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(linkerhand_msgs REQUIRED)

# 引入 SDK（作为子目录构建，保持 SDK 独立）
set(BUILD_TESTS OFF CACHE BOOL "Disable SDK test programs when building as a subdirectory" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../sdk ${CMAKE_CURRENT_BINARY_DIR}/sdk)

add_executable(linkerhand_node src/linkerhand_node.cpp)
target_include_directories(linkerhand_node PRIVATE include)
target_link_libraries(linkerhand_node linkerhand_cpp_sdk)
ament_target_dependencies(linkerhand_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
  linkerhand_msgs
)

install(TARGETS linkerhand_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
