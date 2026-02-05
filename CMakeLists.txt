cmake_minimum_required(VERSION 3.16)

project(linkerhand_cpp_sdk LANGUAGES CXX)

file(GLOB_RECURSE LINKERHAND_CPP_SDK_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(linkerhand_cpp_sdk ${LINKERHAND_CPP_SDK_SOURCES})

target_compile_features(linkerhand_cpp_sdk PUBLIC cxx_std_17)
target_include_directories(linkerhand_cpp_sdk PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

find_package(Threads REQUIRED)
target_link_libraries(linkerhand_cpp_sdk PRIVATE Threads::Threads)

# ============================================================================
# 测试程序
# ============================================================================
option(BUILD_TESTS "Build test programs" ON)

if(BUILD_TESTS)
  # 压力传感器测试
  add_executable(test_force_sensor tests/test_force_sensor.cpp)
  target_link_libraries(test_force_sensor PRIVATE linkerhand_cpp_sdk Threads::Threads)

  # 角度控制测试
  add_executable(test_angle tests/test_angle.cpp)
  target_link_libraries(test_angle PRIVATE linkerhand_cpp_sdk Threads::Threads)
endif()

