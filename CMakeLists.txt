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

