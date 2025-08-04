cmake_minimum_required(VERSION 3.10)  # CMake version check
project(simple_pid)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard

# Add main.cpp file of project root directory as source file
set(SOURCE_FILES PID.cpp)
#include_directories(${PROJECT_SOURCE_DIR})
#add_library(applib foo.cpp)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(simple_pid ${SOURCE_FILES})

