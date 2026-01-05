cmake_minimum_required(VERSION 3.14)
project(pid_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# PID library (directly from component)
add_library(sf_algo_pid
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/sf_algo_pid/pid.cpp
)
target_include_directories(sf_algo_pid PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/sf_algo_pid/include
)

# Unit test executable
add_executable(pid_test pid_test.cpp)
target_link_libraries(pid_test PRIVATE sf_algo_pid)
