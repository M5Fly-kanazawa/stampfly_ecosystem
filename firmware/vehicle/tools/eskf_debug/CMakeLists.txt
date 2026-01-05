cmake_minimum_required(VERSION 3.14)
project(eskf_debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ESP-IDF types stub
add_library(esp_stubs INTERFACE)
target_include_directories(esp_stubs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/stubs)

# Math library (header-only)
add_library(sf_algo_math INTERFACE)
target_include_directories(sf_algo_math INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/sf_algo_math/include
)

# ESKF library (using actual component source with ESP-IDF stubs)
add_library(sf_algo_eskf
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/sf_algo_eskf/eskf.cpp
)
target_include_directories(sf_algo_eskf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/sf_algo_eskf/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)
# esp_stubs must come BEFORE other includes to override ESP-IDF headers
target_include_directories(sf_algo_eskf BEFORE PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/stubs)
target_link_libraries(sf_algo_eskf PUBLIC sf_algo_math)

# Main debug executable (simple test)
add_executable(eskf_debug main.cpp)
target_link_libraries(eskf_debug PRIVATE sf_algo_eskf)

# Replay tool (run ESKF on recorded sensor data)
add_executable(eskf_replay replay.cpp)
target_link_libraries(eskf_replay PRIVATE sf_algo_eskf)

# Unit test executable
add_executable(eskf_test eskf_test.cpp)
target_link_libraries(eskf_test PRIVATE sf_algo_eskf)
