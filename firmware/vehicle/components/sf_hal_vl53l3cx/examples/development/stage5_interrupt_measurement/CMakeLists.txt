# Stage 5: VL53L3CX Interrupt-based Distance Measurement
cmake_minimum_required(VERSION 3.16)

# Add parent directory (stampfly_tof component) to component search path
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stage5_interrupt_measurement)
