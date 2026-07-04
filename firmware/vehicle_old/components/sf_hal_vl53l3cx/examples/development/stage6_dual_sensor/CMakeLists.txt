# Stage 6: VL53L3CX Dual Sensor Operation
cmake_minimum_required(VERSION 3.16)

# Add parent directory (stampfly_tof component) to component search path
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stage6_dual_sensor)
