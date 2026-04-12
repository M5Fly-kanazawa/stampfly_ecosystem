# Stage 4: VL53L3CX Polling Distance Measurement
cmake_minimum_required(VERSION 3.16)

# Add parent directory as extra component directory to access stampfly_tof component
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stage4_polling_measurement)
