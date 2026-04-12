cmake_minimum_required(VERSION 3.16)

# Add the parent directory as an extra component directory
# This allows us to use the stampfly_tof component
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stage1_i2c_scan)
