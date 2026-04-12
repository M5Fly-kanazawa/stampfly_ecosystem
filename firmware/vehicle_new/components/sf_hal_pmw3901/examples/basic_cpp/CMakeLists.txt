cmake_minimum_required(VERSION 3.16)

# Include PMW3901 component from project root
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(pmw3901_cpp_example)
