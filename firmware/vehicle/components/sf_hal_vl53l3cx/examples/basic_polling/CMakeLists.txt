cmake_minimum_required(VERSION 3.16)

# Add stampfly_tof component from parent directory
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(basic_polling)
