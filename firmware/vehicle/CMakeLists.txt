# StampFly RTOS Skeleton
# ESP-IDF v5.4 Project

cmake_minimum_required(VERSION 3.16)

# Set the components directory
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stampfly_rtos_skelton)
