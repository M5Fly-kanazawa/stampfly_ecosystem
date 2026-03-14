# Stage 1: SPI Basic Communication Test Project
cmake_minimum_required(VERSION 3.16)

# Add parent directory (stampfly_imu component) to component search path
set(EXTRA_COMPONENT_DIRS ../..)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bmi270_stage1_spi_basic)
