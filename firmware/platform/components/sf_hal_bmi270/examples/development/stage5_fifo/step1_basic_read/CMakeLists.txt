# Step 1: BMI270 FIFO Basic Manual Read
cmake_minimum_required(VERSION 3.16)

# Add parent directory as extra component
set(EXTRA_COMPONENT_DIRS ../../..)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bmi270_step1_fifo_basic)
