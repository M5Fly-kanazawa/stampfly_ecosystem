# Stage 3: BMI270 Polling Data Read Example
cmake_minimum_required(VERSION 3.16)

# Add parent directory as extra component
set(EXTRA_COMPONENT_DIRS ../..)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bmi270_stage3_polling)
