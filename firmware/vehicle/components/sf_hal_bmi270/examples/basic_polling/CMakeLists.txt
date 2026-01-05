# BMI270 Basic Polling Example

cmake_minimum_required(VERSION 3.16)

# Add BMI270 driver component
set(EXTRA_COMPONENT_DIRS "../../components/bmi270_driver")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bmi270_basic_polling)
