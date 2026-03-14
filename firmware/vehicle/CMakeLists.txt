# StampFly RTOS Skeleton
# ESP-IDF v5.4 Project

cmake_minimum_required(VERSION 3.16)

# Set the components directory
# Include both local components and common shared components
# ローカルコンポーネントと共通コンポーネントの両方を含める
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
    ${CMAKE_CURRENT_SOURCE_DIR}/../platform/components
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stampfly_rtos_skelton)
