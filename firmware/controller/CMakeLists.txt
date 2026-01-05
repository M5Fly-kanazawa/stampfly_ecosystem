# ESP-IDF プロジェクト設定
cmake_minimum_required(VERSION 3.16)

# カスタムコンポーネントのパス
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware_controller)
