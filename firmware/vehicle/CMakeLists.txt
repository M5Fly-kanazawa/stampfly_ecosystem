# StampFly vehicle — Vehicle Firmware
# StampFly vehicle — 機体ファームウェア
#
# @design requirements.md §1 — StampFly control firmware                [--]
# @design detailed_design.md §7 — Directory structure                   [--]

cmake_minimum_required(VERSION 3.16)

# Component directories
# コンポーネントディレクトリ
# ../common: shared ESP-NOW protocol definitions (firmware/common/protocol) —
#            ../common: 共有ESP-NOWプロトコル定義(firmware/common/protocol)
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stampfly_vehicle)
