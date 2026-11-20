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

# Experiment switches for A/B builds (branch exp/imu-cycle-budget-20261120).
# Pass extra compile definitions without editing any source file, e.g.
#   idf.py -B <dir> -D SDKCONFIG=<dir>/sdkconfig \
#          -D "SF_EXP_FLAGS=-DSF_EXP_ESKF_FAST=1;-DSF_EXP_ONE_KIND_PER_CYCLE=1" build
# Every switch defaults to 0 in the source, so a build without SF_EXP_FLAGS is
# the unchanged firmware.
# A/B ビルド用の実験スイッチ（ブランチ exp/imu-cycle-budget-20261120）。
# ソースを編集せずに追加のコンパイル定義を渡す（上の例）。どのスイッチも
# ソース側の既定は 0 なので、SF_EXP_FLAGS なしのビルドは変更前と同じファームになる。
if(DEFINED SF_EXP_FLAGS AND NOT "${SF_EXP_FLAGS}" STREQUAL "")
    idf_build_set_property(COMPILE_OPTIONS "${SF_EXP_FLAGS}" APPEND)
    message(STATUS "SF_EXP_FLAGS: ${SF_EXP_FLAGS}")
endif()

project(stampfly_vehicle)
