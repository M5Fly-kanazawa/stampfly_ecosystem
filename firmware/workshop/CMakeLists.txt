# StampFly Workshop — ESP-IDF v5.4 Project
# StampFly ワークショップ — ESP-IDF v5.4 プロジェクト
#
# Rebuilt on top of the current firmware/vehicle component base (cross-cutting
# rule R12, workshop_migration.md M5). The workshop main/ links the SAME
# sf_* components as vehicle/ (flat naming, Pub-Sub topics, 4-tier access) —
# there is no separate "workshop component set" to drift from vehicle.
# 現行 firmware/vehicle のコンポーネント群の上に再構築（横断ルール R12、
# workshop_migration.md M5）。workshop の main/ は vehicle/ と同じ sf_*
# コンポーネント（フラット命名・Pub-Subトピック・4階層アクセス）にリンクする —
# vehicle から乖離する「workshop 専用コンポーネント集合」は存在しない。

cmake_minimum_required(VERSION 3.16)

# Reuse vehicle's components + the shared protocol definitions.
# vehicle のコンポーネントと共有プロトコル定義を再利用する。
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../vehicle/components
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stampfly_workshop)
