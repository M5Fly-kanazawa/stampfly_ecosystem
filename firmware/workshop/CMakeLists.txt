# StampFly Workshop Skeleton
# ESP-IDF v5.4 Project
#
# Reuses vehicle_old components via EXTRA_COMPONENT_DIRS
# (vehicle_new -> vehicle promotion moved the layered sf_hal_*/sf_algo_*/sf_svc_*
#  components to vehicle_old; workshop still depends on that layered API)
# vehicle_old コンポーネントを再利用
# （vehicle_new→vehicle 昇格で層分けコンポーネント sf_hal_*/sf_algo_*/sf_svc_* は
#  vehicle_old に移動した。workshop は引き続きその層分けAPIに依存する）

cmake_minimum_required(VERSION 3.16)

# Include vehicle_old components and common shared components
# vehicle_old コンポーネントと共通コンポーネントの両方を含める
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../vehicle_old/components
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stampfly_workshop)
