# =============================================================================
# StampFly SIL — host build (physics-based, MuJoCo, algorithm-independent)
# StampFly SIL — ホストビルド（物理ベース・MuJoCo・アルゴリズム非依存）
#
# P1.0 deliverable: two smoke tests that de-risk the two big unknowns —
#   (1) MuJoCo builds and steps on this host   (target: mujoco_smoke)
#   (2) the firmware algorithm cores compile and run on host via interfaces
#       (target: cores_smoke)
#
# P1.0 の成果物: 2つの大きな未知を潰すスモークテスト —
#   (1) MuJoCo がこのホストでビルド・ステップする       (target: mujoco_smoke)
#   (2) ファームの算法コアが host 上でインターフェース経由で動く
#       (target: cores_smoke)
#
# Source of truth for the SIL design: simulator/sil/RESET_PLAN.md
# SIL 設計の正: simulator/sil/RESET_PLAN.md
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(stampfly_sil LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Build switches — independent so cores_smoke can build without the (slow)
# MuJoCo fetch, and vice versa.
# ビルドスイッチ — 独立にして、cores_smoke を（遅い）MuJoCo 取得なしで
# ビルドできるように。逆も同様。
option(SIL_BUILD_CORES_SMOKE  "Build the firmware-cores host smoke test" ON)
option(SIL_BUILD_MUJOCO_SMOKE "Build the MuJoCo host smoke test"         ON)

# Paths
# パス
set(VN ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/vehicle_new)
set(SIL_COMPAT ${CMAKE_CURRENT_SOURCE_DIR}/compat)

# -----------------------------------------------------------------------------
# Firmware algorithm cores as a host library (compiled unmodified, via the
# compat stubs). Code Identity: same sources the firmware ships.
# ファームの算法コアを host ライブラリ化（compat スタブ経由で無改変
# コンパイル）。Code Identity: ファームが出荷するのと同じソース。
# -----------------------------------------------------------------------------
if(SIL_BUILD_CORES_SMOKE)
  add_library(sf_cores STATIC
    ${VN}/components/sf_estimator_eskf/eskf_estimator.cpp
    ${VN}/components/sf_estimator_eskf/eskf_core.cpp
    ${VN}/components/sf_controller_pid/pid_controller.cpp
    ${VN}/components/sf_core/params.cpp
    ${SIL_COMPAT}/nvs_shim.cpp
    ${SIL_COMPAT}/clock_shim.cpp
  )
  target_include_directories(sf_cores PUBLIC
    ${SIL_COMPAT}
    ${VN}/components/sf_math/include
    ${VN}/components/sf_core/include
    ${VN}/components/sf_estimator/include
    ${VN}/components/sf_estimator_eskf/include
    ${VN}/components/sf_controller/include
    ${VN}/components/sf_controller_pid/include
    ${VN}/components/sf_state/include
  )

  add_executable(cores_smoke smoke/cores_smoke.cpp)
  target_link_libraries(cores_smoke PRIVATE sf_cores)
endif()

# -----------------------------------------------------------------------------
# MuJoCo smoke (fetches MuJoCo 3.9.0 the first time — can take a few minutes).
# MuJoCo スモーク（初回は MuJoCo 3.9.0 を取得 — 数分かかることがある）。
# -----------------------------------------------------------------------------
if(SIL_BUILD_MUJOCO_SMOKE)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/mujoco.cmake)

  add_executable(mujoco_smoke smoke/mujoco_smoke.cpp)
  target_link_libraries(mujoco_smoke PRIVATE mujoco)
endif()
