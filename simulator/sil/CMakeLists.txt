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
option(SIL_BUILD_RTOS_SMOKE   "Build the host RTOS-emulator smoke test"  ON)
option(SIL_BUILD_FRAMES_TEST  "Build the coordinate-frames unit test"    ON)
option(SIL_BUILD_NOISE_TEST   "Build the sensor-noise (N0) unit test"    ON)

# Paths
# パス
set(VN ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/vehicle)
set(SIL_COMPAT ${CMAKE_CURRENT_SOURCE_DIR}/compat)

# -----------------------------------------------------------------------------
# Firmware algorithm cores as a host library (compiled unmodified, via the
# compat stubs). Code Identity: same sources the firmware ships.
# ファームの算法コアを host ライブラリ化（compat スタブ経由で無改変
# コンパイル）。Code Identity: ファームが出荷するのと同じソース。
# -----------------------------------------------------------------------------
if(SIL_BUILD_CORES_SMOKE OR SIL_BUILD_RTOS_SMOKE)
  add_library(sf_cores STATIC
    ${VN}/components/sf_estimator_eskf/eskf_estimator.cpp
    ${VN}/components/sf_estimator_eskf/eskf_core.cpp
    ${VN}/components/sf_estimator_complementary/complementary_estimator.cpp
    ${VN}/components/sf_controller_pid/pid_controller.cpp
    ${VN}/components/sf_takeoff_landing/takeoff_landing.cpp   # ToF ground/airborne detect (imu_task)
    ${VN}/components/sf_calibration/calibration.cpp           # boot gyro/accel bias calib (imu_task)
    ${VN}/components/sf_failsafe/failsafe.cpp                 # ImuAnomalyDetector (imu_task, 400Hz impact/gyro)
    ${VN}/components/sf_core/params.cpp
    ${SIL_COMPAT}/nvs_shim.cpp
    ${SIL_COMPAT}/clock_shim.cpp
    # Windows/MinGW-only compat TU (empty translation unit elsewhere): defines the
    # stdout/stderr shadow globals declared by esp_idf_host/cstdio, which is on
    # sf_cores's PUBLIC include path just below — so it reaches every consumer
    # (cores_smoke, rtos_smoke, hover_smoke, rate_tune) whenever any firmware
    # source they compile reads/writes stdout/stderr (e.g. fprintf(stderr, ...)
    # in error paths), not just emu_vehicle/emu_vehicle_old (which carry their
    # own copy since they don't link sf_cores). See esp_idf_host/cstdio for why
    # this shadow exists.
    # Windows/MinGW 限定の互換 TU（他では空翻訳単位）: esp_idf_host/cstdio が宣言する
    # stdout/stderr シャドウ変数を実体化する。同ヘッダは直下の sf_cores の PUBLIC
    # include パスにあるため、cores_smoke/rtos_smoke/hover_smoke/rate_tune が
    # コンパイルするどの firmware ソースが stdout/stderr を読み書きしても
    # （エラー経路の fprintf(stderr, ...) 等）届く — sf_cores をリンクしない
    # （自前でコピーを持つ）emu_vehicle/emu_vehicle_old だけではない。このシムが
    # 存在する理由は esp_idf_host/cstdio 参照。
    ${CMAKE_CURRENT_SOURCE_DIR}/esp_idf_host/win_stdio_shim.cpp
  )
  target_include_directories(sf_cores PUBLIC
    ${SIL_COMPAT}
    ${CMAKE_CURRENT_SOURCE_DIR}/esp_idf_host   # compat/esp_log.h re-exports esp_heap_caps.h (lives here)
    ${VN}/components/sf_math/include
    ${VN}/components/sf_core/include
    ${VN}/components/sf_estimator/include
    ${VN}/components/sf_estimator_eskf/include
    ${VN}/components/sf_estimator_complementary/include
    ${VN}/components/sf_controller/include
    ${VN}/components/sf_controller_pid/include
    ${VN}/components/sf_state/include
    ${VN}/components/sf_takeoff_landing/include
    ${VN}/components/sf_calibration/include
    ${VN}/components/sf_failsafe/include
  )
endif()

if(SIL_BUILD_CORES_SMOKE)
  add_executable(cores_smoke smoke/cores_smoke.cpp)
  target_link_libraries(cores_smoke PRIVATE sf_cores)
endif()

# -----------------------------------------------------------------------------
# RTOS-emulator smoke: run the real ImuTask/ControlTask/StateTask on the host
# cooperative scheduler. Sensor HALs are replaced at the wrapper seam (sim_hal).
# RTOS エミュレータのスモーク: 実 ImuTask/ControlTask/StateTask をホストの
# 協調スケジューラで走らせる。センサ HAL はラッパー境界で差し替え（sim_hal）。
# -----------------------------------------------------------------------------
if(SIL_BUILD_RTOS_SMOKE)
  find_package(Threads REQUIRED)

  add_executable(rtos_smoke
    smoke/rtos_smoke.cpp
    rtos/scheduler.cpp
    rtos/esp_timer_shim.cpp               # esp_timer periodic → deterministic 400Hz source
    sim_hal/sim_bmi270.cpp
    sim_hal/sim_motor_driver.cpp          # host MotorDriver stub (no driver/ledc.h)
    ${VN}/tasks/imu_task.cpp
    ${VN}/tasks/control_task.cpp
    ${VN}/tasks/state_task.cpp
    ${VN}/components/sf_state/state_manager.cpp
    ${VN}/components/sf_actuator/actuator.cpp   # real X-quad mixer (control_task wires it)
  )
  target_include_directories(rtos_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/rtos
    ${CMAKE_CURRENT_SOURCE_DIR}/sim_hal   # sim BMI270 wrapper replaces the HAL
    ${VN}/main                            # config.hpp
    ${VN}/tasks                           # tasks.hpp (sf::tasks::control_handle)
    ${VN}/components/sf_actuator/include   # actuator.hpp (mixer)
    ${VN}/components/sf_hal_motor/include  # motor_driver.hpp (stubbed by sim_hal)
  )
  target_link_libraries(rtos_smoke PRIVATE sf_cores Threads::Threads)
endif()

# -----------------------------------------------------------------------------
# MuJoCo smoke (fetches MuJoCo 3.9.0 the first time — can take a few minutes).
# MuJoCo スモーク（初回は MuJoCo 3.9.0 を取得 — 数分かかることがある）。
# -----------------------------------------------------------------------------
if(SIL_BUILD_MUJOCO_SMOKE)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/mujoco.cmake)

  add_executable(mujoco_smoke smoke/mujoco_smoke.cpp)
  target_link_libraries(mujoco_smoke PRIVATE mujoco)

  # Physics smoke: StampFly model hovers; built-in sensors agree with the
  # StampFly frame convention via the frames module.
  # 物理スモーク: StampFly モデルがホバーし、内蔵センサが frames 経由で一致。
  add_executable(physics_smoke smoke/physics_smoke.cpp)
  target_include_directories(physics_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/frames
    ${VN}/components/sf_math/include
  )
  target_link_libraries(physics_smoke PRIVATE mujoco)

  # Mesh inspector: print mesh Z-extents / radius profiles (for placing visual
  # meshes — motor-can top, prop hub, etc.).
  # メッシュ検査: メッシュの Z 範囲・半径プロファイルを表示（視覚メッシュの
  # 配置用 — モータ缶上面・プロペラハブ等）。
  add_executable(mesh_inspect smoke/mesh_inspect.cpp)
  target_link_libraries(mesh_inspect PRIVATE mujoco)

  # Plant smoke: the SIL physics plant (motor model + truth + synthetic FRD
  # sensors) hovers and its synthetic sensors agree with physics truth.
  # プラントスモーク: SIL 物理プラント（モータモデル＋真値＋合成 FRD センサ）が
  # ホバーし、合成センサが物理真値と一致する。
  add_executable(plant_smoke smoke/plant_smoke.cpp plant/plant.cpp)
  target_include_directories(plant_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/plant
    ${CMAKE_CURRENT_SOURCE_DIR}/frames
    ${VN}/components/sf_math/include
    ${VN}/components/sf_core/include   # data_types.hpp (header-only)
  )
  target_link_libraries(plant_smoke PRIVATE mujoco)

  # Closed-loop hover (P1.2c / gate G3): the real firmware loop on the RTOS
  # emulator + the MuJoCo Plant closing the loop. Needs both the RTOS infra
  # (sf_cores + threads + esp_timer shim) and MuJoCo.
  # 閉ループホバー（P1.2c / ゲート G3）: RTOS エミュレータ上の実ファームループ＋
  # MuJoCo Plant でループを閉じる。RTOS インフラと MuJoCo の両方が必要。
  find_package(Threads REQUIRED)
  add_executable(hover_smoke
    smoke/hover_smoke.cpp
    plant/plant.cpp
    rtos/scheduler.cpp
    rtos/esp_timer_shim.cpp
    sim_hal/sim_bmi270.cpp
    sim_hal/sim_motor_driver.cpp
    ${VN}/tasks/imu_task.cpp
    ${VN}/tasks/control_task.cpp
    ${VN}/tasks/state_task.cpp
    ${VN}/components/sf_state/state_manager.cpp
    ${VN}/components/sf_actuator/actuator.cpp
  )
  target_include_directories(hover_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/rtos
    ${CMAKE_CURRENT_SOURCE_DIR}/sim_hal
    ${CMAKE_CURRENT_SOURCE_DIR}/plant
    ${CMAKE_CURRENT_SOURCE_DIR}/frames
    ${VN}/main
    ${VN}/tasks
    ${VN}/components/sf_actuator/include
    ${VN}/components/sf_hal_motor/include
    ${VN}/components/sf_math/include
    ${VN}/components/sf_core/include
  )
  target_link_libraries(hover_smoke PRIVATE sf_cores Threads::Threads mujoco)

  # Rate-loop tuning harness: yaw-rate step with runtime-injected gains, for the
  # SIL-backed recalibration sweep. Same wiring as hover_smoke.
  # レートループ調整ハーネス: 実行時注入ゲインでヨーレートステップ。掃引用。
  add_executable(rate_tune
    smoke/rate_tune.cpp
    plant/plant.cpp
    rtos/scheduler.cpp
    rtos/esp_timer_shim.cpp
    sim_hal/sim_bmi270.cpp
    sim_hal/sim_motor_driver.cpp
    ${VN}/tasks/imu_task.cpp
    ${VN}/tasks/control_task.cpp
    ${VN}/tasks/state_task.cpp
    ${VN}/components/sf_state/state_manager.cpp
    ${VN}/components/sf_actuator/actuator.cpp
  )
  target_include_directories(rate_tune PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/rtos
    ${CMAKE_CURRENT_SOURCE_DIR}/sim_hal
    ${CMAKE_CURRENT_SOURCE_DIR}/plant
    ${CMAKE_CURRENT_SOURCE_DIR}/frames
    ${VN}/main
    ${VN}/tasks
    ${VN}/components/sf_actuator/include
    ${VN}/components/sf_hal_motor/include
    ${VN}/components/sf_math/include
    ${VN}/components/sf_core/include
  )
  target_link_libraries(rate_tune PRIVATE sf_cores Threads::Threads mujoco)
endif()

# -----------------------------------------------------------------------------
# Coordinate-frames unit test: the ONLY coordinate transform in the SIL.
# 座標系の単体テスト: SIL で唯一の座標変換。
# -----------------------------------------------------------------------------
if(SIL_BUILD_FRAMES_TEST)
  add_executable(frames_test frames/frames_test.cpp)
  target_include_directories(frames_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/frames
    ${VN}/components/sf_math/include   # header-only Vec3/Quat
  )
endif()

# -----------------------------------------------------------------------------
# Sensor-noise (N0) unit test: white σ, off=clean, startup bias, RW √t,
# determinism. Physics-free (no MuJoCo) — tests SensorNoise in isolation.
# センサノイズ(N0)単体テスト: 白色σ・off=クリーン・起動バイアス・RW√t・決定論。
# 物理非依存（MuJoCo 不要）— SensorNoise 単体を検証。
# -----------------------------------------------------------------------------
if(SIL_BUILD_NOISE_TEST)
  add_executable(noise_test smoke/noise_test.cpp)
  target_include_directories(noise_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/plant)
endif()

# -----------------------------------------------------------------------------
# StampFly Emulator — run the REAL firmware app_main() on the host.
#
# The ESP-IDF host platform (esp_idf_host/ + compat/) replaces the SoC; the
# StampFly chips/board are emulated as virtual devices (E1+). The firmware is
# compiled UNMODIFIED and firmware-agnostic (E5 builds firmware/vehicle_old the
# same way). E0 goal: app_main() links and all 14 tasks run on the cooperative
# scheduler. The shim chain is C++-only (esp_err.h uses <cstdint>), so the .c
# HAL/driver sources are compiled as C++.
#
# StampFly エミュレータ — 実ファームの app_main() をホストで走らせる。esp_idf_host/＋
# compat/ が SoC を肩代わりし、チップ/ボードは仮想デバイス（E1+）。ファームは無改変・
# ファーム非依存（E5 で firmware/vehicle_old も同様）。.c も C++ としてコンパイル。
# -----------------------------------------------------------------------------
option(SIL_BUILD_EMU "Build the StampFly emulator (real firmware app_main on host)" ON)
if(SIL_BUILD_EMU)
  find_package(Threads REQUIRED)
  set(EMU_HOST ${CMAKE_CURRENT_SOURCE_DIR}/esp_idf_host)

  # Windows/MinGW-only compat TU: defines the stdout/stderr shadow globals
  # declared by esp_idf_host/cstdio (see that header for why). Empty
  # translation unit on non-Windows hosts, so it is harmless to always list.
  # Windows/MinGW 限定の互換 TU: esp_idf_host/cstdio が宣言する stdout/stderr
  # シャドウ変数を実体化する（理由は同ヘッダ参照）。非 Windows では空の翻訳単位
  # になるので常に列挙して無害。
  set(EMU_WIN_COMPAT_SRCS ${EMU_HOST}/win_stdio_shim.cpp)

  # All firmware sources under main/tasks/components, EXCLUDING the VL53L3CX
  # vendor docs/ (STM32 CubeIDE examples) and any examples/tests/build output.
  # main/tasks/components の全ソース。VL53 の docs/（STM32例）・examples/test は除外。
  file(GLOB_RECURSE EMU_FW_SRCS CONFIGURE_DEPENDS
       ${VN}/main/*.cpp ${VN}/main/*.c
       ${VN}/tasks/*.cpp ${VN}/tasks/*.c
       ${VN}/components/*.cpp ${VN}/components/*.c)
  list(FILTER EMU_FW_SRCS EXCLUDE REGEX "/(docs|examples|test|build)/|_test\\.|/CubeIDE")

  # The C HAL/driver sources (BMI270, PMW3901, VL53L3CX BareDriver) compile as C
  # against the C-compatible host shims — exactly as on real ESP-IDF. The .cpp
  # firmware compiles as C++. (Earlier attempt compiled .c as C++ and hit
  # extern "C" + C++-stdlib clashes; C-compiling the C drivers is the right fix.)
  # C の HAL/ドライバは C 互換シムに対し C でコンパイル（実 ESP-IDF と同じ）。
  set(CMAKE_C_STANDARD 11)

  # Public include dirs of every component + the driver-internal subdirs.
  # 各コンポーネントの公開 include ＋ ドライバ内部の src サブディレクトリ。
  file(GLOB EMU_FW_INCS LIST_DIRECTORIES true ${VN}/components/*/include)
  list(APPEND EMU_FW_INCS
       ${VN}/components/sf_hal_vl53l3cx/include/vl53lx
       ${VN}/components/sf_hal_vl53l3cx/src
       ${VN}/components/sf_hal_vl53l3cx/src/vl53lx
       ${VN}/components/sf_hal_bmi270/src
       ${VN}/components/sf_hal_pmw3901/src
       ${VN}/main ${VN}/tasks
       ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/common/protocol/include)  # shared protocol (sf_comm)

  add_executable(emu_vehicle
    ${CMAKE_CURRENT_SOURCE_DIR}/emu/emu_main.cpp
    ${EMU_FW_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/vl53_platform_glue.c  # missing fw platform fn
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/virtual_board.cpp     # E1: BMI270 + motors → Plant
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/vl53_device.cpp       # E2: VL53L3CX ToF chip model
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/pmw3901_device.cpp    # PMW3901 optical-flow chip model
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/espnow_hub.cpp        # shared ESP-NOW hub (comm.cpp uses it)
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/emu_record.cpp        # recorder (passive; hub records frames)
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/emu_trajectory.cpp    # P8: review-video trajectory (SIL_EMU_TRAJ)
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/emu_vehicle_glue.cpp  # estimate overlay (ESKF est → trajectory)
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/scenario.cpp          # P8: *.scn scripted-input driver
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/scenario_inject.cpp   # P8: 14B ControlPacket builder (SSOT)
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/console_feeder.cpp    # P8: scripted console bytes → stdin
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/esp_console_shim.cpp  # Phase 6: CLI console registry (cli_task)
    plant/plant.cpp                                           # MuJoCo physics plant
    rtos/scheduler.cpp
    rtos/esp_timer_shim.cpp
    compat/clock_shim.cpp
    compat/nvs_shim.cpp
    ${EMU_WIN_COMPAT_SRCS})
  # esp_idf_host + compat FIRST so the shim headers win over anything else.
  # esp_idf_host ＋ compat を最優先にしてシムヘッダを優先解決させる。
  target_include_directories(emu_vehicle BEFORE PRIVATE
    ${EMU_HOST} ${SIL_COMPAT} ${CMAKE_CURRENT_SOURCE_DIR}/rtos
    ${CMAKE_CURRENT_SOURCE_DIR}/plant ${CMAKE_CURRENT_SOURCE_DIR}/frames
    ${CMAKE_CURRENT_SOURCE_DIR}/devices ${EMU_FW_INCS})
  target_link_libraries(emu_vehicle PRIVATE Threads::Threads mujoco)

  # ---------------------------------------------------------------------------
  # Second firmware: firmware/vehicle_old (LEGACY, 87 real flights, complete).
  # Proves the emulator is firmware-agnostic — a DIFFERENT codebase (own
  # drivers, own services, own state machine) runs on the SAME libsf_emu (host
  # platform + virtual board + Plant). The SIL infra (plant/virtual_board)
  # keeps using the promoted vehicle's sf_core/sf_math types; the firmware
  # boundary is raw chip bytes, so there is no firmware coupling (no
  # header-name collisions — verified).
  # 2本目: レガシーファーム firmware/vehicle_old（完成・87飛行）。別実装が同じ
  # 基盤で走る＝ファーム非依存の実証。SIL基盤は昇格後vehicleの
  # sf_core/sf_math を使い続け、ファーム境界は raw チップバイトなので結合なし。
  # ---------------------------------------------------------------------------
  set(VV ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/vehicle_old)
  file(GLOB_RECURSE EMU_VV_SRCS CONFIGURE_DEPENDS
       ${VV}/main/*.cpp ${VV}/main/*.c
       ${VV}/components/*.cpp ${VV}/components/*.c)
  list(FILTER EMU_VV_SRCS EXCLUDE REGEX "/(docs|examples|test|build)/|_test\\.|/CubeIDE")

  file(GLOB EMU_VV_INCS LIST_DIRECTORIES true ${VV}/components/*/include)
  list(APPEND EMU_VV_INCS
       ${VV}/components/sf_hal_vl53l3cx/include/vl53lx
       ${VV}/components/sf_hal_vl53l3cx/src
       ${VV}/components/sf_hal_vl53l3cx/src/vl53lx
       ${VV}/components/sf_hal_bmi270/src
       ${VV}/components/sf_hal_pmw3901/src
       ${VV}/main
       ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/common/protocol/include  # shared protocol
       # SIL infra (Plant/virtual_board) types — no collision with old firmware:
       ${VN}/components/sf_core/include ${VN}/components/sf_math/include)

  add_executable(emu_vehicle_old
    ${CMAKE_CURRENT_SOURCE_DIR}/emu/emu_main_generic.cpp
    ${EMU_VV_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/vl53_platform_glue.c   # missing fw platform fn
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/emu_vehicle_old_glue.cpp  # workshop-only weak symbol
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/virtual_board.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/vl53_device.cpp        # E2: VL53L3CX ToF chip model
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/pmw3901_device.cpp     # PMW3901 optical-flow chip model
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/espnow_hub.cpp          # shared ESP-NOW recv/send hub
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/scenario_inject.cpp     # E6: shared ControlPacket builder
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/virtual_pilot_vehicle.cpp  # ESP-NOW virtual pilot (E3)
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/scenario.cpp           # E6: *.scn parser + driver task
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/console_feeder.cpp     # E6: scripted console bytes
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/esp_console_shim.cpp   # E6: dispatching console registry
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/emu_record.cpp         # E6: events.jsonl recorder
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/emu_trajectory.cpp     # review-video trajectory recorder
    plant/plant.cpp
    rtos/scheduler.cpp
    rtos/esp_timer_shim.cpp
    compat/clock_shim.cpp
    compat/nvs_shim.cpp
    ${EMU_WIN_COMPAT_SRCS})
  target_include_directories(emu_vehicle_old BEFORE PRIVATE
    ${EMU_HOST} ${SIL_COMPAT} ${CMAKE_CURRENT_SOURCE_DIR}/rtos
    ${CMAKE_CURRENT_SOURCE_DIR}/plant ${CMAKE_CURRENT_SOURCE_DIR}/frames
    ${CMAKE_CURRENT_SOURCE_DIR}/devices ${EMU_VV_INCS})
  target_link_libraries(emu_vehicle_old PRIVATE Threads::Threads mujoco)
endif()

# -----------------------------------------------------------------------------
# VL53L3CX offline gen4 probe (Milestone 2 visibility tool). Drives the UNMODIFIED
# ST BareDriver against the chip model (devices/vl53_device.cpp) directly on the
# host — NO FreeRTOS / esp_timer / MuJoCo / scheduler — and prints, per frame, the
# full MultiRangingData AND the decoded histogram_bin_data gen4 consumes. OFF by
# default: a debugging tool, not part of the regression set. Fast to rebuild/run.
# VL53L3CX オフライン gen4 プローブ（M2 可視化ツール）。無改変 ST ドライバをチップモデルに
# 直結（FreeRTOS/MuJoCo/スケジューラ無し）。OFF 既定。高速反復用。
# -----------------------------------------------------------------------------
option(SIL_BUILD_VL53_PROBE "Build the offline VL53L3CX gen4 probe (Milestone 2)" OFF)
if(SIL_BUILD_VL53_PROBE)
  set(VL53_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/vehicle_old/components/sf_hal_vl53l3cx)

  # ST driver core (compiled UNMODIFIED). The firmware's vl53lx_platform.c
  # (FreeRTOS/esp) is deliberately NOT in this list — the probe links its own host
  # platform port instead (so no scheduler/timer deps leak in).
  # ST ドライバコア（無改変）。ファームの vl53lx_platform.c は不使用（プローブが自前 port）。
  file(GLOB VL53_CORE_SRCS ${VL53_SRC}/src/vl53lx/*.c)
  list(APPEND VL53_CORE_SRCS
       ${VL53_SRC}/src/vl53lx_platform_ipp.c
       ${VL53_SRC}/src/vl53lx_outlier_filter.c)

  add_executable(vl53_probe
    smoke/vl53_probe.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/vl53_device.cpp
    ${VL53_CORE_SRCS})
  set_target_properties(vl53_probe PROPERTIES C_STANDARD 11)
  # esp_idf_host (driver/i2c_master.h) + compat (esp_err.h) are header-only here.
  target_include_directories(vl53_probe BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/esp_idf_host ${SIL_COMPAT}
    ${CMAKE_CURRENT_SOURCE_DIR}/devices
    ${VL53_SRC}/include
    ${VL53_SRC}/include/vl53lx
    ${VL53_SRC}/src
    ${VL53_SRC}/src/vl53lx)
endif()

# -----------------------------------------------------------------------------
# PMW3901 offline flow probe — drives the chip model (devices/pmw3901_device.cpp)
# directly on the host (no FreeRTOS / MuJoCo) to verify the SPI register protocol,
# the motion-burst framing, and the translational flow round-trip deterministically.
# ON by default: it is fast, self-contained, and part of the regression set.
# PMW3901 オフラインフロー probe — チップモデルを host で直接駆動し、SPI レジスタ規約・
# motion burst・並進フロー round-trip を決定論的に検証。既定 ON（高速・自己完結・回帰）。
# -----------------------------------------------------------------------------
option(SIL_BUILD_PMW3901_PROBE "Build the offline PMW3901 optical-flow probe" ON)
if(SIL_BUILD_PMW3901_PROBE)
  add_executable(pmw3901_probe
    smoke/pmw3901_probe.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/pmw3901_device.cpp)
  target_include_directories(pmw3901_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/devices)
endif()
