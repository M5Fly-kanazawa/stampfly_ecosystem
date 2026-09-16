# vehicle Unit Test — runs on host PC
# vehicle 単体テスト — ホストPC上で実行

CXX = g++
# -D_USE_MATH_DEFINES: MinGW's strict -std=c++17 hides M_PI (a POSIX/MSVC
# extension, not ISO C++) unless this is set before <cmath> is included.
# -D_USE_MATH_DEFINES: MinGWの-std=c++17は、<cmath>を読む前にこれを定義しない
# 限りM_PI（POSIX/MSVC拡張でISO C++非標準）を隠す。
CXXFLAGS = -std=c++17 -D_USE_MATH_DEFINES -Wall -Wextra -O2 \
    -I. \
    -I../components/sf_math/include \
    -I../components/sf_estimator_eskf/include \
    -I../components/sf_controller_pid/include -I../components/sf_autotune/include \
    -I../components/sf_core/include \
    -I../components/sf_estimator/include \
    -I../components/sf_controller/include \
    -I../components/sf_state/include \
    -I../components/sf_takeoff_landing/include \
    -I../components/sf_telemetry/include

SRCS = test_main.cpp \
       ../components/sf_estimator_eskf/eskf_core.cpp ../components/sf_autotune/autotune.cpp \
       ../components/sf_takeoff_landing/takeoff_landing.cpp

TARGET = test_vehicle

# Flip maneuver: ESKF attitude-correction hold + ToF re-acquisition
# (flip-maneuver-plan.md §3.6). A separate binary (own main()/TEST() framework,
# same pattern as test_main.cpp) so it does not need to join test_main.cpp's
# runner list; it links the SAME eskf_core.cpp translation unit as test_vehicle.
# 宙返りマニューバ: ESKF 姿勢補正ホールド + ToF 再取り込み（plan §3.6）。
# 独自バイナリ（test_main.cpp と同型の own main()/TEST() フレームワーク）とし、
# test_main.cpp のランナー一覧に参加する必要をなくす。test_vehicle と同じ
# eskf_core.cpp 翻訳単位をリンクする。
SRCS_ESKF_FLIP = test_eskf_flip_hold.cpp ../components/sf_estimator_eskf/eskf_core.cpp
TARGET_ESKF_FLIP = test_eskf_flip_hold

# Flip maneuver: setpoint sequencer (flip-maneuver-plan.md §3.2/§3.5/§7).
# Same own-binary pattern as TARGET_ESKF_FLIP above — FlipSequencer has no
# ESP-IDF/FreeRTOS/param-system dependency (see flip_sequencer.hpp), so its
# own main()/TEST() framework needs only flip_sequencer.cpp itself.
# 宙返りマニューバ: 設定点列生成器（plan §3.2/§3.5/§7）。上の TARGET_ESKF_FLIP
# と同じ独自バイナリの形 — FlipSequencer は ESP-IDF/FreeRTOS/paramシステムに
# 依存しない（flip_sequencer.hpp 参照）ため、own main()/TEST() フレームワークは
# flip_sequencer.cpp 自身だけをリンクすればよい。
SRCS_FLIP_SEQUENCER = test_flip_sequencer.cpp ../components/sf_controller_pid/flip_sequencer.cpp
TARGET_FLIP_SEQUENCER = test_flip_sequencer

# NOTE: test_state_manager is temporarily disabled. It compiled the firmware
# state_manager/failsafe through the now-removed simulator/sils/compat shim and
# sils_topics.cpp (deleted in the SILS clean-slate, 2026-05-31). It will be
# re-wired onto the new SILS host shim; test_state_manager.cpp is kept for that.
# 注: test_state_manager は一時無効化。削除済み simulator/sils/compat シム +
# sils_topics.cpp 経由でビルドしていたため(SILS更地化)。新SILSのホストシム上で
# 再配線予定。test_state_manager.cpp は再配線のため残置。

.PHONY: all clean test

all: $(TARGET) $(TARGET_ESKF_FLIP) $(TARGET_FLIP_SEQUENCER)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_ESKF_FLIP): $(SRCS_ESKF_FLIP)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_FLIP_SEQUENCER): $(SRCS_FLIP_SEQUENCER)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

test: all
	./$(TARGET)
	./$(TARGET_ESKF_FLIP)
	./$(TARGET_FLIP_SEQUENCER)

clean:
	rm -f $(TARGET) $(TARGET_ESKF_FLIP) $(TARGET_FLIP_SEQUENCER) test_state_manager
