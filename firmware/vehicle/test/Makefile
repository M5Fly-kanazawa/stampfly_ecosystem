# vehicle Unit Test — runs on host PC
# vehicle 単体テスト — ホストPC上で実行

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 \
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

# NOTE: test_state_manager is temporarily disabled. It compiled the firmware
# state_manager/failsafe through the now-removed simulator/sils/compat shim and
# sils_topics.cpp (deleted in the SILS clean-slate, 2026-05-31). It will be
# re-wired onto the new SILS host shim; test_state_manager.cpp is kept for that.
# 注: test_state_manager は一時無効化。削除済み simulator/sils/compat シム +
# sils_topics.cpp 経由でビルドしていたため(SILS更地化)。新SILSのホストシム上で
# 再配線予定。test_state_manager.cpp は再配線のため残置。

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

test: all
	./$(TARGET)

clean:
	rm -f $(TARGET) test_state_manager
