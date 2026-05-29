# vehicle_new Unit Test — runs on host PC
# vehicle_new 単体テスト — ホストPC上で実行

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 \
    -I. \
    -I../components/sf_math/include \
    -I../components/sf_estimator_eskf/include \
    -I../components/sf_controller_pid/include \
    -I../components/sf_core/include \
    -I../components/sf_estimator/include \
    -I../components/sf_controller/include \
    -I../components/sf_state/include

SRCS = test_main.cpp \
       ../components/sf_estimator_eskf/eskf_core.cpp

TARGET = test_vehicle_new

# State transition / failsafe tests: compile the UNMODIFIED firmware sources
# through the simulator/sil/compat shim (Code Identity). sil_topics.cpp provides
# the topic extern instances normally defined by NVS-dependent params.cpp.
# 状態遷移/failsafe テスト: 無改変のファームソースを simulator/sil/compat シム
# 経由でコンパイル(Code Identity)。sil_topics.cpp が NVS依存 params.cpp の代わりに
# トピック extern 実体を提供する。
SIL = ../../../simulator/sil
STATE_TARGET   = test_state_manager
STATE_CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I$(SIL)/compat $(CXXFLAGS) \
                 -I../components/sf_failsafe/include
STATE_SRCS = test_state_manager.cpp \
             ../components/sf_state/state_manager.cpp \
             ../components/sf_failsafe/failsafe.cpp \
             $(SIL)/sil_topics.cpp

.PHONY: all clean test

all: $(TARGET) $(STATE_TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(STATE_TARGET): $(STATE_SRCS)
	$(CXX) $(STATE_CXXFLAGS) -o $@ $^ -lm

test: all
	-./$(TARGET)
	./$(STATE_TARGET)

clean:
	rm -f $(TARGET) $(STATE_TARGET)
