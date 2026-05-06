# StampFly SIL Simulation — runs on host PC
# StampFly SILシミュレーション — ホストPC上で実行

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 \
    -I. \
    -I../test \
    -I../components/sf_math/include \
    -I../components/sf_estimator_eskf/include \
    -I../components/sf_controller_pid/include \
    -I../components/sf_core/include \
    -I../components/sf_estimator/include \
    -I../components/sf_controller/include \
    -I../components/sf_state/include

# Sources shared by every SIL program
# 全SILプログラムで共有するソース
COMMON_SRCS = \
    ../components/sf_estimator_eskf/eskf_core.cpp \
    sf_params_sil.cpp

# Default target: legacy demo binary
# デフォルト: 旧来のデモバイナリ
TARGET = sil_stampfly

# Test programs share the common objects, each with its own *.cpp
# テストプログラムは共通オブジェクトを共有、それぞれ独自の *.cpp を持つ
TEST_PROGS = \
    flight_scenario_test \
    control_test \
    sine_response_test \
    eskf_step_hold_test \
    eskf_step_recovery_test \
    eskf_sine_then_hover_test \
    test_drop \
    tune_attitude \
    tune_extended \
    tune_final \
    tune_lpf \
    tune_sweep \
    test_params_sil

.PHONY: all tests clean run csv plot

all: $(TARGET)

# Legacy demo
# 旧来のデモ
$(TARGET): sil_main.cpp $(COMMON_SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

# Build all test programs
# 全テストプログラムをビルド
tests: $(TEST_PROGS)

# Generic test program rule: <name> built from <name>.cpp + COMMON_SRCS
# 汎用テストプログラムルール: <name> は <name>.cpp + COMMON_SRCS からビルド
$(TEST_PROGS): %: %.cpp $(COMMON_SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

run: $(TARGET)
	./$(TARGET)

# Save CSV output for plotting / プロット用CSV出力を保存
csv: $(TARGET)
	./$(TARGET) > flight_log.csv
	@echo "Saved to flight_log.csv"

# Plot with Python (requires matplotlib) / Pythonでプロット（matplotlib必要）
plot: csv
	python3 plot_flight.py

# clean removes only non-tracked build artifacts. test_drop and flight_log.csv
# are currently tracked (predates the simulator-binary .gitignore policy);
# leave them alone here so `make clean` does not produce unstaged deletions.
# clean は git 追跡されていないビルド生成物のみを削除する。
# test_drop と flight_log.csv は現在追跡中（バイナリ .gitignore 化より前にコミット済み）
# のため、make clean で勝手に削除しない。
NON_TRACKED_PROGS = $(filter-out test_drop, $(TEST_PROGS))

clean:
	rm -f $(TARGET) $(NON_TRACKED_PROGS)
