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

SRCS = sil_main.cpp \
       ../components/sf_estimator_eskf/eskf_core.cpp

TARGET = sil_stampfly

.PHONY: all clean run csv plot

all: $(TARGET)

$(TARGET): $(SRCS)
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

clean:
	rm -f $(TARGET) flight_log.csv
