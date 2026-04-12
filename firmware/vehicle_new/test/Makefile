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

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
