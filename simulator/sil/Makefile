# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
#
# Host SIL harness for vehicle_new — references firmware core sources directly
# (Code Identity). The compat/ shim is injected first via -I so unmodified
# firmware headers compile on a PC.
#
# vehicle_new 用ホストSILハーネス。本体コアを直接参照コンパイルする(Code
# Identity)。compat/ シムを -I 先頭で注入し、無改変のファームヘッダをPCで
# コンパイルさせる。

FW  = ../../firmware/vehicle_new
CXX ?= g++

CXXFLAGS = -std=c++17 -O2 -Wall -Wextra \
  -I compat \
  -I . \
  -I $(FW)/sim \
  -I $(FW)/components/sf_math/include \
  -I $(FW)/components/sf_estimator_eskf/include \
  -I $(FW)/components/sf_controller_pid/include \
  -I $(FW)/components/sf_core/include \
  -I $(FW)/components/sf_state/include \
  -I $(FW)/components/sf_failsafe/include

# Firmware core sources compiled directly (the heart of Code Identity)
# 直接コンパイルする本体コアソース（Code Identity の核心）
CORE_SRCS = \
  $(FW)/components/sf_estimator_eskf/eskf_core.cpp \
  $(FW)/components/sf_state/state_manager.cpp \
  $(FW)/components/sf_failsafe/failsafe.cpp \
  $(FW)/sim/sf_params_sil.cpp \
  sil_topics.cpp

.PHONY: all clean check
all: sil_min_check integrated_sil

# M1 link smoke test: prove the unmodified core links on a PC
# M1 リンク確認: 無改変コアがPCでリンクすることを実証
sil_min_check: sil_min_check.cpp $(CORE_SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

# M3 integrated SIL: physics + ESKF + PID + StateManager + Failsafe
# M3 統合SIL: 物理 + ESKF + PID + StateManager + Failsafe
integrated_sil: integrated_sil.cpp $(CORE_SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

# M11 regression challenge: compile the LEGACY firmware/vehicle ESKF by
# reference (unmodified) via the compat shim. Foundation for reproducing
# known legacy bugs (P-collapse, BA divergence) in the SIL.
# M11 回帰チャレンジ: 旧 firmware/vehicle のESKFを compat シム経由で
# 無改変参照コンパイル。既知の旧バグをSILで再現する基盤。
OLD = ../../firmware/vehicle
OLD_ESKF_CXXFLAGS = -std=c++17 -O2 -Wall -I compat \
  -I $(OLD)/components/sf_algo_eskf/include \
  -I $(OLD)/components/sf_algo_math/include

old_eskf_check: old_eskf_check.cpp $(OLD)/components/sf_algo_eskf/eskf.cpp
	$(CXX) $(OLD_ESKF_CXXFLAGS) -o $@ $^ -lm

# M11-2 regression challenge: drive the LEGACY ESKF (unmodified) with the SAME
# quad_physics + control as integrated_sil, and reproduce known legacy bugs
# (A: P-collapse, C: BA divergence). Links BOTH the vehicle_new physics/control
# headers and the legacy eskf.cpp by reference (Code Identity on both sides).
# M11-2 回帰チャレンジ: 旧ESKF(無改変)を integrated_sil と同じ quad_physics +
# 制御で駆動し、既知の旧バグ(A:P崩壊, C:BA発散)を再現。新側の物理/制御ヘッダと
# 旧 eskf.cpp の両方を参照リンク(両側で Code Identity)。
REG_CXXFLAGS = -std=c++17 -O2 -Wall -Wextra \
  -I compat \
  -I $(FW)/components/sf_math/include \
  -I $(FW)/components/sf_controller_pid/include \
  -I $(FW)/sim \
  -I $(OLD)/components/sf_algo_eskf/include \
  -I $(OLD)/components/sf_algo_math/include

old_eskf_regression: old_eskf_regression.cpp $(OLD)/components/sf_algo_eskf/eskf.cpp
	$(CXX) $(REG_CXXFLAGS) -o $@ $^ -lm

check: sil_min_check
	./sil_min_check

clean:
	rm -f sil_min_check integrated_sil old_eskf_check old_eskf_regression
