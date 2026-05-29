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
all: sil_min_check

# M1 link smoke test: prove the unmodified core links on a PC
# M1 リンク確認: 無改変コアがPCでリンクすることを実証
sil_min_check: sil_min_check.cpp $(CORE_SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

check: sil_min_check
	./sil_min_check

clean:
	rm -f sil_min_check integrated_sil
