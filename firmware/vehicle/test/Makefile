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

# Tilt consistency monitor (detailed_design §3 注10). Pure logic — own binary, same
# pattern as above; links only tilt_consistency.cpp (sf_math is header-only).
# 傾き整合モニタ（detailed_design §3 注10）。純粋ロジック — 上と同じ独自バイナリの形で、
# tilt_consistency.cpp のみリンクする（sf_math はヘッダのみ）。
SRCS_TILT_CONSISTENCY = test_tilt_consistency.cpp ../components/sf_estimator/tilt_consistency.cpp
TARGET_TILT_CONSISTENCY = test_tilt_consistency

# Silence / freshness watch (sf::SilenceWatch: signed age of a new sample + saturating silent
# time; sensor_health, comm loss, long-silence cases beyond 35.8 / 71.6 min).
# Header-only logic — own binary, no extra sources.
# 出力停止・鮮度の監視器（sf::SilenceWatch: 新サンプルの符号付き経過 + 飽和する停止時間。
# sensor_health・通信断・35.8 / 71.6 分を超える長時間の出力停止）。
# ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_SENSOR_FRESHNESS = test_sensor_freshness.cpp
TARGET_SENSOR_FRESHNESS = test_sensor_freshness

# Downward-ToF silence: the SilenceWatch judgement (threshold boundary, reversal, uint32 wrap,
# silence > 36 min, never-seen, out-of-range is not silent) and the state machine's response
# for every state (tof_silence_policy.hpp). Header-only logic — own binary, no extra sources.
# 下向き ToF の出力停止: SilenceWatch の判定（しきい値境界・逆転・uint32 巻き戻り・36 分超・未到達・
# 範囲外は出力停止でない）と、全状態に対する状態機械の応答（tof_silence_policy.hpp）。
# ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_TOF_SILENCE = test_tof_silence.cpp
TARGET_TOF_SILENCE = test_tof_silence

# ToF re-acquisition judgement (sf::ToFHealthTracker, judgeToFSilenceReason, FailsafeStatusGuard,
# decideToFModeRefusal): "the estimator has re-acquired the ToF", stuck-invalid while flying, the stale
# failsafe_status guard, and the in-flight mode-request consumption. Header-only logic — own binary.
# ToF 取り込み直しの判定（sf::ToFHealthTracker・judgeToFSilenceReason・FailsafeStatusGuard・
# decideToFModeRefusal）: 「推定器が ToF を取り込み直した」、飛行中の無効の継続、古い failsafe_status の
# ガード、飛行中のモード要求の消費。ヘッダのみのロジック — 独自バイナリ。
SRCS_TOF_HEALTH = test_tof_health.cpp
TARGET_TOF_HEALTH = test_tof_health

# Blind landing thrust profile (blind_landing.hpp): time function, floor, late ramp, start thrust
# calibrated by the accelerometer, upward specific force. Header-only logic — own binary.
# ToF なしの着陸の推力プロファイル（blind_landing.hpp）: 時間関数・床・後半のランプ・加速度計で較正した
# 開始推力・上向きの加速度計の測定値。ヘッダのみのロジック — 独自バイナリ。
SRCS_BLIND_LANDING = test_blind_landing.cpp
TARGET_BLIND_LANDING = test_blind_landing

# TAKEOFF decisions (takeoff_policy.hpp): manual / auto completion, the auto-takeoff abort on the positive floor evidence of the healthy ToF (TakeoffFloorWatch),
# and the comm-loss landing from TAKEOFF / FLYING, every cell of the normative table. Header-only logic — own binary.
# TAKEOFF の判断（takeoff_policy.hpp）: 手動・自動の完了、健全な ToF の積極的な床の証拠（TakeoffFloorWatch）による自動離陸の中止、TAKEOFF・FLYING からの通信断着陸。
# 規範表の全セル。ヘッダのみのロジック — 独自バイナリ。
SRCS_TAKEOFF_POLICY = test_takeoff_policy.cpp
TARGET_TAKEOFF_POLICY = test_takeoff_policy

# Log sink line buffer (sf::LogRing, log_ring.hpp): FIFO order, wrap-around, overflow counting, the
# longest line. Header-only logic — own binary, no extra sources.
# ログシンクの行バッファ（sf::LogRing、log_ring.hpp）: FIFO の順序・一周・溢れの計数・最長の行。
# ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_LOG_RING = test_log_ring.cpp
TARGET_LOG_RING = test_log_ring

# 400 Hz loop timing monitor (sf::LoopTimingMonitor, loop_timing.hpp): late-wake threshold (1.5 periods),
# busy budget, armed split, uint32 wrap. Header-only logic — own binary, no extra sources.
# 400 Hz ループ周期の監視器（sf::LoopTimingMonitor、loop_timing.hpp）: 遅れの境界（1.5 周期）・処理時間の
# 予算・ARM 中の別計上・uint32 の桁あふれ。ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_LOOP_TIMING = test_loop_timing.cpp
TARGET_LOOP_TIMING = test_loop_timing

# NOTE: test_state_manager is temporarily disabled. It compiled the firmware
# state_manager/failsafe through the now-removed simulator/sils/compat shim and
# sils_topics.cpp (deleted in the SILS clean-slate, 2026-05-31). It will be
# re-wired onto the new SILS host shim; test_state_manager.cpp is kept for that.
# 注: test_state_manager は一時無効化。削除済み simulator/sils/compat シム +
# sils_topics.cpp 経由でビルドしていたため(SILS更地化)。新SILSのホストシム上で
# 再配線予定。test_state_manager.cpp は再配線のため残置。

.PHONY: all clean test

all: $(TARGET) $(TARGET_ESKF_FLIP) $(TARGET_FLIP_SEQUENCER) $(TARGET_TILT_CONSISTENCY) $(TARGET_SENSOR_FRESHNESS) $(TARGET_TOF_SILENCE) $(TARGET_TOF_HEALTH) $(TARGET_BLIND_LANDING) $(TARGET_TAKEOFF_POLICY) $(TARGET_LOG_RING) $(TARGET_LOOP_TIMING)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_ESKF_FLIP): $(SRCS_ESKF_FLIP)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_FLIP_SEQUENCER): $(SRCS_FLIP_SEQUENCER)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_TILT_CONSISTENCY): $(SRCS_TILT_CONSISTENCY)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_SENSOR_FRESHNESS): $(SRCS_SENSOR_FRESHNESS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_TOF_SILENCE): $(SRCS_TOF_SILENCE)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_TOF_HEALTH): $(SRCS_TOF_HEALTH)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_BLIND_LANDING): $(SRCS_BLIND_LANDING)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_TAKEOFF_POLICY): $(SRCS_TAKEOFF_POLICY)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_LOG_RING): $(SRCS_LOG_RING)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_LOOP_TIMING): $(SRCS_LOOP_TIMING)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

test: all
	./$(TARGET)
	./$(TARGET_ESKF_FLIP)
	./$(TARGET_FLIP_SEQUENCER)
	./$(TARGET_TILT_CONSISTENCY)
	./$(TARGET_SENSOR_FRESHNESS)
	./$(TARGET_TOF_SILENCE)
	./$(TARGET_TOF_HEALTH)
	./$(TARGET_BLIND_LANDING)
	./$(TARGET_TAKEOFF_POLICY)
	./$(TARGET_LOG_RING)
	./$(TARGET_LOOP_TIMING)

clean:
	rm -f $(TARGET) $(TARGET_ESKF_FLIP) $(TARGET_FLIP_SEQUENCER) $(TARGET_TILT_CONSISTENCY) $(TARGET_SENSOR_FRESHNESS) $(TARGET_TOF_SILENCE) $(TARGET_TOF_HEALTH) $(TARGET_BLIND_LANDING) $(TARGET_TAKEOFF_POLICY) $(TARGET_LOG_RING) $(TARGET_LOOP_TIMING) test_state_manager
