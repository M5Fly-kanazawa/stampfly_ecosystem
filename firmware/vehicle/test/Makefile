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
    -I../components/sf_failsafe/include \
    -I../components/sf_command/include \
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

# Non-finite number detection (nonfinite_guard.hpp): the mask, the per-boundary monitor (first boundary and time,
# checked / detected cycles) and the motor-output guard (finite duties unchanged, NaN / infinity driven as 0).
# Header-only logic — own binary, no extra sources.
# 有限でない数の検出（nonfinite_guard.hpp）: マスク・境界ごとの監視器（最初の境界と時刻・検査した周期と検出した
# 周期）・モータ出力の保護（有限な duty は不変、NaN・無限大は 0）。ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_NONFINITE_GUARD = test_nonfinite_guard.cpp
TARGET_NONFINITE_GUARD = test_nonfinite_guard

# Low-battery alert timing (sf::BatteryAlertPolicy, battery_alert_policy.hpp): the EMERGENCY on entry into the
# critical region, its re-raise cadence, re-entry after a recovery to the warning band, the bound on a chattering
# voltage, the WARNING latch. Header-only logic — own binary, no extra sources.
# 低電圧アラートのタイミング（sf::BatteryAlertPolicy、battery_alert_policy.hpp）: 危険領域への突入時の EMERGENCY・
# 再発報の周期・警告帯へ回復した後の再突入・ばたつく電圧の頻度の上限・WARNING のラッチ。
# ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_BATTERY_ALERT_POLICY = test_battery_alert_policy.cpp
TARGET_BATTERY_ALERT_POLICY = test_battery_alert_policy

# API argument checks (sf::api_arguments: finite numbers, `cw` / `ccw` range, `sysid` range) and the angle wrap
# sf::math::wrapAngleRad that terminates for any finite input. Header-only logic — own binary, no extra sources.
# API の引数検査（sf::api_arguments: 有限の数・`cw` / `ccw` の範囲・`sysid` の範囲）と、有限な入力ならどれでも
# 終わる角度の折り返し sf::math::wrapAngleRad。ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_API_ARGUMENTS = test_api_arguments.cpp
TARGET_API_ARGUMENTS = test_api_arguments

# `cw` / `ccw` guidance rules (sf::rotate_guidance: target lead, unwrapped heading, completion, timeout).
# Header-only logic — own binary, no extra sources.
# `cw` / `ccw` の誘導規則（sf::rotate_guidance: 目標の先行量・アンラップした向き・完了判定・待ち時間）。
# ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_ROTATE_GUIDANCE = test_rotate_guidance.cpp
TARGET_ROTATE_GUIDANCE = test_rotate_guidance

# Range check of a float parameter (sf::params::isFloatInRange: finite and inside the range; a NaN is refused).
# Header-only logic — own binary, no extra sources.
# float パラメータの範囲検査（sf::params::isFloatInRange: 有限かつ範囲内。NaN は拒否）。
# ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_PARAM_RANGE = test_param_range.cpp
TARGET_PARAM_RANGE = test_param_range

# StateManager against the normative table (detailed_design.md §3.1). state_manager.cpp is compiled unmodified; the topics
# (sf_core/params.cpp) and the clock, log and NVS shims come from simulator/sils/compat, the same host shims the SILS uses.
# It does NOT use -I. (the local esp_log.h / esp_timer.h stubs): the SILS shims provide the virtual clock and the log hook.
# 規範表（detailed_design.md §3.1）に対する StateManager。state_manager.cpp は無改変でコンパイルし、トピック
# （sf_core/params.cpp）と時計・ログ・NVS の代用は SILS と同じホスト用シム simulator/sils/compat から取る。
# -I.（ローカルの esp_log.h / esp_timer.h の代用）は使わない: 仮想時計とログのフックは SILS のシムが提供する。
SILS_COMPAT = ../../../simulator/sils/compat
CXXFLAGS_STATE_MANAGER = -std=c++17 -D_USE_MATH_DEFINES -Wall -Wextra -O2 \
    -I$(SILS_COMPAT) -I../../../simulator/sils/esp_idf_host \
    -I../components/sf_math/include \
    -I../components/sf_core/include \
    -I../components/sf_state/include
SRCS_STATE_MANAGER = test_state_manager.cpp \
       ../components/sf_state/state_manager.cpp ../components/sf_core/params.cpp \
       $(SILS_COMPAT)/clock_shim.cpp $(SILS_COMPAT)/log_shim.cpp $(SILS_COMPAT)/nvs_shim.cpp
TARGET_STATE_MANAGER = test_state_manager

# Log sink (sf_core/log_sink.cpp): the two exits of one ESP_LOG entry (console ring, WiFi ring), no copy and no loss report while
# no capture is active, a capture starting from "now", exact loss counts carried at the place of the loss, an over-long line cut
# and flagged. log_sink.cpp is compiled unmodified; the clock and ESP_LOGx come from simulator/sils/compat (same shims as the
# StateManager test, hence no -I. here).
# ログシンク（sf_core/log_sink.cpp）: 1 つの ESP_LOG の入口の 2 つの出口（コンソール用・WiFi 用リング）、取得が無い間はコピーも
# 欠落報告もしない、取得は「今」から始まる、欠落の正確な数が欠落の位置で運ばれる、長すぎる行は切ってフラグを付ける。
# log_sink.cpp は無改変でコンパイルし、時計と ESP_LOGx は simulator/sils/compat から取る（StateManager の試験と同じシムなので -I. は使わない）。
SRCS_LOG_SINK = test_log_sink.cpp ../components/sf_core/log_sink.cpp \
       $(SILS_COMPAT)/clock_shim.cpp $(SILS_COMPAT)/log_shim.cpp
TARGET_LOG_SINK = test_log_sink

# Floor-relative barometric height (sf::BarometricHeightEstimator): the floor zero, the offset learned from the ToF, the hold while
# the ToF is silent, the repeated 50 Hz samples of a 13 Hz sensor. Header-only logic — own binary, no extra sources.
# 床基準の気圧高度（sf::BarometricHeightEstimator）: 床のゼロ、ToF から学習するオフセット、ToF の出力停止中の保持、
# 13 Hz のセンサの 50 Hz の繰り返しサンプル。ヘッダのみのロジック — 独自バイナリで追加ソース不要。
SRCS_BAROMETRIC_HEIGHT = test_barometric_height.cpp
TARGET_BAROMETRIC_HEIGHT = test_barometric_height

.PHONY: all clean test

all: $(TARGET) $(TARGET_ESKF_FLIP) $(TARGET_FLIP_SEQUENCER) $(TARGET_TILT_CONSISTENCY) $(TARGET_SENSOR_FRESHNESS) $(TARGET_TOF_SILENCE) $(TARGET_TOF_HEALTH) $(TARGET_BLIND_LANDING) $(TARGET_TAKEOFF_POLICY) $(TARGET_LOG_RING) $(TARGET_LOG_SINK) $(TARGET_LOOP_TIMING) $(TARGET_NONFINITE_GUARD) $(TARGET_BATTERY_ALERT_POLICY) $(TARGET_API_ARGUMENTS) $(TARGET_ROTATE_GUIDANCE) $(TARGET_PARAM_RANGE) $(TARGET_STATE_MANAGER) $(TARGET_BAROMETRIC_HEIGHT)

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

$(TARGET_NONFINITE_GUARD): $(SRCS_NONFINITE_GUARD)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_BATTERY_ALERT_POLICY): $(SRCS_BATTERY_ALERT_POLICY)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_API_ARGUMENTS): $(SRCS_API_ARGUMENTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_ROTATE_GUIDANCE): $(SRCS_ROTATE_GUIDANCE)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_PARAM_RANGE): $(SRCS_PARAM_RANGE)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_STATE_MANAGER): $(SRCS_STATE_MANAGER)
	$(CXX) $(CXXFLAGS_STATE_MANAGER) -o $@ $^ -lm

$(TARGET_BAROMETRIC_HEIGHT): $(SRCS_BAROMETRIC_HEIGHT)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(TARGET_LOG_SINK): $(SRCS_LOG_SINK)
	$(CXX) $(CXXFLAGS_STATE_MANAGER) -o $@ $^ -lm

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
	./$(TARGET_LOG_SINK)
	./$(TARGET_LOOP_TIMING)
	./$(TARGET_NONFINITE_GUARD)
	./$(TARGET_BATTERY_ALERT_POLICY)
	./$(TARGET_API_ARGUMENTS)
	./$(TARGET_ROTATE_GUIDANCE)
	./$(TARGET_PARAM_RANGE)
	./$(TARGET_STATE_MANAGER)
	./$(TARGET_BAROMETRIC_HEIGHT)

clean:
	rm -f $(TARGET) $(TARGET_ESKF_FLIP) $(TARGET_FLIP_SEQUENCER) $(TARGET_TILT_CONSISTENCY) $(TARGET_SENSOR_FRESHNESS) $(TARGET_TOF_SILENCE) $(TARGET_TOF_HEALTH) $(TARGET_BLIND_LANDING) $(TARGET_TAKEOFF_POLICY) $(TARGET_LOG_RING) $(TARGET_LOG_SINK) $(TARGET_LOOP_TIMING) $(TARGET_NONFINITE_GUARD) $(TARGET_BATTERY_ALERT_POLICY) $(TARGET_API_ARGUMENTS) $(TARGET_ROTATE_GUIDANCE) $(TARGET_PARAM_RANGE) $(TARGET_STATE_MANAGER) $(TARGET_BAROMETRIC_HEIGHT)
