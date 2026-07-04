# vehicle_new 機能ステータス — 計画・追加・未移植の整理

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

vehicle_new は、旧 vehicle の設計思想を改め、**コードの守備範囲を明確にしてスパゲッティ化を防ぎ、保守・発展・更新を容易にする**ことを狙った、機能的には vehicle 互換でさらに発展させたファームウェアである。本書はその開発状況を3つの観点で整理する:

1. **計画当初（設計文書）にあって完了したもの**（§2）
2. **計画当初にあって未完了のもの**（§3）
3. **後から追加されたもの** — 実機検証・設計議論が駆動した設計外の追加・仕様変更（§4）
4. **vehicle にあって vehicle_new にないもの** — 未移植・意図的廃止・等価置換の区別（§5）

### 対象読者

vehicle_new の開発状況を把握したい開発者・教材利用者。次に何を作るべきか／何が意図的に無いのかを判断する材料とする。

### 判定基準と更新

- 「計画当初」= 6設計文書（requirements / architecture / detailed_design / coding_and_education / development_roadmap / hardware_init）に記載があるもの
- 「完了」= 実装済みかつ SIL 回帰または実機ベンチで検証済み
- 最終更新: 2026-06-11。機能の出入りがあったら本書を更新すること

## 2. 計画当初にあって完了したもの

### アーキテクチャ基盤

| 機能 | 内容 | 検証 |
|------|------|------|
| 4階層アクセス＋横断ルール R1〜R16 | コンポーネント責務分離、@design タグ、直接呼び出し禁止 | 全コードレビュー済（2026-06-10） |
| Pub-Sub トピック通信 | Latest / Queue / RingBuffer の3種、約30トピック | SIL＋実機 |
| BSP（sf_board） | バス所有（I2C/SPI/LEDC/RMT/netif）、Critical/Optional/Recoverable 起動分類 | 実機 |
| パラメータシステム | params.cpp = SSOT、NVS 永続化（FNV-1aキー）、ReloadParams によるライブ反映 | 実機 |
| タスク構成 | 16タスク（IMU/Control 400Hz、State、センサ群、Comm、Telemetry、Log、CLI、Notify ほか）、コア分担 | 実機（負荷 watermark 毎分ログ） |

### センサ・推定・制御

| 機能 | 内容 | 検証 |
|------|------|------|
| センサ HAL 一式 | BMI270（SPI 1600Hz・OSR4）/ PMW3901 / BMM150（25Hz・DRDYゲート）/ BMP280 / VL53L3CX（底面）/ INA3221 | 実機（全センサ設計レート達成を Data Stream で確認） |
| 15状態 ESKF | χ²ゲート・active_mask による P 行列隔離・疎構造化（predict 720積和） | SIL G2 ゲート＋実機 |
| 推定器差し替え | IEstimator、`estimator_type` パラメータで ESKF / 相補フィルタ切替 | SIL 比較検証（P6） |
| 鉛直系 | ToF-only 鉛直＋接地アンカー＋離陸エッジの鉛直ハンドオフ | SIL alt_rmse 1.6cm＋実機 |
| 制御器差し替え | IController、カスケード PID（ACRO/STABILIZE/ALT_HOLD/POS_HOLD） | SIL 16シナリオ |
| ミキサー | B⁻¹ 制御配分（物理単位 Nm/N）＋モータ曲線＋ライブ電池電圧補償 | SIL＋実機 |
| 起動校正 | ジャイロ/加速度バイアス測定→推定器種付け、完了まで ARM ゲート | 実機（§4 の静止ゲートで強化） |

### 状態機械・安全

| 機能 | 内容 | 検証 |
|------|------|------|
| フライト状態機械 | INIT→IDLE_GROUND↔IDLE_HELD→ARMED_GROUND→TAKEOFF→FLYING→LANDING、StateManager 単一所有 | SIL＋実機 |
| フェイルセーフ | 通信断／電池 EMERGENCY → 自動着陸（Landing verb、−0.3 m/s）、衝撃・ジャイロ異常 → 即時 DISARM | SIL commloss/crash_refly |
| 墜落→再飛行 readiness | ESKF リセット＋再校正＋モード再伝播の全鎖 | SIL crash_refly 21/21 |
| ペアリング | 相互 MAC 学習・混信フィルタ・NVS 復元・チャネル追従（1教室30機前提） | SIL pairing＋実機 |

### 通信・ログ・UI

| 機能 | 内容 | 検証 |
|------|------|------|
| SSOT プロトコル準拠 | ControlPacket 14B（protocol/spec 準拠、コントローラ無改変） | 実機 |
| Telemetry | UDP 50Hz 状態モニタ。受信は `sf telemetry`（ターミナル、`--web` でブラウザ。UDP→SSE プロキシ — requirements §7 の WebSocket は stdlib 等価の SSE で実現） | 実機＋ループバック E2E |
| Data Stream | 制御周期 400Hz の解析ログ。**旧 vehicle 電文完全互換**（UDP 8890 / 0x50 統合パケット）で `sf log wifi` → `sf log viz` が無改造で動く | 実機 E2E（全センサ設計レート、欠損ゼロ） |
| Blackbox | SPIFFS バイナリ記録、ARM→DISARM で1セッション | 実装済（SPIFFS 無しはグレースフル無効） |
| CLI（USB＋TCP） | esp_console レジストリ（R6）。USB-CDC REPL＋**TCP ポート23**（電池駆動ベンチ用） | 実機（モータテストで使用） |
| WiFi STA/AP 両対応 | `wifi.mode` パラメータ＋CLI `wifi`、ESP-NOW チャネル共存 | 実機 |
| LED/ブザー UI | 状態別 LED・イベント音・mute/輝度の NVS 設定 | 実機（§4 の2チャネル化で発展） |

### 開発基盤（SIL）

| 機能 | 内容 | 検証 |
|------|------|------|
| StampFly エミュレータ | 実 app_main・全タスク・実ドライバを**無改変**でホスト実行（Code Identity） | 16シナリオ回帰 |
| シナリオ DSL＋expect ゲート | rc/wind/fault/bias/handle 注入、G1〜G4 機械判定 | TEST_MATRIX.md |
| 3原則 | Code / Param / Model Identity（ロードマップ §2） | params.cpp 共有、ミキサー/モータ曲線が SIL プラントと厳密逆 |

## 3. 計画当初にあって未完了のもの

| 機能 | 計画箇所 | 状態 | 備考 |
|------|---------|------|------|
| Tello API / UDP コマンド受信 | requirements §7（TelloAPI: Yes） | **コア実装済**（2026-06-11, ApiTask）: command/takeoff/land/emergency/stop/移動/回頭/クエリ＋Python SDK（tools/stampfly_py）。SIL api_flight で全鎖検証 | 残: Tello 互換の拡張コマンド（flip/curve 等）、実機飛行検証 |
| Data Stream の USB 経路 | requirements §7（UDP/USB 選択） | UDP のみ | WiFi 不要環境向けの変種 |
| 校正の NVS 永続化 | calibration.cpp に保存系あり | **意図的保留** | NVS commit のフラッシュ消去が 400Hz ループを >10ms 停止させる。CONFIG_SPI_FLASH_AUTO_SUSPEND 調査とセットで再開 |
| 前方 ToF | hardware_init（XSHUT 配線済み） | HAL あり・未ブリングアップ | 障害物検知用途 |
| POS_HOLD 実機検証 | roadmap Phase 4 | SIL のみ PASS | 地面付近のフロー品質が実機の未知数 |
| 磁気ヨー融合の常用化 | ESKF に観測枠あり | `eskf.use_mag` 既定 off | 校正機能は §4 で整備済み。実機ログで磁気健全性を見て判断 |

## 4. 後から追加されたもの（実機・設計議論が駆動）

設計文書に無かったが、実機ベンチ・飛行・設計レビューが必要性を炙り出した追加。**いずれも設計文書に反映済み**（要件・詳細設計の該当箇所を更新）。

| 追加 | きっかけ | commit（2026-06） |
|------|---------|------------------|
| **静止ゲート付き起動校正**（動き検出で蓄積破棄・やり直し＋窓内分散チェック） | 実機で「起動直後/墜落後に突然反転」— 運搬中の動きが校正を汚染していた（旧 vehicle の Phase 2 安定ゲートの移植漏れ） | 8cc1932 |
| **旧実績 PID ゲイン移植＋D-on-M 化** | 制御則の新旧比較で微分対象の差（誤差微分→測定値微分）と出力リミット差を発見・整合。実績ゲイン 1:1 移植 | 3985cf0 |
| **ジャイロバイアス偏差クランプ**（ノミナル±0.03 rad/s） | 設計議論:「無関係なセンサの異常がクロス共分散経由でレートループ用バイアスを汚す」→ PX4 流の被害有界化 | 942a1c9 |
| **地上モード変更＋ALT/POS 自動離陸**（仕様変更） | 「設置時のモード変更が最も安全なのにできないのは不自然」。制御器に鉛直フェーズ（Grounded=推力ゼロ/TakeoffClimb/Airborne）導入 | 4220de1 |
| **2チャネル LED UI**（本体=モード色、StampS3=システム状態） | 「3つの LED の使い分けができていない」— disarm 中のモード視認の要望 | b5bba36 |
| **地磁気校正の配線**（magcal＋起動時磁気参照捕捉＋未校正は融合から自動除外） | 「校正機能がない」— 移植済みだが未配線の MagCalibrator を発見し配線 | 9598dd7 |
| **INIT 停止根治**（-O2 固定＋ESKF 疎構造化＋StateTask コア0） | 実機でコア1飽和→StateTask 飢餓。SIL では CPU 飽和が見えない教訓 | b4f4800 |
| ARM の press-toggle 化 | 実機でボタン押下中のみ ARM される誤実装が発覚（コントローラの ARM はモーメンタリ） | — |
| 設計保留事項の確定（A1〜A6） | レビューで設計保留 → ユーザー決定: 自動着陸 verb / WiFi 両対応 / 相補フィルタ ToF / ReloadParams / 衝撃検出 400Hz | 5コミット |
| 運用安全の小品 | `param save` の armed 拒否（フラッシュ停止対策）／起動音の2秒遅延（esptool 窓回避）／ControlTask 通知ウォッチドッグ／`param reset` | — |
| Data Stream 品質3点 | 差分量（flow）は全サンプル配送・DRDY はデータと同一バースト読み・容量上限棄却の計数可視化 — いずれも実機ベンチが摘出 | 7bd69db ほか |

## 5. vehicle にあって vehicle_new にないもの

### 未移植（将来の移植候補）

| 旧機能 | 内容 | 優先度メモ |
|--------|------|-----------|
| CLI コマンド群（約50） | 自律飛行（`takeoff`/`land`/`hover`/`jump`）、Tello 風相対移動（`up/down/forward/.../cw/ccw`）、クエリ（`battery?` 等）、`trim`/`gain` | Tello API（§3）と同根。教材価値が高い |
| flight_command サービス | 上記コマンドの実行系（自律コマンドキュー） | 同上 |
| USB バイナリログ取得 | 旧 `sf log capture`（USB 経由） | Data Stream USB 変種（§3）として実現予定 |
| レベル基準の活用 | 旧 level_calibrator 相当。新は level_offset を計算するが**未使用**（ログのみ） | 傾いた机での校正精度に関わる |

### 意図的に廃止（設計判断）

| 旧機能 | 廃止理由 |
|--------|---------|
| WebSocket テレメトリ | UDP に統一（requirements §7 に明記。ブロッキングリスク排除）。ブラウザ表示はホスト側プロキシで対応する方針 |
| 12Hz ジャイロノッチフィルタ | 旧 vehicle の実験で「無効が最良」と確定済み（振動はフィードバック励振であり、ノッチは位相を悪化させた） |
| 気圧の鉛直融合 | ToF-only 鉛直をユーザー方針として決定（ESKF 鉛直発散の解決時）。気圧センサ自体は読んでおり Data Stream に流れる |
| グローバル変数群（globals.cpp） | Pub-Sub トピックへ全面置換（スパゲッティ化の主因だった） |
| sf_svc_* の旧サービス構造 | 新アーキテクチャ（4階層＋R1〜R16）で再設計。機能は等価実装で置換（下表） |

### 等価機能で置換済み

| 旧 | 新 | 備考 |
|----|----|------|
| sf_svc_wifi_cli | TCP CLI（ポート23、esp_console 共有） | 同一コマンドが USB/TCP 両方で動く |
| StationaryDetector | StillnessConfig（校正の静止ゲート） | 閾値は旧実績値を踏襲（生バイアス向けに拡幅） |
| mag_calibration（旧 magcal） | `magcal` CLI＋MagTask 所有（R5 準拠） | アルゴリズムは旧コードをそのまま移植 |
| HOVER_THRUST_CORRECTION | hover_thrust = mg×1.12 | 旧の飛行実測補正を継承 |
| altitude/position_controller | PidController 内カスケード | 実績ゲイン（PI-v1）移植済み |
| LEDManager 優先度 | Notify 優先度オーバーレイ＋2チャネル | 低電圧>ペアリング>校正>状態 の優先順は踏襲 |

---

<a id="english"></a>

## 1. Overview

### About This Document

vehicle_new is a redesign of the legacy vehicle firmware aimed at **clear component responsibilities, no spaghetti, and easy maintenance/evolution** — functionally compatible with vehicle and extended beyond it. This document organizes the development status from three angles: originally-planned features that are done (§2), originally-planned features not yet done (§3), features added later driven by hardware testing and design discussions (§4), and legacy-vehicle features absent from vehicle_new (§5), distinguishing not-yet-ported / deliberately-dropped / replaced-by-equivalent.

### Criteria

- "Originally planned" = written in the six design documents (requirements / architecture / detailed_design / coding_and_education / development_roadmap / hardware_init)
- "Done" = implemented AND verified by the SIL regression or on hardware
- Last updated 2026-06-11; update this document when features move.

## 2. Planned and Completed

| Area | Highlights | Verified |
|------|-----------|----------|
| Architecture | 4-layer access + rules R1–R16, Pub-Sub topics (~30), BSP (sf_board) with Critical/Optional/Recoverable boot classes, params.cpp SSOT + NVS + live reload, 16 tasks (IMU/Control 400 Hz) | Full code review + hardware |
| Sensors | BMI270 (SPI 1600 Hz, OSR4) / PMW3901 / BMM150 (25 Hz, DRDY-gated) / BMP280 / VL53L3CX / INA3221 — all at design rates (confirmed via Data Stream) | Hardware E2E |
| Estimation | 15-state ESKF (chi2 gates, active_mask P isolation, sparse predict), swappable IEstimator (ESKF/complementary), ToF-only vertical + ground anchoring + takeoff handoff | SIL G2 + hardware |
| Control | Swappable IController, cascade PID (ACRO/STAB/ALT/POS), B^-1 mixer in physical units + motor curve + live battery compensation | SIL 16 scenarios |
| State machine & safety | Full flight state machine, comm-loss/battery auto-landing (Landing verb, −0.3 m/s), impact DISARM, crash→re-fly readiness chain, pairing (anti-cross-talk, 30 craft/classroom) | SIL + hardware |
| Comms & logging | SSOT ControlPacket 14B, Telemetry UDP 50 Hz (receiver: `sf telemetry`, terminal or `--web` browser via UDP→SSE), **Data Stream 400 Hz wire-compatible with legacy** (`sf log wifi`/`viz` unmodified), Blackbox on SPIFFS, CLI over USB + TCP:23, WiFi STA/AP dual | Hardware E2E |
| SIL | Real app_main/tasks/drivers run UNMODIFIED on the host (Code Identity), scenario DSL + expect gates G1–G4, Code/Param/Model Identity principles | 16/16 regression |

## 3. Planned, Not Yet Done

| Feature | Status |
|---------|--------|
| Tello API / UDP command receive | CORE DONE (ApiTask: command/takeoff/land/emergency/stop/moves/rotate/queries + Python SDK, SIL-verified end-to-end); extended Tello verbs + hardware flights remain |
| Data Stream over USB | UDP only today |
| Calibration NVS persistence | Deliberately deferred — NVS flash erase stalls the 400 Hz loop >10 ms; revisit with CONFIG_SPI_FLASH_AUTO_SUSPEND |
| Front ToF | HAL present, not brought up |
| POS_HOLD hardware validation | SIL-only PASS; near-ground flow quality is the unknown |
| Mag yaw fusion as default | `eskf.use_mag` off; calibration tooling now exists (§4) — decide after flight-log review |

## 4. Added Later (driven by hardware & design discussions)

All retrofitted into the design documents.

| Addition | Trigger | Commit (2026-06) |
|----------|---------|------------------|
| Stillness-gated boot calibration (+ window variance check) | Hardware flips right after boot/crash — carry motion polluted the bias average (legacy Phase-2 stability gate had not been ported) | 8cc1932 |
| Legacy flight-proven PID gains + D-on-M | Control-law comparison found derivative-input and limit differences; gains ported 1:1 | 3985cf0 |
| Gyro-bias deviation clamp (nominal ± 0.03 rad/s) | Design discussion: any sensor anomaly reaches the rate-loop bias through cross-covariances — PX4-style bounded damage | 942a1c9 |
| Ground mode change + ALT/POS auto-takeoff (spec change) | "Changing modes while parked is the safest time"; controller vertical phases (Grounded = zero thrust / TakeoffClimb / Airborne) | 4220de1 |
| Dual LED channels (body = mode colour, StampS3 = system state) | Bench UX feedback | b5bba36 |
| Magnetometer calibration wiring (magcal + boot mag-ref capture + uncalibrated keep-out) | "No mag calibration" — found the ported-but-unwired MagCalibrator | 9598dd7 |
| INIT-stuck root fix (-O2 + sparse ESKF + StateTask on core 0) | Core-1 saturation starved StateTask on hardware; SIL cannot see CPU saturation | b4f4800 |
| ARM press-toggle, A1–A6 decisions, param-save armed refusal, deferred boot chime, ControlTask watchdog, `param reset`, Data Stream delivery fixes (flow full-rate, DRDY-in-burst, capacity-drop accounting) | Various hardware findings | — |

## 5. In vehicle but Not in vehicle_new

### Not yet ported (candidates)

| Legacy feature | Note |
|----------------|------|
| ~50 CLI commands (autonomous `takeoff`/`land`/`hover`/`jump`, Tello-style moves, queries, `trim`/`gain`) | Same root as the Tello API item (§3); high educational value |
| flight_command service (autonomous command queue) | Ditto |
| USB binary log capture (`sf log capture`) | To be realized as the USB Data Stream variant |
| Level-reference usage | level_offset is computed but unused (log only) |

### Deliberately dropped

| Legacy feature | Reason |
|----------------|--------|
| WebSocket telemetry | Unified on UDP (stated in requirements §7; blocking risk); browser view via a host-side proxy |
| 12 Hz gyro notch filter | Legacy experiments concluded "off is best" (the oscillation was feedback excitation) |
| Baro vertical fusion | ToF-only vertical chosen by policy; baro is still read and streamed |
| Global variables (globals.cpp) | Fully replaced by Pub-Sub topics — the main spaghetti source |
| Legacy sf_svc_* service structure | Re-architected (4 layers + R1–R16); functions replaced by equivalents below |

### Replaced by equivalents

| Legacy | vehicle_new |
|--------|-------------|
| sf_svc_wifi_cli | TCP CLI (port 23, shared esp_console registry) |
| StationaryDetector | StillnessConfig (calibration stillness gate, raw-bias-widened thresholds) |
| mag_calibration | `magcal` CLI + MagTask ownership (R5), same algorithm |
| HOVER_THRUST_CORRECTION | hover_thrust = mg × 1.12 (flight-measured, inherited) |
| altitude/position_controller | Cascades inside PidController (proven PI-v1 gains ported) |
| LEDManager priority | Notify priority overlay + dual channels (same precedence) |
