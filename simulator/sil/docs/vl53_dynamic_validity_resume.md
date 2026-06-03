# VL53 動的 valid 性 → 空中ホバー 再開ノート

> 自己完結の再開メモ。空コンテキストの新セッションでも、このファイルだけで続行できる。
> Self-contained resume note.

最終更新: 2026-06-02 / 直近コミット: `59bf02b`（VL53 config応答型修正）。
**前提**: 全参照は実コード裏取り済み。経緯は auto-memory `project_eskf_vertical_divergence.md`。

---

## 0. 現状サマリー（State）

| 項目 | 状態 |
|------|------|
| **VL53 動的 valid 性** | **✅ 解決**（commit `59bf02b`）。ToF が ~1.5 m/s の鉛直運動まで status=0 を維持（旧: 0.30 m/s で崖状に全滅）。probe + フル emu で検証済み。 |
| ToF-only ALT_HOLD 空中ホバー | **✅ 成立**（commit `cea0d8c`, 2026-06-03）。§2 の「残ブロッカー」の真因は **SIL Plant 時間基準バグ＝物理が仮想時間の約3倍速**だった（推力でもESKFでもない）。修正（timestep 0.0025→0.00025=4000Hz ＋ Plant::step の固定timestep累積器）で hover_alt peak 900m→0.677m, ESKF ALT capture 0.62m（修正前 -0.00m 発散）, ALT_HOLD 0.62m 保持。**§2 以降の記述は二次症状の観察であり真因ではない — 詳細は `simulator/sil/docs/plant_timebase_bug.md`**。 |

---

## 1. VL53 動的 valid 性 = 解決（commit 59bf02b）

**根本原因**（実測・実コード裏取り）: ドライバは interleaved A/B 測距で、毎フレーム
`rd_timing_status` を反転し（vl53lx_core.c:181）2つの VCSEL period 設定で交互に histogram を
復号する（api_core.c:2677-2689）:
- 設定A: period span 24576（vcsel reg 9）→ zdp=22528, ambient 4bin
- 設定B: period span 49152（vcsel reg 11）→ zdp=30720, ambient 0bin

モデルは**両設定に同一バイト**を送っていたため:
1. 設定B の位相窓 `upper=(valid_phase_high<<8)+zdp=(136<<8)+30720=65536` が gen3.c:847 の
   **uint16 で 0 にオーバーフロー** → 任意の正位相が RANGEPHASECHECK（public OUTOFBOUNDS=4）。
2. 連続フレームは逆設定で **zdp が 8192 食い違う** → phase_consistency_check（core.c:1786,
   許容2048）が毎フレーム 8192 跳躍を検出し PHASECONSISTENCY（public WRAP_TARGET=7）。
   **静止でも失敗**。public VALID は下流の UWR 復元（連続レンジ一致時のみ）頼みで、
   >0.30 m/s で崩壊。

**修正**（vl53_device.cpp, 物理的に正しい＝実機の per-config phasecal を模す）:
- 非wrap の自前フレーム番号で設定A/Bを検出（frame 0,1=A、以後 2,4,6…=B）。
  `result__stream_count`(d[3]) のパリティは uint8 256フレーム wrap で desync するため uint32 frame を保持。
- 設定B に `phasecal_result__reference_phase=40960`（REF_PHASE_B）を与え zdp を 22528 に統一。
  → オーバーフロー解消＋連続位相差が motion 分のみ → gen4 が RANGECOMPLETE(9) を直接返す。
- 設定B は ambient strip=0 に合わせピークを out_bin に配置（設定Aは out_bin+4）。

**検証**: probe 速度掃引 0〜1.5 m/s 全 VALID / 400フレーム長走行 wrap 跨ぎ 398/400 / 静止
100-1300mm ±20mm / 範囲外 no-target / フル emu `bottom≈target status=0` /
回帰 hover_espnow 14・console_cli 8 PASS。

**診断ツール**: `vl53_probe <mm> <frames> [step_mm] [hold]`（step_mm で動的距離、print_internal で
gen4 内部 zdp/位相窓/p_011/status をダンプ）。`cmake -S simulator/sil -B simulator/sil/build
-DSIL_BUILD_VL53_PROBE=ON && cmake --build … --target vl53_probe`。

---

## 1.5 電池/動力モデルの過推力 = 解決（commit 66752df, ユーザー指摘）

**原則（ユーザー）**: ファームは実機で飛んだコード＝正しい。飛ばないならエミュレータの
バグかパラメータ誤り。→ 過推力は **Plant 側の不備**だった。

- ファームの hover FF は `mass·g·HOVER_THRUST_CORRECTION(1.12)`（config.hpp:533, 実飛行
  ログのスロットル実測）を指令。1.12 は実機モータ/プロップが理想曲線より弱い分＋電池サグの補償。
- だが Plant は損失ゼロの理想曲線をそのまま使い、この 1.12 の実機欠損を再現していなかった
  → ファームの hover 指令(duty~0.70)で Plant が 0.40N(hover 0.363N の 1.12倍)を出し、
  net 0.037N の定常過推力（DUTYDBG で直接計測, 一時計装→revert）。
- **修正**: `Plant::Config::thrust_efficiency = 1/1.12 ≈ 0.893` を dutyToThrust に乗ずる
  （hoverDuty も反映）。plant_smoke で Plant の net-0 hover_duty が 0.652→0.698 に上がり
  ファーム指令 duty(~0.70)と一致 → ファームの hover 指令が実 mg を生む（Model Identity）。
- 回帰: hover_espnow 14/14, console_cli 8/8 PASS, plant_smoke 全 PASS。

## 2. 空中ホバーの残ブロッカー：ESKF 離陸ハンドオフ（次の作業対象）

VL53 valid 化＋過推力修正の後も hover_alt は runaway 上昇する。VDBG（imu_task:393 直後,
検証後 revert）で確定した残る故障連鎖:

- **ToF は valid で climb を追従している**（tofH=1, tof=0.29→1.38m）。VL53 修正は効いている。
- **だが ESKF position が追従しない**: 離陸検出時 `resetPositionVelocity()`（imu_task.cpp:268）が
  pz を 0 に戻すが、機体は離陸閾値(tof>0.10m を 200ms 保持=takeoff_hold_samples 80)を満たす
  頃には既に空中(~0.3m+)。直後 ToF は 10サイクル(~0.33s)スキップ(imu_task.cpp:324)。
  スキップ明けに ToF イノベーション(真alt − (−pz=0) ≈ 0.5m+)が TOF_INNOV_GATE(0.5m,
  config.hpp:172)を超え**棄却→pz が 0 に固着→以後 accel-bias 発散(baz 0.07→負)→ESKF alt が
  負へ発散**。ALT_HOLD はこの盲目 ESKF を追って増推(thrust>hover)→ climb → 正帰還 runaway。

**未解明**: STABILIZE 開ループ phase C の climb が η 適用後のモデル予測(raw3310→net~0.13 m/s²)
より速い（trajectory/DUTYDBG とも ~2-3 m/s²）。推力/スロットル経路にもう1つ subtlety がある
可能性（throttle 正規化? duty 飽和?）。要・数値切り分け。

**ユーザー原則に照らした解釈**: ①離陸ハンドオフは firmware の正しい挙動（実機で飛んだ）→
エミュレータ側の何か（ToF/高度タイミング, 残る過推力, シナリオ）が実機と違う。②シナリオは
「STABILIZE で穏やかに離陸→空中で ToF ロック確立→ALT_HOLD 切替」が正しい構造（地上 engage は
NG）。現 hover_alt.scn はこの構造だが phase C の過推力で 1.4m を突破してしまう。

## 2bis. 旧ノートの「真のブロッカー」記述（参考・上記 §2 に統合）

**hover_alt.scn を走らせると機体が runaway上昇（数百〜数千m）し crash-disarm する。** VDBG
（imu_task.cpp に一時挿入し検証後 revert 済み）で確定した故障連鎖:

1. **Plant 過推力**: firmware が hover と思う duty（~0.70, hover FF=0.407N=0.363×1.12
   HOVER_THRUST_CORRECTION）で、Plant は net 上昇推力を出す → 機体が上昇。
2. **離陸後 ToF 10サイクル(~0.3s)スキップ**（imu_task.cpp:324-326, 共分散リセット過補正防止）
   → この盲目窓で機体が 1.4m 超＆5m/s超へ。
3. **firmware jump filter（TOF_MAX_CHANGE=5m/s）** が高速上昇で ToF を凍結（VDBG: tof=1.089m で
   固着, tofH 1→0）。
4. ToF 喪失 → **ESKF 加速度バイアス z が上昇加速度を吸収**（VDBG: baz 0.07→-0.87）→ 鉛直推定が
   真値と乖離（pz≈0 のまま）→ vz の符号も反転 → vel PID が誤って増推 → **正帰還 runaway**。

**核心**: VL53 修正で 0-1.4m 帯の ToF は valid になったが、**firmware 自身のゲート（離陸後スキップ
＋jump filter）＋ Plant 過推力**が、離陸過渡の決定的な ~0.5s で ToF を活かせない。機体は ToF が
再融合される前に 1.4m（ToF レンジ上限）を突破し、以後は鉛直センサ皆無（baro off）で盲目になる。

**未解明・要判断の論点**:
- **Plant 過推力は真か**: メモリ §1 は「thrust 正しい(0.28 m/s² @ duty0.663)」とするが、本試験では
  duty~0.70 で機体が数m/s で上昇。duty 0.663→0.70 の差が過渡を生むのか、Plant の duty→thrust が
  firmware モデルより steep なのか（plant.hpp: V=duty·vbat→ω→T=Ct·ω², 高duty で T∝duty）。
  **数値裏付けの上で**切り分けが要る（feedback_control_simulation）。
- ToF レンジ上限: 修正後 valid 窓は ~3.27m 相当だが、24-bin buffer 制約で設定A は ~1636mm、
  設定B は ~2406mm までしかピークを置けない（MAX_MM=1400 を ~1600 へ上げる余地はあるが 3m は
  phase-wrap 処理=M2 範囲外が要る）。

---

## 3. 次セッションの作業プラン

**目標**: ToF-only ALT_HOLD の安定ホバー → `hover_alt.scn --video` でレビュー動画。

1. **過渡を抑え 1.4m 以内・<5m/s に収める**のが鍵。候補（要・数値検証、firmware は無改変）:
   - 離陸を更に緩やかに（ALT_HOLD setpoint をゆっくりランプ、climb stick を最小に）。だが
     離陸後 ToF スキップ(0.3s)中の Plant 過推力が過渡を作るため、過推力の理解が先。
   - Plant の duty→thrust を PLANTDBG で再計測し、firmware hover duty（~0.63）での net accel を
     直接確認 → 過推力が真なら原因（vbat? Ct? 電圧補償?）を特定。
2. **ToF レンジを ~1600mm へ拡張**（MAX_MM 引き上げ＋設定A の strip 制約確認）で過渡の余裕を増やす。
3. 安定後 `--video`（PASS かつ trajectory.csv 非空時のみ、画像確認はサブエージェント限定=CLAUDE.md）。

**検証レシピ**:
```bash
sf sil scenario simulator/sil/scenarios/hover_alt.scn --duration 38000000
# trajectory.csv: col9=alt(真値), col17-20=motor duty。alt_est(col14)は真値ミラーで firmware ESKF ではない。
# firmware ESKF は console.log の "ALT_HOLD: sp=.. alt=.. vz=.." が正。
```
**VDBG**（imu_task.cpp:393 `state.updateAccelBias` 直後、検証後 revert）:
```cpp
static uint32_t g_vdbg=0;
if (has_taken_off && (++g_vdbg % 20 == 0))
    ESP_LOGI(TAG,"VDBG tofH=%d tof=%.3f pz=%.3f vz=%.3f baz=%.4f",
             (int)g_tof_task_healthy, g_tof_bottom_buf.count()>0?g_tof_bottom_buf.latest():-1.0f,
             eskf_state.position.z, eskf_state.velocity.z, eskf_state.accel_bias.z);
```
**成功条件**: 飛行中 tofH=1 維持、ESKF alt が真値追従（誤差数cm）、ALT_HOLD で alt 平坦、baz 0近傍。

---

## 4. 関連

- メモリ: `project_eskf_vertical_divergence.md`（全経緯）、`project_stampfly_emulator.md`、
  `project_altitude_hold.md`（実機 ToF-only ALT_HOLD 成功）。
- VL53 修正の詳細: commit `59bf02b`、`simulator/sil/docs/vl53_m2_resume.md`（合成 histogram の土台）。
- 主要ファイル: `simulator/sil/devices/vl53_device.cpp`（config応答型）、
  `simulator/sil/smoke/vl53_probe.cpp`（診断）、`firmware/vehicle/main/tasks/imu_task.cpp`（ToF スキップ・
  jump filter・無改変）、`firmware/vehicle/main/altitude_controller.hpp`（ALT_HOLD 制御）、
  `simulator/sil/scenarios/hover_alt.scn`（閉ループ ALT_HOLD 離陸シナリオ）。
