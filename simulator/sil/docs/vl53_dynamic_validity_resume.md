# VL53 モデル「動的 valid 性」作り込み — 再開ノート

> 自己完結の再開メモ。空コンテキストの新セッションでも、このファイルだけで
> 「エミュレータの VL53L3CX 模型を、飛行中（下向き距離が毎フレーム変化する状況）でも
> 実機同様に status=0 (VALID) を返すよう作り込み、ToF-only の ALTITUDE_HOLD を
> エミュレータ上で安定成立させる」ところまで進められることを目標にする。
> Self-contained resume note: make the synthetic VL53L3CX return VALID status during
> dynamic (changing-distance) flight so ToF-only ALT_HOLD holds in the emulator.

最終更新: 2026-06-02 / 直近コミット: `8fcb57e`（VL53模型: 範囲外で no-target を返す）。
**前提**: 全参照は実コード裏取り済み。経緯と確定結論は auto-memory `project_eskf_vertical_divergence.md` に記録。

---

## 0. ゴール（State / Goal）

- 実機 StampFly は **ToF のみ**（baro 無効）で ALTITUDE_HOLD が動作する（実飛行でパラメータ調整済み）。
  → **旧ファーム（`firmware/vehicle` = emu_vehicle）のパラメータは現実物理に対して妥当**。firmware は触らない。
- だがエミュレータ上では ALT_HOLD が **runaway（暴走上昇）** する。根本原因は **VL53 模型が飛行中に valid を返さない**こと（下記 §2）。
- **ゴール = VL53 模型を作り込み、飛行中も status=0 を返す → ESKF が ToF を融合 → ToF-only ALT_HOLD が安定**。達成後 `hover_alt.scn --video` でレビュー動画。

---

## 1. 確定した事実（裏取り済み・訂正反映）

| 項目 | 結論 | 裏取り |
|------|------|--------|
| **推力物理** | **正しい**。MuJoCo `qacc[2]`=0.28 m/s²（duty0.663, thrTot=0.3733N, mass0.037）でモデル予測と一致。離陸は穏やか(0.28〜0.77 m/s²) | PLANTDBG 直接計測。以前「7m/s²」は emu_trajectory 時刻列の読み違いだった |
| Plant motor 曲線 | firmware と**同一**。Plant Am/Bm/Cm(5.39e-8/6.33e-4/1.53e-2) = firmware 電気モデル(Rm=0.34,Km=6.125e-4,Cq=9.71e-11)の定常簡約。Ct=1e-8 一致 | plant.hpp:68-76, motor_model.cpp:12-21 |
| hover duty 差(0.652 vs 0.698) | 物理ズレではない。1.12 HOVER_THRUST_CORRECTION 分のみ | config.hpp:533 |
| **アームタイミング** | アームは CTRL_FLAG_ARM **立上りエッジ かつ FlightState==IDLE** のみ受理(外れると requestArm 無言false)。boot は app_main 冒頭 `vTaskDelay(3000)`(USB待ち, main.cpp:449)で3s遅延→**IDLE は仮想 t≈9.5s**。`hover_alt.scn` 位相A=13s で対応済 | main.cpp:302,449,963; stampfly_state.cpp:67 |
| ToF 範囲外 | **修正済(8fcb57e)**: dist>MAX_MM(1400) は ambient のみ→no-target(NumberOfObjectsFound=0)を返す(飽和定数を valid と偽らない) | vl53_device.cpp fill_histogram |

**触ってはいけないもの**: firmware 全般（Code Identity）、HOVER_THRUST_CORRECTION 等の制御パラメータ、推力較正（正しい）。**作り込むのは emulator 側の VL53 模型のみ**。

---

## 2. 真の問題：VL53 模型が飛行中 valid を返さない（作り込み対象）

| 状況 | ToF status | 機序 |
|------|-----------|------|
| オフライン `vl53_probe <mm> 6`（**静止**距離） | frame0,1=6 → frame2=4 → **frame3+ = 0 (VALID)** | 6連続フレームを restart 無しで回すと実 gen4 ドライバの wrap-check/位相整合が settle |
| 接地（真高度~10mm） | status=254（無効） | 真距離 10mm < firmware の **MIN_VALID_DISTANCE_MM=30mm**（vl53l3cx_wrapper.cpp:268,304）で棄却。**実機同様で正常**（下向き ToF は接地で近すぎ） |
| **飛行中（距離が毎フレーム変化）** | **status が VALID に昇格しない**（飛行中 tofH=1 が0回） | 実 gen4 ドライバの wrap-check/位相整合が**距離変化で settle しない** → status 6 のまま → firmware が無効判定(254) → ESKF が ToF を融合できない → 鉛直推定が accel-attitude 補正で腐敗 → 閉ループ runaway |

**核心**: 実機の VL53L3CX は動きながらでも valid を返すが、**合成 histogram（vl53_device.cpp）が、実 gen4 ドライバの位相整合チェックを「距離変化中」に満たせていない**。模型の status 昇格は stream_count ベースで擬似再現しているが、ドライバ側の wrap/consistency が距離変化を弾く。

---

## 3. 調査・作業プラン（次セッション）

1. **静止 vs 動的の切り分け（最初に）**: firmware は read 毎に `clearInterruptAndStartMeasurement`（measurement 再開）する。probe は 6連続フレームを restart 無しで回す。この**駆動パターンの差**が status 昇格に影響するか確認する。
   - 案: `Plant::setStartHeight(0.5)` で機体を 0.5m 静止配置し、disarmed のまま firmware に ToF を読ませて status=0 になるか見る（要・機体を 0.5m に保持する仕掛け／または専用 smoke）。
   - 静止0.5mで valid なら → **距離変化（動き）が原因**。静止0.5mでも無効なら → **firmware の restart 駆動パターン**が原因。
2. **実 gen4 ドライバの wrap-check / phase-consistency の要件を把握**（`third_party` の VL53LX ドライバ: `VL53LX_hist_*`、wrap-check、`result__range_status` の昇格条件）。距離変化中でも valid を返させるには、模型がどんな histogram 系列を出せばよいか。
3. **vl53_device.cpp の作り込み**: 連続フレームで位相整合が保たれる histogram 系列を生成する（フレーム間の zero_distance_phase / stream_count / bin 系列の整合を、距離が変わっても維持）。M2 で `zdp=22528`, `strip4`, `stream_count進行`, sub-bin skew(±17mm) を実装済み（vl53_m2_resume.md 参照）— その上に「動的 valid 性」を足す。
4. **回帰**: `vl53_probe 500 6`（静止 status=0 維持）、`vl53_probe 2000 6`（範囲外 no-target 維持）、`hover_espnow.scn`（接地 14checks PASS）。

---

## 4. 検証レシピ（成功判定）

```bash
sf sil build --target emu_vehicle
sf sil scenario simulator/sil/scenarios/hover_alt.scn --duration 37000000
```
- **一時 VDBG**（imu_task.cpp、検証後 revert）で per-cycle に `tofH / tof値 / ESKF pz / vz / ba_z` を出す:
  ```cpp
  // state.updateAccelBias(...) の直後に追加（has_taken_off と eskf_state が in-scope）
  static uint32_t g_vdbg=0;
  if (has_taken_off && (++g_vdbg % 40 == 0))
      ESP_LOGI(TAG,"VDBG tofH=%d tof=%.3f pz=%.3f vz=%.3f baz=%.4f",
               (int)g_tof_task_healthy, g_tof_bottom_buf.count()>0?g_tof_bottom_buf.latest():-1.0f,
               eskf_state.position.z, eskf_state.velocity.z, eskf_state.accel_bias.z);
  ```
- **成功条件**:
  1. 飛行中 **tofH=1**（ToF が valid を返し続ける）。
  2. ESKF alt（`-pz`）が真値(trajectory.csv `pz`/`alt_est`=col4/14)を追従（誤差 数cm）。
  3. ALT_HOLD で alt が平坦（位相D末 `|d(alt)/dt| < 0.05 m/s`）、duty が hover域（暴走無し）。
  4. ba_z が腐敗しない（0 近傍に留まる）。
- 達成後 `--video`（PASS かつ trajectory.csv 非空時のみレンダ。画像確認はサブエージェント限定＝CLAUDE.md）。

---

## 5. 関連

- メモリ: `project_eskf_vertical_divergence.md`（全経緯・訂正済み結論）、`project_stampfly_emulator.md`（全体）、`project_altitude_hold.md`（実機 ToF-only ALT_HOLD 成功の経緯）。
- VL53 M2: `simulator/sil/docs/vl53_m2_resume.md`（合成 histogram 実装の詳細＝今回の土台）。
- 旧ノート: `simulator/sil/docs/hover_resume.md`（§1 の「閉ループが吸収」前提は **ESKF が正しい前提**で、ToF 無効により崩れた。本ノートが上位）。
- 主要ファイル: `simulator/sil/devices/vl53_device.cpp`（合成 histogram・作り込み対象）、`firmware/vehicle/components/sf_hal_vl53l3cx/`（HAL・無改変）、`firmware/vehicle/main/tasks/tof_task.cpp`（jump filter・health・無改変）、`simulator/sil/scenarios/hover_alt.scn`。
- ツール: `simulator/sil/build/vl53_probe <mm> <frames>`（オフライン gen4 probe）。
