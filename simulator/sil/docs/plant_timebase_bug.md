# SIL Plant 時間基準バグ — 物理が仮想時間の約3倍速で進行

> 自己完結の分析ノート。空コンテキストの新セッションでも、このファイルだけで状況を把握できる。
> Self-contained root-cause note.

最終更新: 2026-06-03 / 状態: **✅ 修正済・検証済**（物理4000Hz化＋固定timestep累積器）。
**前提**: 全数値は実コード裏取り・実測（emu_vehicle_old / hover_alt.scn）。

---

## 1. 結論（一文）

**SIL のフル emu（emu_vehicle / emu_vehicle_old）は、MuJoCo 物理をスケジューラ仮想時間の
約3倍の速さで進めている。** これは推力やセンサの問題ではなく、Plant ステップの**時間基準
（タイムベース）バグ**。過去に「過推力による climb 1.75 m/s²」「ESKF 鉛直発散」「ToF 凍結」と
記録した現象は、いずれもこのバグの二次症状である可能性が高い。

---

## 2. 根本原因（実コード）

| 場所 | 振る舞い |
|------|---------|
| `plant.cpp` `Plant::step(dt)` | `mj_step(m_, d_)` を呼ぶ。**MuJoCo の `mj_step` は引数 `dt` を無視し、モデル固定 timestep `m_->opt.timestep`（= `stampfly.xml` の `option timestep="0.0025"` = 2.5 ms）を1回だけ進める。** `dt` はモータ一次遅れ `alpha` とノイズ前進にしか使われない。 |
| `emu_main_generic.cpp` `on_advance(now_us)` / `emu_main.cpp` | スケジューラの**クロック前進イベント毎**に `sil_board_step_plant(now_us-last)` を1回呼ぶ＝ Plant を 0.0025s 1回進める。 |
| `scheduler.cpp` `Scheduler::run` | 決定論的協調スケジューラ。Ready タスクが無いとき仮想時計 `now_us_` を「次の起床/タイマ発火」へ**不規則にジャンプ**させ、その都度 `on_advance_` を呼ぶ。 |

**不整合の核心**: 物理は「`on_advance` 呼び出し回数 × 0.0025s」しか進まない。一方スケジューラ
仮想時間は実際の起床間隔で進む。**14個のタスクが各々の周期（1ms, 2.5ms, …）で起床するため、
2.5ms あたりの distinct なクロックイベント数は 400Hz の約3倍**。よって:

```
物理時間  d_->time  =  (on_advance 呼び出し回数) × 0.0025 s
                    ≈  3 × now_us（スケジューラ仮想時間）
```

`mj_step` が `dt` を尊重する、という設計者の暗黙の前提が誤り（mj_step は dt を取らない）。

---

## 3. 実測の裏付け（emu_vehicle_old / hover_alt.scn）

**STEPDBG**（`on_advance` に一時計装→revert 済）: 物理時間 / スケジューラ時間 の比

| on_advance 呼び出し | 物理時間 | スケジューラ時間 | 比（累積） |
|--------------------|---------|----------------|-----------|
| 4000  | 10.0 s | 7.355 s | 1.360 |
| 8000  | 20.0 s | 10.796 s | 1.853 |
| 12000 | 30.0 s | 14.129 s | 2.123 |
| 20000 | 50.0 s | 20.796 s | 2.404 |
| 40000 | 100.0 s | 37.462 s | 2.669 |

**増分比**（飛行フェーズ・全タスク稼働時）: 4000コール（物理10s）ごとにスケジューラ時間は
約 3.33s しか進まない → **増分比 = 10/3.33 ≈ 3.0**。起動直後は稼働タスクが少なく比が小さい
（1.36）→ 飛行中に約3.0へ収束。**比が時間で変動する＝物理速度の歪みは非一様**。

**「過推力 climb 1.75 m/s²」との一致**:
- duty 0.706（STABILIZE phase C, 実測）での Plant 真の鉛直加速度 = **0.196 m/s²**
  （plant_smoke で hover_duty 0.6976→mg 一致を確認、手計算でも duty 0.706 → net 0.0073N → 0.196 m/s²）。
- 物理が3倍速 → 見かけの climb 加速度 ∝ 真値 × （時間倍率）² = 0.196 × 3² = **1.76 m/s²**。
- trajectory.csv の実測 climb（線形速度ランプ）= **1.75 m/s²**。**完全一致**。

**潔白が確認された経路**（このバグとは無関係に正しい）:
- スロットル正規化: `normalizeThrottle = clamp((raw-2048)/2048, 0, 1)`（control_arbiter.cpp:275）。raw3310→0.616。
- STABILIZE 推力指令: `total_thrust = throttle × MAX_TOTAL_THRUST(0.672)` = 0.414N（control_task.cpp:821, 実測一致）。
- firmware thrust→duty（motor_model.hpp）と Plant duty→thrust（plant.cpp）は電気モデルの定常簡約として
  **正確に逆関数**（Am=Rm·Cq/Km=5.39e-8 等を検算済）。efficiency 1/1.12 分だけ Plant 出力が小さい。
- duty 配送: レガシー firmware の**本物の** motor_driver.cpp が ledc_set_duty→`g_motor_duty`→Plant.setDuty
  （8bit, max_duty=255 でラウンドトリップ, trajectory m0=0.706 で確認）。**duty は正しく 0.706 が Plant に届く**。

---

## 4. 影響範囲（過去の SIL 結論への波及）

| 対象 | 影響 |
|------|------|
| **フル emu の飛行ダイナミクス全般** | emu_vehicle / emu_vehicle_old とも物理が3倍速。climb 率・速度・ToF 変化率・ESKF 挙動の**定量結論はすべて要再評価**。 |
| **ESKF 鉛直発散 / 離陸ハンドオフ lock-out**（memory `project_eskf_vertical_divergence` §2） | **二次症状の可能性大**。物理3倍速 → firmware は dt=2.5ms で予測するが ToF は3倍動く位置を返す → イノベーション過大 → accel-bias が辻褄合わせで発散。jump filter（5 m/s）も真 1.7 m/s 上昇が見かけ 5 m/s 超で誤発火。**firmware の構造的欠陥ではなく、時間歪みが firmware に不可能なセンサ整合性を強いている**疑い。 |
| **Plant 推力効率 1/1.12（commit 66752df）** | **修正自体は独立に正当**（plant_smoke は固定 dt 直接駆動でバグ無し、hover_duty 0.698 が firmware ALT_HOLD hover duty と一致＝Model Identity）。ただし当時の動機「過推力 climb」は時間基準バグの誤帰属だった。 |
| **固定 dt で直接 step する smoke 群** | plant_smoke / hover_smoke / rate_tune / physics_smoke / noise_test / frames_test は `step(0.0025)` をループで呼ぶ＝物理時間=ループ時間。**影響なし**。 |
| **決定性** | バグは決定論的（スケジューラが決定論）＝再現可能。だが物理レートが誤り。過去に「非決定」と見えたのは d_->time と now_us の混同＋サンプル時刻違いの錯覚。 |

---

## 5. 実施した修正（✅ 完了）

**2点を同時に直した（制御検証の標準＝物理は制御の10倍以上で積分する）:**

1. **物理を 4000Hz に細かく**（`models/stampfly.xml`）: `option timestep` を `0.0025`→`0.00025`
   （400Hz 制御の10倍）。制御サンプル間のプラント挙動を積分する（物理刻み=制御周期だと制御の
   間の物理が無い）。RK4 はそのまま、刻みだけ細かく。
2. **固定 timestep 累積器**（`plant.cpp` `Plant::step` / `Plant::substep`）: `step(dt)` は経過仮想
   時間 `dt` を `step_accum_` に累積し、`m_->opt.timestep`（=h）刻みで**収まる整数回だけ** `substep(h)`
   を回し端数を繰り越す。`substep` が従来の step 本体（モータ遅れ→推力→反トルク+風→mj_step→ノイズ、
   ただし dt の代わりに h を使う）。→ `d_->time` が `now_us` と 1:1 同期、かつプラントは 4000Hz で積分。
   - 実装位置は `Plant::step` 自身。全 step 呼び出し元（emu 両系統＋固定dt smoke群）が一貫して
     「dt ぶん進む」よう正される。固定 dt=0.0025 で呼ぶ smoke は substep 10回＝同じ総時間・10倍精細。

**検証結果（実測, 2026-06-03）**:
- **runaway 解消**: hover_alt peak alt **900m → 0.677m**（穏やかな離陸）。
- **離陸ハンドオフ修正**: ESKF の `ALT capture: alt=0.62m`（修正前は **-0.00m** で発散）。
- **climb 真値化**: phase C(t=14→16) の加速度 ≈ **0.22 m/s²**（予測 0.196 と一致, 修正前 1.75）。
- **ALT_HOLD 保持**: sp=0.62m, alt が 0.62m に整定, vz≈0, thrust 0.406N≈hover 0.407N。
- **時間 1:1**: trajectory が t=38s で終了（修正前は物理 t=100s まで暴走）。
- **決定論的**: 2回連続実行で peak alt=0.677m 一致。
- **回帰なし**: plant_smoke 全PASS / hover_espnow 14/14 / console_cli 8/8（**sf 既定 25s で**。
  注意: 仮想 pilot の arm は ~20s 以降ゆえ duration 20s だと arm 前に終わる＝偽 FAIL になる）。
- **ESKF 鉛直発散 blocker = 連鎖解消**を確認（§4 の予測どおり、物理が実レートに戻り ToF/ESKF が
  整合した）。`project_eskf_vertical_divergence` の「離陸ハンドオフ」blocker はこの時間基準バグの
  二次症状だったと確定。

---

## 6. 検証レシピ（再確認用）

```bash
sf sil scenario simulator/sil/scenarios/hover_alt.scn --duration 38000000
# console.out: peak alt ~0.68 m（runaway しない）
# console.log: "ALT capture: alt=0.62m"、ALT_HOLD で alt が 0.62m 平坦、vz≈0
# 回帰: sf sil scenario .../hover_espnow.scn （既定25s）= 14/14、console_cli.scn = 8/8
./simulator/sil/build/plant_smoke   # 全PASS（hover 維持）
```

---

## 7. 関連

- メモリ: `project_eskf_vertical_divergence`（離陸ハンドオフ blocker=本バグの二次症状疑い）、
  `project_stampfly_emulator`、`feedback_control_simulation`（数値検証必須）。
- 主要ファイル: `simulator/sil/plant/plant.cpp`（`step` の mj_step 固定刻み）、
  `simulator/sil/devices/virtual_board.cpp`（`sil_board_step_plant`）、
  `simulator/sil/emu/emu_main_generic.cpp` / `emu_main.cpp`（`on_advance`）、
  `simulator/sil/rtos/scheduler.cpp`（仮想時計の不規則ジャンプ）、
  `simulator/sil/models/stampfly.xml`（`option timestep="0.0025"`）。
- 直前の作業: `simulator/sil/docs/vl53_dynamic_validity_resume.md`（VL53 動的 valid 性は解決済、
  本バグはその後の「空中ホバー blocker」の真因）。

---

<a id="english"></a>

## 1. Conclusion (one line)

**The full SIL emulator (emu_vehicle / emu_vehicle_old) advances the MuJoCo physics at ~3× the
scheduler's virtual-time rate.** This is a Plant-stepping **time-base bug**, not a thrust or sensor
issue. The previously logged "over-thrust climb 1.75 m/s²", "ESKF vertical divergence" and "ToF
freeze" are very likely secondary symptoms of this single bug.

## 2. Root cause

`Plant::step(dt)` calls `mj_step`, which **ignores `dt` and advances the FIXED model timestep
(`m_->opt.timestep` = 0.0025 s) exactly once**. `on_advance(now_us)` calls it once per scheduler
**clock-advance event**. The cooperative scheduler jumps its virtual clock to the next wake/timer in
irregular steps; with 14 firmware tasks at various periods there are ~3× as many distinct clock
events per 2.5 ms as a single 400 Hz tick. Hence physics time = (#calls)×0.0025 ≈ 3×now_us.

## 3. Evidence

STEPDBG steady-state physics/scheduler ratio ≈ 3.0. True thrust accel at duty 0.706 = 0.196 m/s²
(plant_smoke + hand calc); ×3² = 1.76 m/s² = the observed 1.75 m/s² climb. The throttle/thrust/duty
path is verified correct, so the over-climb is purely the time-base distortion.

## 4. Impact

All full-emu flight-dynamics conclusions need re-evaluation. The ESKF divergence / takeoff handoff
lock-out is probably a direct consequence (physics 3× fast → ToF moves 3× → innovation gate /
jump filter / accel-bias all break). The thrust-efficiency fix (66752df) is independently valid
(plant_smoke is unaffected) but was motivated by the misattributed over-climb. Direct fixed-dt smoke
tests are unaffected.

## 5. Fix

Fixed-timestep accumulator in `sil_board_step_plant` (shared by both emus): accumulate elapsed
virtual time, run `mj_step` floor(accum/h) times, carry the remainder, so `d_->time` tracks `now_us`
1:1. Expected to resolve the entire takeoff-handoff blocker chain — to be confirmed by measurement.
