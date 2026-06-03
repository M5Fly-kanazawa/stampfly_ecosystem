# P5 センサノイズ N0 — ✅完了ノート＋P6 への引き継ぎ

> 自己完結の到達点メモ。空コンテキストの新セッションでも、このファイルだけで状況を把握できる。
> P5 (sensor noise N0) is DONE; this note now hands off to P6.

最終更新: 2026-06-03（P5 達成）。**前提**: 全参照は実コード裏取り済み。ロードマップは
`simulator/sil/RESET_PLAN.md` §10/§13。

---

## 0. P5 ✅完了（What was done）

**合格基準（RESET_PLAN §10）= G2（推定誤差有界）＋決定論＋統計テスト＋レビュー動画 — 全て達成。**

- **配線**: ノイズモデルは実装済みだったが emu が既定 off で `Plant::init` を呼んでいた。両 emu 入口
  （`emu/emu_main_generic.cpp`=emu_vehicle, `emu/emu_main.cpp`=emu_vehicle_new）に
  `SIL_EMU_NOISE=n0`/`SIL_EMU_SEED` env を配線し、`sf sil scenario --noise n0 --seed N` から制御可能に。
  既定 off は従来と **byte-identical**（クリーン経路不変・回帰確認済み）。
- **白色σの substep 非依存化**: 時間基準修正で物理が 4kHz substep ＝ `advance()` が 4kHz 呼びになり、
  白色を substep（0.25ms）で離散化すると σ が √10 倍に膨張する欠陥を是正。`SensorNoise::Config::white_dt`
  （=ファーム 400Hz 読み取り周期）で白色を離散化するよう分離。`smoke/noise_test.cpp` に
  substep 分離テストを追加し数値確認済（4kHz substep でもファームが見る σ は 400Hz 相当）。
- **検証**: G2＝`hover_alt`(16s) 7シード全て有界（保持窓 std 1.3–3.9cm）、`hover_long`(90s) で alt std~2.7cm・
  緩い有界ドリフト・姿勢チルト平均~4°(max10°)で発散なし。決定論＝同シードで traj/stdout/stderr 完全一致。
  レビュー動画＝`viz/out_scn_hover_alt/scn_hover_alt.mp4`（`render_video.py` 単一 run カメラを px/py 毎フレーム
  追従に修正、床 3m→200m 拡大＝物理不変）。`sensor_noise.hpp` の `@design` を `[--]→[OK]`、RESET_PLAN P5 を ✅。

**知見（→P6 動機）**: ALTITUDE_HOLD は水平位置を保持しないため、N0 残留 accel バイアスが ESKF 姿勢に~4°
チルトを生み機体が水平に数十m ドリフトする（高度・姿勢は有界＝G2満たす）。`hover_long` 着地での 12m 上昇は
接地衝撃検出→crash DISARM 後の MuJoCo 剛体接触の数値爆発（モータ disarm 済み）で P5 欠陥ではない。

---

## 0b. 前提として完了済み（背景）

- **P0〜P4 ✅**（更地化／物理SIL骨格／アルゴリズム非依存／CLI・ダッシュボード／レビュー動画）。
- **エミュレータ E0〜E6 ✅**（実 app_main を14タスクでホスト実行・Plant 閉ループ・決定論シナリオ注入・
  VL53/INA3221/BMP280 モデル）。
- **時間基準バグ修正済**（commit `cea0d8c`, 物理4000Hz・仮想時間1:1ロック）。詳細 `plant_timebase_bug.md`。

---

## 1. ノイズモデルは実装済み・適用済み（作り直し不要）

| 要素 | 状態 |
|------|------|
| ノイズモデル `simulator/sil/plant/sensor_noise.hpp` | **完成**。N0 = 白色ガウス（gyro 0.000122 rad/s/√Hz, accel 0.00157 m/s²/√Hz）＋起動時バイアス1σ（gyro 0.005, accel 0.02 = **起動校正「後」の残留**）＋バイアスRW（gyro 1e-4, accel 1e-3 /√s）。シード付き決定論。 |
| 単体テスト `simulator/sil/smoke/noise_test.cpp` | あり |
| Plant が適用 `plant.cpp` | `noise_.init(cfg_.noise)`(L62)、`imu()` で `applyAccel/applyGyro`(L277-278)、`substep` で `noise_.advance(h)`(L230)。**時間基準修正後は substep(0.25ms)ごとに advance** |
| 仕様書 | `firmware/vehicle_new/docs/noise_and_vibration_model.md` §2-3 |

---

## 2. 実施済みの配線（参照） — 変更ファイル

| 変更 | ファイル | 内容 |
|------|---------|------|
| emu に noise env | `emu/emu_main_generic.cpp`, `emu/emu_main.cpp` | `plant_config_from_env()` を追加。`SIL_EMU_NOISE=n0`/`SIL_EMU_SEED` を読み `g_plant.init(path, cfg)`。env 未設定/off は `Config{}`＝従来と byte-identical |
| CLI | `lib/sfcli/commands/sil.py` | `sf sil scenario` に `--noise`(off/n0)・`--seed` を追加。subprocess env に `SIL_EMU_NOISE/SEED` を渡し、results.json に記録 |
| 白色σ分離 | `plant/sensor_noise.hpp` | `Config::white_dt`(=0.0025=400Hz) を追加し `drawWhite()` を引数なし化。白色を物理 substep でなくファーム読み取りレートで離散化（√10倍膨張を是正）。RW は substep の √dt 累積のまま正しい |
| 単体テスト | `smoke/noise_test.cpp` | `test_white_substep_decoupled` を追加（4kHz substep + 400Hz 読みで σ が 400Hz 相当に留まることを数値確認） |
| 動画カメラ/床 | `viz/render_video.py`, `models/stampfly.xml` | 単一 run カメラを px/py 毎フレーム追従。床プレーン 3m→200m（texrepeat 比例、物理不変） |

> 旧 `sf sil run`(=`hover_smoke`) も同じ白色σ膨張欠陥を持っていたが、`sensor_noise.hpp` 修正で同時に是正
> 済み。ただし hover_smoke は核ループ部分試験ゆえ P5 の正路ではない（忠実な検証は emu_vehicle 上）。

---

## 3. 検証レシピ（P5 達成の再現手順・回帰確認用）

```bash
# (1) ビルド
source setup_env.sh; cmake --build simulator/sil/build --target emu_vehicle noise_test
./simulator/sil/build/noise_test                       # モデルの単体テスト

# (2) ノイズONでホバー（hover_long で長時間の有界性を見る）
sf sil scenario simulator/sil/scenarios/hover_long.scn --duration 116000000 --noise n0 --seed 12345
#   合格(G2): ホバー中 ESKF alt が真値の数cm以内で有界、発散/runaway 無し、disarm/crash 無し。
#   姿勢が乱れすぎない（sensor_noise.hpp の警告: 大きな accel バイアスは ESKF 姿勢を発散させる。
#   N0 の残留 accel バイアス1σ=0.02 は小さく設計。傾き 0.02→5°/0.05→13° の感度に注意）。

# (3) 決定論: 同シードで2回 → byte-identical
sf sil scenario .../hover_long.scn --noise n0 --seed 42 > /tmp/a.log 2>&1
sf sil scenario .../hover_long.scn --noise n0 --seed 42 > /tmp/b.log 2>&1
diff <trajectory or console>  # 完全一致なら決定論OK

# (4) 統計テスト(§13): ノイズONとOFFで複数シード→推定誤差の分布が有界・期待σ内。
#   ESKF vs 相補(--target/estimator)の比較は P6 で意味を持つ(N0は両者大差ない見込み)。

# (5) P5 レビュー動画(§9 必須): sf sil scenario .../hover_alt.scn --noise n0 --video
#   単一 run カメラは px/py 毎フレーム追従に修正済（位置保持なし＋ノイズで機体が数十m流れるため、
#   固定 lookat だと画面外）。床は 200m に拡大（追従しても床が見える）。
#   ※ 画像確認はサブエージェント限定(CLAUDE.md)。
```

**合格判定（P5 達成済み・2026-06-03）**: ① noise n0 でホバーが有界（G2）✅ ② 同シード決定論 ✅
③ 統計テスト（7シード有界）✅ ④ レビュー動画 ✅。`sensor_noise.hpp` `@design [--]→[OK]`、RESET_PLAN P5 ✅。

---

## 4. 注意・論点

- **ESKF accel-bias 頑健性**（sensor_noise.hpp:50-58 の知見）: 未校正の大きな accel バイアス(≥0.1 m/s²)は
  ESKF 姿勢ループを発散させる（傾き 0.02→5°, 0.05→13°, 0.10→47°）。相補フィルタは有界。N0 は残留
  バイアスを小さく(0.02)模擬。**P6 で起動校正(水平静止 ba_z≈2g)を再現**し全オフセットを捕えるのが動機。
- **firmware 無改変の原則**: ノイズは Plant（センサ側）に載せる。firmware は触らない。
- **決定論を壊さない**: env 未設定なら off＝現行と byte-identical（既存 N0 経路の不変条件）。
- 旧 `sf sil run`(hover_smoke) は欠陥ゆえ P5 の正路ではない。**emu_vehicle 上で行う**。

## 5. 関連
- メモリ: `project_sil_architecture`(P5はN0)、`project_stampfly_emulator`、`project_eskf_vertical_divergence`
  (時間基準修正・ホバー成立)、`project_estimator_attitude_comparison`(相補 vs ESKF 姿勢の宿題=P6で追求)。
- 文書: `RESET_PLAN.md` §10(ロードマップ)/§13(P5-P10 検証能力)、`plant_timebase_bug.md`、
  `firmware/vehicle_new/docs/noise_and_vibration_model.md`。
- 主要ファイル: `plant/sensor_noise.hpp`(モデル)、`plant/plant.cpp`(適用)、`emu/emu_main_generic.cpp`(配線先)、
  `lib/sfcli/commands/sil.py`(CLI配線先)、`smoke/noise_test.cpp`(単体テスト)、
  `scenarios/hover_long.scn`/`hover_alt.scn`(試験飛行)。

---

## 6. 次の目標 = P6（センサノイズ N1/N2 ＋ ToF/Baro/Flow ＋ ESKF vs 相補）

RESET_PLAN §10/§13 P6。**ゴール**: スロットル依存・帯域制限ノイズ（σ_axis=K[axis]·duty²、f_low/f_high）
と ToF/Baro/Flow の観測ノイズ（R 表）を加え、**ノイズ下で ESKF が相補フィルタより優位なことを定量化**
（P4 比較動画をノイズ版に更新）＋ G4（飽和率）。

- **P5 が炙り出した宿題（P6 で追う）**: N0 残留 accel バイアスで ESKF 姿勢が~4°チルト→水平ドリフト。
  **起動校正（水平静止 ba_z≈2g, `noise_and_vibration_model.md` §3）を再現**して全オフセットを捕え、
  チルト/ドリフトを抑える。`project_estimator_attitude_comparison`（相補の方が姿勢安定）の原因究明もここ。
- **着手前に確認**: ノイズモデル（`sensor_noise.hpp`）は N0 のみ。N1/N2 は密度の duty 依存項・帯域制限
  （1次/2次フィルタ）を追加実装する必要がある。ToF/Baro/Flow ノイズは Plant の各センサ合成（`plant.cpp`
  の `tof()/baro()/flow()`）に R を載せる。**firmware 無改変**の原則を守る（ノイズは Plant 側）。
- **比較経路は既存**: `sf sil compare`（hover_smoke ベース）。ただし忠実版は emu_vehicle 上で
  ESKF/相補を切り替える経路（`--target`/estimator 選択）が要る。P6 着手時に emu の estimator 切替を要確認。
