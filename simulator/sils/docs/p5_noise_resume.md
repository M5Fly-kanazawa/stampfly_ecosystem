# P5 センサノイズ N0 — ✅完了ノート＋P6 への引き継ぎ

> 自己完結の到達点メモ。空コンテキストの新セッションでも、このファイルだけで状況を把握できる。
> P5 (sensor noise N0) is DONE; this note now hands off to P6.

最終更新: 2026-06-03（P5 達成）。**前提**: 全参照は実コード裏取り済み。ロードマップは
`simulator/sils/RESET_PLAN.md` §10/§13。

---

## 0. P5 ✅完了（What was done）

**合格基準（RESET_PLAN §10）= G2（推定誤差有界）＋決定論＋統計テスト＋レビュー動画 — 全て達成。**

- **配線**: ノイズモデルは実装済みだったが emu が既定 off で `Plant::init` を呼んでいた。両 emu 入口
  （`emu/emu_main_generic.cpp`=emu_vehicle_old, `emu/emu_main.cpp`=emu_vehicle）に
  `SILS_EMU_NOISE=n0`/`SILS_EMU_SEED` env を配線し、`sf sils scenario --noise n0 --seed N` から制御可能に。
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

- **P0〜P4 ✅**（更地化／物理SILS骨格／アルゴリズム非依存／CLI・ダッシュボード／レビュー動画）。
- **エミュレータ E0〜E6 ✅**（実 app_main を14タスクでホスト実行・Plant 閉ループ・決定論シナリオ注入・
  VL53/INA3221/BMP280 モデル）。
- **時間基準バグ修正済**（commit `cea0d8c`, 物理4000Hz・仮想時間1:1ロック）。詳細 `plant_timebase_bug.md`。

---

## 1. ノイズモデルは実装済み・適用済み（作り直し不要）

| 要素 | 状態 |
|------|------|
| ノイズモデル `simulator/sils/plant/sensor_noise.hpp` | **完成**。N0 = 白色ガウス（gyro 0.000122 rad/s/√Hz, accel 0.00157 m/s²/√Hz）＋起動時バイアス1σ（gyro 0.005, accel 0.02 = **起動校正「後」の残留**）＋バイアスRW（gyro 1e-4, accel 1e-3 /√s）。シード付き決定論。 |
| 単体テスト `simulator/sils/smoke/noise_test.cpp` | あり |
| Plant が適用 `plant.cpp` | `noise_.init(cfg_.noise)`(L62)、`imu()` で `applyAccel/applyGyro`(L277-278)、`substep` で `noise_.advance(h)`(L230)。**時間基準修正後は substep(0.25ms)ごとに advance** |
| 仕様書 | `firmware/vehicle/docs/noise_and_vibration_model.md` §2-3 |

---

## 2. 実施済みの配線（参照） — 変更ファイル

| 変更 | ファイル | 内容 |
|------|---------|------|
| emu に noise env | `emu/emu_main_generic.cpp`, `emu/emu_main.cpp` | `plant_config_from_env()` を追加。`SILS_EMU_NOISE=n0`/`SILS_EMU_SEED` を読み `g_plant.init(path, cfg)`。env 未設定/off は `Config{}`＝従来と byte-identical |
| CLI | `lib/sfcli/commands/sils.py` | `sf sils scenario` に `--noise`(off/n0)・`--seed` を追加。subprocess env に `SILS_EMU_NOISE/SEED` を渡し、results.json に記録 |
| 白色σ分離 | `plant/sensor_noise.hpp` | `Config::white_dt`(=0.0025=400Hz) を追加し `drawWhite()` を引数なし化。白色を物理 substep でなくファーム読み取りレートで離散化（√10倍膨張を是正）。RW は substep の √dt 累積のまま正しい |
| 単体テスト | `smoke/noise_test.cpp` | `test_white_substep_decoupled` を追加（4kHz substep + 400Hz 読みで σ が 400Hz 相当に留まることを数値確認） |
| 動画カメラ/床 | `viz/render_video.py`, `models/stampfly.xml` | 単一 run カメラを px/py 毎フレーム追従。床プレーン 3m→200m（texrepeat 比例、物理不変） |

> 旧 `sf sils run`(=`hover_smoke`) も同じ白色σ膨張欠陥を持っていたが、`sensor_noise.hpp` 修正で同時に是正
> 済み。ただし hover_smoke は核ループ部分試験ゆえ P5 の正路ではない（忠実な検証は emu_vehicle 上）。

---

## 3. 検証レシピ（P5 達成の再現手順・回帰確認用）

```bash
# (1) ビルド
source setup_env.sh; cmake --build simulator/sils/build --target emu_vehicle noise_test
./simulator/sils/build/noise_test                       # モデルの単体テスト

# (2) ノイズONでホバー（hover_long で長時間の有界性を見る）
sf sils scenario simulator/sils/scenarios/hover_long.scn --duration 116000000 --noise n0 --seed 12345
#   合格(G2): ホバー中 ESKF alt が真値の数cm以内で有界、発散/runaway 無し、disarm/crash 無し。
#   姿勢が乱れすぎない（sensor_noise.hpp の警告: 大きな accel バイアスは ESKF 姿勢を発散させる。
#   N0 の残留 accel バイアス1σ=0.02 は小さく設計。傾き 0.02→5°/0.05→13° の感度に注意）。

# (3) 決定論: 同シードで2回 → byte-identical
sf sils scenario .../hover_long.scn --noise n0 --seed 42 > /tmp/a.log 2>&1
sf sils scenario .../hover_long.scn --noise n0 --seed 42 > /tmp/b.log 2>&1
diff <trajectory or console>  # 完全一致なら決定論OK

# (4) 統計テスト(§13): ノイズONとOFFで複数シード→推定誤差の分布が有界・期待σ内。
#   ESKF vs 相補(--target/estimator)の比較は P6 で意味を持つ(N0は両者大差ない見込み)。

# (5) P5 レビュー動画(§9 必須): sf sils scenario .../hover_alt.scn --noise n0 --video
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
- 旧 `sf sils run`(hover_smoke) は欠陥ゆえ P5 の正路ではない。**emu_vehicle 上で行う**。

## 5. 関連
- メモリ: `project_sils_architecture`(P5はN0)、`project_stampfly_emulator`、`project_eskf_vertical_divergence`
  (時間基準修正・ホバー成立)、`project_estimator_attitude_comparison`(相補 vs ESKF 姿勢の宿題=P6で追求)。
- 文書: `RESET_PLAN.md` §10(ロードマップ)/§13(P5-P10 検証能力)、`plant_timebase_bug.md`、
  `firmware/vehicle/docs/noise_and_vibration_model.md`。
- 主要ファイル: `plant/sensor_noise.hpp`(モデル)、`plant/plant.cpp`(適用)、`emu/emu_main_generic.cpp`(配線先)、
  `lib/sfcli/commands/sils.py`(CLI配線先)、`smoke/noise_test.cpp`(単体テスト)、
  `scenarios/hover_long.scn`/`hover_alt.scn`(試験飛行)。

---

## 6. P6 進行中（センサノイズ N1/N2 ＋ ToF/Baro/Flow ＋ ESKF vs 相補）

RESET_PLAN §10/§13 P6。**ゴール**: スロットル依存・帯域制限ノイズ（σ_axis=K[axis]·duty²、f_low/f_high）
と ToF/Baro/Flow の観測ノイズ（R 表）を加え、**ノイズ下で ESKF が相補フィルタより優位なことを定量化**
（P4 比較動画をノイズ版に更新）＋ G4（飽和率）。仕様書 §7 の段階: N1→N2→(N3 Flow)。

### 段階1/3 ✅ N1 スロットル依存振動（2026-06-03 実装済み）
- `sensor_noise.hpp`: `Config::vib_enable`/`vib_accel_k[3]`/`vib_gyro_k[3]` を追加。`drawWhite()` で
  σ_axis=K[axis]·throttle² の白色振動を静的白色に加算（1サンプル rms ゆえ 1/√dt なし）。`setThrottle()` を
  Plant が substep 毎に4モータ平均 duty で呼ぶ。`SILS_EMU_NOISE=n1`（NOISE_LEVELS に追加、emu の
  plant_config_from_env が n0/n1 を分岐）。`noise_test` に軸別 σ＋ゼロスロットル消失を追加。
- 検証: 単体 PASS（軸別 σ ≈ K·duty²）、N0/off byte-identical（不変）、n1 決定論。
- **所見（重要）**: N1（広帯域）下で hover_alt は離陸が揚力不足→沈下（有界・非発散だが hover 品質劣化）。
  広帯域ゆえ低周波成分がファーム LPF を通過し ESKF を実機以上に劣化させる＝**N2 帯域制限の動機**。
  SILS hover duty(~0.62-0.70)は legacy hover02 のフィット点より高く K·duty² が過大気味な点も留意。

### 段階2/3 ✅ N2 帯域制限＋ToF/Baro 観測ノイズ（2026-06-03 実装済み）
- `sensor_noise.hpp`: `vib_bandlimit`/`vib_freq_low,high`、2次帯域通過（RBJ・中心 √(500·667)≈577Hz、
  Q=f0/帯域幅）。**物理 4kHz substep で生成 → ファーム 400Hz 読みで ~100–177Hz にエイリアシング**
  （白色は read レート、振動は substep レートで色付け＝別経路。`stepVibration`/`biquadStep`/`computeBandpass`、
  H2 ノルムで単位 rms 正規化）。`obs_enable`/`tof_sigma`/`baro_sigma`、advance() で ToF/baro サンプルを
  事前抽選し const な `applyTof/applyBaro` が足す。`plant.cpp` の `tof()/baro()` に配線。
- **ToF ノイズの地上問題と修正**: σ=0.03（doc R 表）は ESKF の膨張 R で**真のセンサノイズではない**。
  プラントには真値（datasheet 級 0.01m）を入れる。さらに静止高 13mm では σ が負/無効を生み起動 ToF 判定と
  離着陸状態機械をチャタリングさせ**アーム拒否**→ ToF ノイズは VL53 信頼下限 >5cm でのみ付与、valid は
  真の距離から決める、で解決（armed:1 確認）。baro(σ=0.1)はテレメトリのみ（USE_BAROMETER=false で未融合）。
- 検証: 単体（帯域制限 rms＋自己相関 0.553＋デシメート分散保存／観測ノイズ σ）全 PASS、N0/N1/off
  byte-identical、n2 決定論。Flow(σ=0.30, 高度依存)・Mag は N3 tier（後段）。
- **所見（重要）**: N2（帯域制限）は N1 より**劇的には改善しない**（複数シードで n1≈n2、peak~0.3m）。
  振動の1サンプル大きさ（duty 0.7 で σ_accel≈1.9・σ_gyro≈30°/s）は帯域制限でも不変で、ESKF の
  accel→傾き補正がこの瞬時値に敏感（ジャイロ積分は高周波を均すが accel-tilt は均さない）。SILS hover
  duty(~0.7)が同定点より高く K·duty² が過大気味な点も。**→ フォロー**: ①ファーム IMU フィルタ設定が
  ~100–177Hz をどれだけ落とすか ②K の duty 整合 ③段階3 比較。

### 段階3/3 ✅ ESKF vs 相補の比較・P6 ゲート（2026-06-04 達成）
- **ハーネス裏取り結果**: emu_vehicle は IEstimator(ESKF/相補)を持つが **airborne シナリオ無し・
  estimator 切替 env 無し**＝arm/離陸できず、忠実比較には数日（データ駆動フェーズと重複）。一方 hover_smoke は
  **実 vehicle の推定器を実 IEstimator ファクトリ経由（`estimator.type` param→`imu_task.cpp:72` createEstimator）
  で走らせる**＝推定器比較には忠実（フル firmware 非実行の欠陥は推定器比較自体に無関係）。→ **hover_smoke 採用**。
- hover_smoke に N1/N2 を配線（`smoke/hover_smoke.cpp` の noise_lvl→vib_enable/vib_bandlimit/obs_enable、
  emu と同じ対応）。hover_smoke は baro 融合（use_baro=true/use_tof=false）ゆえ n2 の baro 観測ノイズが効く。
- **定量結果（5シード平均、hover_smoke 自身の g2_att_rmse_deg 厳密指標）**:
  姿勢 comp/ESKF＝N0 0.33x／N1 **2.20x**／N2 0.93x、高度 comp/ESKF＝N0 0.41x／N1 1.60x／N2 1.32x（>1=ESKF優位）。
  **＝低ノイズ(N0)は単純な相補が優位、現実ノイズ(N1振動)では ESKF が明確優位（相補姿勢が 4.26°±3.05 で不安定化）、
  N2 は高度で ESKF 優位**。「中身が違うと結果も違う」＋「ノイズ下で ESKF 優位」を定量実証。
- 比較動画 `viz/out_p6/p6_compare.mp4`（`sf sils compare -m P6 --noise n2`、ゲート承認 pass=true）。
- **将来課題**: emu_vehicle 上の完全忠実比較（airborne シナリオ＋estimator 切替 env）はデータ駆動フェーズと統合。

- **P5 が炙り出した宿題（P6 で追う）**: N0 残留 accel バイアスで ESKF 姿勢が~4°チルト→水平ドリフト。
  **起動校正（水平静止 ba_z≈2g, `noise_and_vibration_model.md` §3）を再現**して全オフセットを捕え、
  チルト/ドリフトを抑える。`project_estimator_attitude_comparison`（相補の方が姿勢安定）の原因究明もここ。
- **着手前に確認**: ノイズモデル（`sensor_noise.hpp`）は N0 のみ。N1/N2 は密度の duty 依存項・帯域制限
  （1次/2次フィルタ）を追加実装する必要がある。ToF/Baro/Flow ノイズは Plant の各センサ合成（`plant.cpp`
  の `tof()/baro()/flow()`）に R を載せる。**firmware 無改変**の原則を守る（ノイズは Plant 側）。
- **比較経路は既存**: `sf sils compare`（hover_smoke ベース）。ただし忠実版は emu_vehicle 上で
  ESKF/相補を切り替える経路（`--target`/estimator 選択）が要る。P6 着手時に emu の estimator 切替を要確認。
