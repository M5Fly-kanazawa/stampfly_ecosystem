# P5 センサノイズ N0 — 再開ノート（次セッションの起点）

> 自己完結の再開メモ。空コンテキストの新セッションでも、このファイルだけで P5 を始められる。
> Self-contained resume note for milestone P5 (sensor noise N0).

最終更新: 2026-06-03 / 直近コミット: `5623ff6`（90秒長時間ホバー）。
**前提**: 全参照は実コード裏取り済み。ロードマップは `simulator/sil/RESET_PLAN.md` §10/§13。

---

## 0. ここまでの到達点（State）

- **P0〜P4 ✅完了**（更地化／物理SIL骨格／アルゴリズム非依存実証／CLI・ダッシュボード／レビュー動画）。
- **エミュレータ E0〜E6 ✅**（実 app_main を14タスクでホスト実行・Plant 閉ループ・決定論シナリオ注入・
  VL53/INA3221/BMP280 モデル）。
- **時間基準バグ修正済**（commit `cea0d8c`, 物理4000Hz・仮想時間1:1ロック）→ 空中ホバー忠実成立、
  90秒長時間ホバーで ESKF 鉛直安定を確証（`hover_long.scn`）。詳細 `simulator/sil/docs/plant_timebase_bug.md`。
- **次の目標 = P5 センサノイズ N0**（RESET_PLAN §10 表 / §13）。

**P5 の合格基準（RESET_PLAN §10）**: 「ノイズ下でホバーが G2（推定誤差有界）＋決定論・統計テストに合格」。
**G2** = 推定誤差が有界（発散しない）。**決定論** = 同シード→同結果（byte-identical）。

---

## 1. ノイズモデルは実装済み・適用済み（作り直し不要）

| 要素 | 状態 |
|------|------|
| ノイズモデル `simulator/sil/plant/sensor_noise.hpp` | **完成**。N0 = 白色ガウス（gyro 0.000122 rad/s/√Hz, accel 0.00157 m/s²/√Hz）＋起動時バイアス1σ（gyro 0.005, accel 0.02 = **起動校正「後」の残留**）＋バイアスRW（gyro 1e-4, accel 1e-3 /√s）。シード付き決定論。 |
| 単体テスト `simulator/sil/smoke/noise_test.cpp` | あり |
| Plant が適用 `plant.cpp` | `noise_.init(cfg_.noise)`(L62)、`imu()` で `applyAccel/applyGyro`(L277-278)、`substep` で `noise_.advance(h)`(L230)。**時間基準修正後は substep(0.25ms)ごとに advance** |
| 仕様書 | `firmware/vehicle_new/docs/noise_and_vibration_model.md` §2-3 |

---

## 2. P5 のギャップ（やること） — エミュレータにノイズを配線する

**問題**: エミュレータ（`emu_vehicle` / `sf sil scenario`）は `g_plant.init(model_path)` を
**既定 Config（noise.enable=false）**で呼ぶ（`emu/emu_main_generic.cpp:154`, `emu/emu_main.cpp:71`）。
→ **ノイズを有効化する経路が無い**。`--noise n0` は旧 `sf sil run`（=`hover_smoke`、核ループ部分試験で
**欠陥**＝実ファーム全コードを走らせない）にしか繋がっていない（`lib/sfcli/commands/sil.py:148` run_run）。

**忠実な P5 は emu_vehicle 上で行う。** 必要な配線（SIL_EMU_TRAJ/SIL_EMU_EVENTS と同じ env パターン）:

### 2-1. emu に noise/seed の env を読ませる
`emu/emu_main_generic.cpp`（および `emu/emu_main.cpp`）で、`g_plant.init(model_path)` の前に
`sil::Plant::Config` を作り、env から noise を設定して `g_plant.init(model_path, cfg)` を呼ぶ:
```cpp
sil::Plant::Config cfg;                       // defaults
const char* noise = std::getenv("SIL_EMU_NOISE");   // "off"(既定) / "n0"
if (noise && std::string(noise) == "n0") {
    cfg.noise.enable = true;
    if (const char* s = std::getenv("SIL_EMU_SEED")) cfg.noise.seed = (uint32_t)atoi(s);
}
g_plant.init(model_path, cfg);                // 既存の init(path) でなく init(path,cfg)
```
※ `Plant::init(const char*, const Config&)` は既存（plant.hpp:113）。既定 off なら現行と byte-identical。

### 2-2. `sf sil scenario` に --noise / --seed を追加（sil.py）
`lib/sfcli/commands/sil.py` の scenario サブパーサ（L93付近）に `--noise`(choices=NOISE_LEVELS, 既定off)
と `--seed`(既定12345) を追加し、`run_scenario`(L243) の subprocess env に
`SIL_EMU_NOISE`/`SIL_EMU_SEED` を渡す（既存の SIL_EMU_EVENTS/SIL_EMU_TRAJ と同じ dict に足す, L261）。

### 2-3. ⚠️ 白色ノイズの σ スケーリングを要確認（時間基準修正の副作用）
`drawWhite(dt)` は σ = density/√dt（連続密度→1サンプル）。**修正後は substep h=0.25ms ごとに白色を
引く**が、**ファームは IMU を 400Hz（2.5ms）で読む**。ファームは Plant の「最新の 0.25ms 分散の白色
サンプル」を読むため、σ が √(2.5ms/0.25ms)=√10≈3.16倍 大きく見える恐れ。
→ **P5 でファームが見る実効 σ が意図した N0（density×√400Hz）になっているか数値確認**し、必要なら
白色を「ファーム読み取りレート（2.5ms）で引く」か density を補正する。
（バイアスRW は √dt 累積ゆえ substep 化で正しい＝総時間で正規化される。問題は白色のみ。）

---

## 3. P5 検証レシピ（配線後）

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
#   render_video.py に --start/--end・カメラ自動フレーミング実装済(commit 3118df3)。
#   ※ 画像確認はサブエージェント限定(CLAUDE.md)。
```

**合格判定（P5 達成）**: ① noise n0 でホバーが有界（G2）② 同シード決定論 ③ 統計テスト合格
④ レビュー動画。達成後 `sensor_noise.hpp` の `@design ... [--]→[OK]`、RESET_PLAN P5 を ✅。

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
