# SNN 姿勢推定・制御 移植 — Stage 2a/2b（教師データ収集基盤 + 学習ループ）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### この検討について

Stroobants et al. 2025 "Neuromorphic Attitude Estimation and Control"（arXiv:2411.13945）の
SNN（スパイキングニューラルネット）姿勢推定・制御を StampFly へ移植する計画の **Stage 2a**。

- **Stage 1**（コミット済み: `feat(snn): wire up untrained SNN estimator/controller skeleton`）:
  `firmware/vehicle/components/sf_estimator_snn` / `sf_controller_snn` / `sf_snn_core` の骨格実装。
  CUBA-LIF ニューロン層 + 線形読み出し。重みは固定シード乱数の**未学習プレースホルダー**（学習コードなし）。
- **Stage 2a**（本ディレクトリ）: 模倣学習の**教師データ収集基盤**。既存の ESKF+PID（デフォルト設定、
  `estimator.type=0` / `controller.type=0` のまま）で SILS シナリオを実行し、フライトログを収集・整形する。
- **Stage 2b**（実装済み・本ディレクトリ、§6-8参照）: 収集したデータで模倣学習の学習ループ（BPTT・損失関数）を実装し、
  実際に学習を実行して推定器・制御器それぞれの重みをエクスポートした。C++ Stage1 骨格へ配線する作業自体
  （per-neuron tau/theta setter の追加、C++側読み出しスケールとの整合）は **Stage 3 として引き続き未着手**
  （§8 参照）。

### 対象読者

SNN 推定器・制御器（`sf_estimator_snn`/`sf_controller_snn`）に学習済み重みを載せる作業を引き継ぐ人。

## 2. venv セットアップ

学習系ライブラリ（torch 等）を Genesis シミュレータ用 venv や `sf` CLI の Python 環境から**分離**するため、
本ディレクトリ専用の venv を使う。

```bash
cd control/design/snn_attitude
python3 -m venv venv                    # 初回のみ。Python 3.12 系を推奨（動作確認は 3.12.12）
./venv/bin/pip install -r requirements.txt
```

`lib/sflog` は pip パッケージ化されていない素のモジュールディレクトリなので、`import sflog` するには
リポジトリの `lib/` を `sys.path`（または `PYTHONPATH`）に追加する必要がある。`build_dataset.py` は
ファイル冒頭で `sys.path.insert(0, <repo_root>/lib)` を自動的に行うので、スクリプト経由なら意識しなくてよい。
対話的に使う場合は以下のどちらか:

```bash
# 方法A: PYTHONPATH
PYTHONPATH=/path/to/stampfly_ecosystem/lib ./venv/bin/python

# 方法B: スクリプト内で sys.path に追加
./venv/bin/python -c "
import sys; sys.path.insert(0, '/path/to/stampfly_ecosystem/lib')
import sflog
"
```

`venv/` はリポジトリ直下の `.gitignore` の `venv/`（拡張子なしパターン、全階層に効く）で既に除外されている。
本ディレクトリに追加の `.gitignore` は不要（確認済み: `git check-ignore` で除外を確認）。

## 3. データ収集手順

既存の ESKF+PID（`estimator.type`/`controller.type` は変更せずデフォルト値 0 のまま）で SILS シナリオを実行し、
フライトログ（`.sflog.zip`）を収集する。`simulator/sils/scenarios/*.scn` は姿勢ステップ・ホバー・宙返り等
複数のシナリオを持つ（一覧は `simulator/sils/scenarios/TEST_MATRIX.md`）。書式はスクリプト形式
（`<t> ch <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> [<alt> <acro> ... <pos>]` のイベント列。
`+` は前イベント末尾からの相対時刻、生 12bit ADC センタ2048）。

本 Stage 2a では**姿勢変化の種類が異なる 5 シナリオ**を選定した（層別に PASS 確認済みのものを優先し、
2026-07-26 時点で複合ロール+ピッチが motor duty 飽和で地面衝突する既知の欠陥がある `pos_flight`/`pos_yaw`
は除外 — 教師データとして質の高い、飽和・墜落のないサンプルを優先するため）:

| シナリオ | モード | 励振 | 収録先 |
|---------|--------|------|--------|
| `acro_flight` | ACRO（レート制御） | roll/pitch/yaw 短いレートダブレット | `data/acro_flight.sflog.zip` |
| `stab_flight` | STABILIZE（姿勢制御・自己水平化） | roll ±8°・pitch +8° ステップ | `data/stab_flight.sflog.zip` |
| `alt_flight` | ALT_HOLD（高度保持） | 鉛直のみ（roll/pitch 擾乱なし） | `data/alt_flight.sflog.zip` |
| `pos_roll` | POS_HOLD（位置保持） | ロール単独ドリフト→捕捉保持 | `data/pos_roll.sflog.zip` |
| `pos_pitch` | POS_HOLD（位置保持） | ピッチ単独ドリフト→捕捉保持 | `data/pos_pitch.sflog.zip` |

実行コマンド（`SF_ROOT_OVERRIDE`/`PYTHONPATH` はこの作業ツリー用の sf CLI ワークアラウンド。詳細は
auto-memory `sf-cli-points-to-other-clone.md` 参照）:

```bash
source setup_env.sh
export SF_ROOT_OVERRIDE=$(pwd) PYTHONPATH=$(pwd)/lib
sf sils build -t vehicle
sf sils scenario simulator/sils/scenarios/acro_flight.scn --target vehicle
sf sils scenario simulator/sils/scenarios/stab_flight.scn --target vehicle
sf sils scenario simulator/sils/scenarios/alt_flight.scn  --target vehicle
sf sils scenario simulator/sils/scenarios/pos_roll.scn    --target vehicle
sf sils scenario simulator/sils/scenarios/pos_pitch.scn   --target vehicle
```

5 シナリオとも全チェック PASS（G1 ログ + G2/G3/G4 数値メトリクス）。生成された
`simulator/sils/viz/out_scn_<name>/sils_<name>_<timestamp>.sflog.zip` を
`control/design/snn_attitude/data/<name>.sflog.zip` にコピーする。

## 4. `build_dataset.py` の使い方

`data/*.sflog.zip` を `sflog.aligned()` で 400Hz IMU 基準に整列し、モータ・PID出力が有効な
（ARM〜DISARM の）飛行区間だけを抜き出して `<name>_aligned.npz` を書く。

```bash
./venv/bin/python build_dataset.py                  # data/ 配下の全 *.sflog.zip を変換
./venv/bin/python build_dataset.py data/acro_flight.sflog.zip   # 個別指定も可
```

各 `.npz` の内容（列の詳細は `column_doc` キーにも同梱）:

| キー | shape | 内容 |
|------|-------|------|
| `t_s` | (N,) | 飛行開始からの経過秒 |
| `seq` | (N,) | 制御周期の通し番号 |
| `gyro_raw`/`accel_raw` | (N,3) | IMU 生データ（rad/s, m/s^2） |
| `gyro`/`accel` | (N,3) | バイアス・スケール補正後の IMU（ESKF融合前） |
| `quat` | (N,4) | ESKF 推定姿勢クォータニオン [w,x,y,z]（body FRD→NED） |
| `euler` | (N,3) | 同、roll/pitch/yaw [rad]（3-2-1抽出） |
| `thrust`/`torque` | (N,)/(N,3) | PID 出力（ミキサー直前, N / N·m） |
| `duty` | (N,4) | ミキサー出力 [FR,RR,RL,FL], 0..1 |
| `rate_ref` | (N,3) | 内側ループ目標角速度 [rad/s]（ACRO の制御対象） |
| `angle_ref` | (N,2) | 外側ループ目標姿勢 roll/pitch [rad]（純ACRO区間はNaN） |
| `total_thrust_ref` | (N,) | 外側ループ目標推力 [N] |
| `pilot_throttle`/`pilot_rpy` | (N,)/(N,3) | 生スティック指令（throttle 0..1, roll/pitch/yaw -1..1） |
| `flight_mode` | (N,) | 飛行モード enum |
| `gyro_bias` | (N,3) | ESKF推定ジャイロバイアス [rad/s]（Stage 2bで追加。制御器の学習入力 `state.angular_rate` = `gyro - gyro_bias` の再構成に必要、§6参照） |

## 5. Stage 2b への申し送り

- データの質: 5シナリオとも SILS 決定論実行（`--noise off`）で PASS、墜落・duty飽和なし。
- 姿勢変化のバリエーション: ACRO で yaw が最大 87.5° まで回転、STABILIZE/POS_HOLD で roll/pitch が
  ±10° 程度。**大角度（宙返り等）のサンプルは含まれていない** — 含めるなら `api_flip_roll`/`api_flip_pitch`
  シナリオが候補だが、Flip 中の制御則は通常の姿勢/レート PID と異なる専用スケジュールの可能性があり
  （要調査）、模倣学習のターゲットとして単純に混ぜてよいか要検討。
- 静止に近いサンプル（`alt_flight`）は roll/pitch/yaw が厳密に 0.00° — ALT_HOLD シナリオが roll/pitch
  スティック擾乱を含まないため。学習データの多様性としては他4シナリオほど有用でない可能性がある。
- データ量: 5シナリオ合計 25,520 行（400Hz）≈ 63.8 秒分。小規模ネットワークの初期学習には足りるはずだが、
  過学習を避けるにはシナリオ追加（他の層別シナリオ・ノイズ有り再実行等）を検討する余地がある。

## 6. Stage 2b: 学習ループ（実装済み）

`dataset.py`（データセット構築）・`train.py`（BPTT学習ループ）・`export_weights.py`（重みエクスポート）を実装し、
推定器・制御器それぞれについて実際に学習を実行した。数式（CUBA-LIFニューロン層のダイナミクス・サロゲート勾配・
論文Eq.5の模倣学習損失）は `cuba_lif_torch.py`/`losses.py` にすでに実装済みのものをそのまま使用し、本 Stage で
変更していない。

### 入力特徴量の検証

学習を始める前に、C++ Stage1実装（`snn_estimator.cpp`/`snn_controller.cpp`）が実際に受け取る値と、データセットの
列が一致するかをコードレベルで確認した（詳細な引用は `dataset.py` モジュール docstring 参照）:

| 確認項目 | 結論 | 根拠 |
|---------|------|------|
| 推定器の `imu.gyro`/`imu.accel` は `gyro`/`gyro_raw`・`accel`/`accel_raw` のどちらに対応するか | `gyro`/`accel`（この機体ファームでは両者は完全に同一値。`np.allclose` で確認、最大差0.0） | `imu_task.cpp::applyImuTransform()` は単位変換のみ（バイアス補正なし）で `sensor_imu` へ発行。`flight_log.yaml` も `gyro_x`/`accel_x` を「推定器入力」、`_raw` を「vehicleでは同値」と明記 |
| `CommandSetpoint.roll/pitch/yaw/throttle` は `pilot_rpy`/`pilot_throttle` と同スケールか | 完全一致、変換不要 | `sf_telemetry/data_stream.cpp` が `command_setpoint.latest()` の値をそのままテレメトリへコピーしており、`control_task.cpp` が制御器へ渡す値と全く同じ Pub-Sub 値 |
| 制御器の `state.angular_rate`（ESKFのバイアス補正済み角速度）をどう再構成するか | `gyro - gyro_bias` | `eskf_core.cpp::predict()` の `ang_rate_ = gyro_raw - bg_` に対応。Stage 2a の元の列集合に `gyro_bias` が無かったため、本 Stage で `build_dataset.py` に追加し全5シナリオを再生成した（§4のテーブル参照。純粋な追加のみで既存列は不変であることを確認済み） |

### ネットワーク構成・ハイパーパラメータ

C++ Stage1骨格と完全一致させた（`train.py`の`_NETWORK_SPECS`）:

| | 推定器 | 制御器 |
|---|---|---|
| 入力次元 | 6（gyro×3, accel/g×3） | 10（angular_rate×3, euler×3, setpoint×4） |
| 隠れ層 | `CubaLifLayer(6→32, recurrent=True, n_fixed=0)` | `CubaLifLayer(10→16, recurrent=False, n_fixed=4)` |
| 出力 | `LinearReadout(32→3)`（roll/pitch/yaw, rad） | `LinearReadout(16→4)`（thrust N, torque×3 N·m） |

`n_fixed=4`（制御器の固定積分ニューロン数）: 論文は150ニューロン中10個（≈6.7%）を固定するが、本ネットワークは
16ニューロンなので単純比例では1個。推力+3軸トルクの4出力チャンネルそれぞれに最低限の積分容量を持たせる意図で
4個を採用した（要調整可、根拠は `train.py` の `_NETWORK_SPECS` コメントにも記載）。

学習ハイパーパラメータ（最終実行値、全て CLI で変更可能）:

| パラメータ | 値 | 備考 |
|-----------|-----|------|
| window / stride | 400 / 100 ステップ | 1秒 / 0.25秒 @ 400Hz |
| shift | 6 ステップ | 論文式の遅延補償。末尾6ステップは`valid_mask`で損失から除外 |
| optimizer | Adam, lr=1e-3 | |
| epochs | 100 | |
| grad_clip | `clip_grad_norm_` max_norm=1.0 | **必須**。無効化すると400ステップBPTTで勾配ノルムが数千に達し発散する（サニティチェックで確認） |
| batch_size | 32 | train windows=205, val windows=33（`stab_flight`のみ） |

### 学習中に発見した2つの数値上の問題（`cuba_lif_torch.py`/`losses.py`は変更せず、`train.py`側で回避）

1. **losses.py の特異点**: `pearson_corr`の`den = sqrt(...) + eps`は`eps`が`sqrt`の**外側**にあり、ウィンドウ内で
   予測または目標が完全に定数（分散0）になると`torch.sqrt(0)`の勾配がNaNになる（`torch.sqrt(torch.tensor(0.,
   requires_grad=True)).backward()`で直接確認）。`pos_roll`/`pos_pitch`/`alt_flight`は軸別励起シナリオのため
   非励起軸のトルクが文字通り0.0（決定論SILSのため）で、これに未学習ネットワークの「無発火で出力一定」も重なり、
   実データの最初のバッチから即座にNaN勾配が発生することを確認した。**該当シナリオを除外すると学習窓の約8割を
   失う**ため不採用とし、代わりに学習（backward）経路にのみ`pred`/`target`双方へ独立な微小ノイズ（std=1e-6、
   実信号や報告精度より4-5桁小さい）を加えて特異点を回避した（検証用のforward/評価経路は一切汚染しない）。
   詳細は`train.py`モジュール docstring 参照。
2. **出力チャネルのスケール不均衡**（lossesのバグではなく、Eq.5がチャネル間重み付けを持たないことに起因する
   学習品質の問題）: 制御器は thrust（~0.1-0.4N）と torque（~1e-4-1e-3N·m）で2-3桁のスケール差があり、正規化なしで
   100エポック学習した予備実験では thrust RMSE は改善する一方 torque RMSE はほぼ動かず、目標のダイナミックレンジの
   約20倍という結果になった。対処として学習分割のみから求めたチャネル別標準偏差（`output_scale`、下限1e-4）で
   目標を正規化して学習し、物理単位への復元（RMSE計算・重みエクスポート）は線形読み出しの性質上
   `pred_physical = pred_normalized * output_scale` で厳密に行える（`readout.weight`の対応する行をスケールする
   だけ）。詳細は`train.py`モジュール docstring 参照。

### 実行方法

```bash
cd control/design/snn_attitude
./venv/bin/python train.py --target estimator
./venv/bin/python train.py --target controller
./venv/bin/python export_weights.py            # 両方まとめてエクスポート
```

### 結果

| ネットワーク | ベストepoch | val loss (Eq.5) | 検証RMSE |
|------------|------------|-----------------|---------|
| 推定器 | 69/100 | 2.736 | 姿勢RMSE(pooled)=3.105° [roll=4.015° pitch=3.048° yaw=1.871°] |
| 制御器 | 90/100 | 4.666 | thrust RMSE=0.0684N, torque RMSE=0.000400N·m |

参考: 論文 Table I は SNN推定器 3.03° vs PID基準 2.67°（同じ「姿勢RMSE」の定義かは要確認、本実装は学習・検証データが
遥かに小規模）。学習曲線は `runs/estimator_loss.png` / `runs/controller_loss.png`（train/val の loss・MSE・
Pearson相関の推移）。検証損失は33ウィンドウ（`stab_flight`のみ）由来で分散が大きく、特に制御器のPearson相関は
エポック間で大きく振動する（データ量に起因、§7参照）。

### エクスポートした重み

| ファイル | 内容 |
|---------|------|
| `runs/estimator_best.pt` / `runs/controller_best.pt` | 学習済み`state_dict`（ベスト検証epoch）+ `spec` + `output_scale` |
| `runs/estimator_weights.npz` / `runs/controller_weights.npz` | w_in, w_rec(推定器のみ), w_out(物理単位), tau_mem, tau_syn, theta のアーカイブ |
| `runs/estimator_weights_draft.hpp` / `runs/controller_weights_draft.hpp` | 同内容をC++ `constexpr float`配列としたドラフト（**未配線**、§8参照） |
| `runs/estimator_history.json` / `runs/controller_history.json` | 全100エポックのtrain/val損失・MSE・相関の履歴 |

## 7. 既知の制約

- **検証データが1シナリオ・33ウィンドウのみ**: `stab_flight`単体のため、val損失・相関のエポック間分散が大きい
  （§6の学習曲線参照）。将来的にはシナリオを増やすか、検証専用の複数シナリオ交差検証を検討する価値がある。
- **大角度（宙返り等）のサンプルなし**: Stage 2aから引き継いだ制約（§5参照）。現状の学習データは概ね±10°以内。
- **出力チャネルのスケール不均衡ワークアラウンド**: `output_scale`の下限1e-4は、学習シナリオで文字通り分散0の
  torque_pitch/torque_yawチャンネル（`pos_roll`/`pos_pitch`はそれぞれ他方の軸が常に0）を正規化する際に効く。
  検証シナリオ（`stab_flight`）ではこれらの軸に本物の（小さいが非0の）信号があるため、正規化後の相対誤差が
  大きく見え、val lossの見かけの大きさ（4.6台）に寄与している。物理単位のRMSE（thrust/torque）は妥当な値。
- **`gyro_bias`の追加による`build_dataset.py`再生成**: Stage 2aのnpzに欠けていたESKF推定ジャイロバイアス列を
  追加し、全5シナリオを再変換した（既存列は不変、純粋な追加のみ確認済み）。

## 8. Stage 3 への申し送り

C++ Stage1骨格（`snn_estimator.cpp`/`snn_controller.cpp`/`sf_snn_core`）への実配線は本Stageのスコープ外。
`export_weights.py`が生成する`*_weights_draft.hpp`のヘッダコメントに詳細を記載しているが、要点:

1. **per-neuron tau/theta setter が未実装**: `CubaLifLayer::setUniformParams()`は全ニューロン共通値しか
   設定できない。学習済みはニューロンごとに異なる値を持つため、`setNeuronParams(tau_mem[], tau_syn[], theta[])`
   のような新規メソッドを`sf_snn_core/cuba_lif_layer.{hpp,cpp}`に追加してから配線すること。
2. **C++側読み出しの後段スケール定数との二重スケール**: `snn_estimator.cpp`の`kAttitudeReadoutScale`(0.2)、
   `snn_controller.cpp`の`kThrustReadoutGain`(0.05)/`kTorqueReadoutGain`(1e-3)/`kHoverThrustN`(0.363N加算)は
   未学習プレースホルダー向けの後段スケーリングであり、本Stageでエクスポートした重み（`w_out`は既に物理単位
   終端まで学習済み）にそのまま適用すると二重スケールになる。C++側の該当定数を無効化する（scale=1, gain=1,
   hover-prior=0）か、`w_out`をさらに事前除算するかはStage3の設計判断（特に`kHoverThrustN`の加算項は線形
   読み出しの重みスケーリングだけでは厳密に打ち消せないため、単純な自動補正はしていない）。
3. **`n_fixed=4`の根拠は比例配分ではなく判断**: §6参照。学習結果を見て見直す余地がある。
4. firmware側の変更は本Stageで一切行っていない（`firmware/vehicle/`配下は無変更）。

---

<a id="english"></a>

## 1. Overview

### About This Study

**Stage 2a** of the plan to port Stroobants et al. 2025 "Neuromorphic Attitude Estimation and Control"
(arXiv:2411.13945) SNN attitude estimation/control to StampFly.

- **Stage 1** (committed: `feat(snn): wire up untrained SNN estimator/controller skeleton`): the skeleton
  implementation of `firmware/vehicle/components/sf_estimator_snn` / `sf_controller_snn` / `sf_snn_core`
  (CUBA-LIF neuron layer + linear readout). Weights are fixed-seed random **untrained placeholders**
  (no training code).
- **Stage 2a** (this directory): the **imitation-learning teacher-data collection** infrastructure. Runs
  SILS scenarios with the existing ESKF+PID (default settings, `estimator.type=0` / `controller.type=0`
  unchanged) and packages the resulting flight logs.
- **Stage 2b** (implemented, this directory, see §6-8): the training loop (BPTT, loss functions) on the
  collected data, actually run, with exported weights for both networks. Wiring the result into the C++
  Stage 1 skeleton (adding a per-neuron tau/theta setter, reconciling it with the C++-side readout scale)
  is **still Stage 3, not started** (see §8).

### Target Audience

Whoever picks up loading trained weights into the SNN estimator/controller
(`sf_estimator_snn`/`sf_controller_snn`).

## 2. venv Setup

A dedicated venv for this directory, kept **separate** from the Genesis simulator's venv and the `sf` CLI's
own Python environment.

```bash
cd control/design/snn_attitude
python3 -m venv venv                    # once. Python 3.12.x recommended (verified on 3.12.12)
./venv/bin/pip install -r requirements.txt
```

`lib/sflog` is a plain module directory, not pip-installed, so `import sflog` needs the repo's `lib/` on
`sys.path` (or `PYTHONPATH`). `build_dataset.py` does this automatically at the top of the file, so no extra
setup is needed to run it as a script. For interactive use, either:

```bash
# Option A: PYTHONPATH
PYTHONPATH=/path/to/stampfly_ecosystem/lib ./venv/bin/python

# Option B: sys.path inside the script
./venv/bin/python -c "
import sys; sys.path.insert(0, '/path/to/stampfly_ecosystem/lib')
import sflog
"
```

`venv/` is already excluded by the repo root `.gitignore`'s bare `venv/` pattern (matches at any depth); no
extra `.gitignore` was needed in this directory (verified with `git check-ignore`).

## 3. Data Collection

Runs SILS scenarios with the existing ESKF+PID (`estimator.type`/`controller.type` left at their default 0)
and collects flight logs (`.sflog.zip`). `simulator/sils/scenarios/*.scn` covers attitude steps, hover, flips
and more (catalog: `simulator/sils/scenarios/TEST_MATRIX.md`). The format is a scripted event list
(`<t> ch <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> [<alt> <acro> ... <pos>]`; `+` is relative to
the previous event's end; sticks are raw 12-bit ADC, centre 2048).

Stage 2a picked **5 scenarios with distinct kinds of attitude change** (preferring ones known to PASS
per-layer; `pos_flight`/`pos_yaw` were excluded because of a known combined roll+pitch motor-duty-saturation
crash as of 2026-07-26 -- prioritizing clean, non-saturated teacher samples):

| Scenario | Mode | Excitation | Stored as |
|----------|------|-----------|-----------|
| `acro_flight` | ACRO (rate) | short roll/pitch/yaw rate doublets | `data/acro_flight.sflog.zip` |
| `stab_flight` | STABILIZE (self-levelling) | roll ±8°, pitch +8° steps | `data/stab_flight.sflog.zip` |
| `alt_flight` | ALT_HOLD | vertical only (no roll/pitch disturbance) | `data/alt_flight.sflog.zip` |
| `pos_roll` | POS_HOLD | roll-only drift, then hold | `data/pos_roll.sflog.zip` |
| `pos_pitch` | POS_HOLD | pitch-only drift, then hold | `data/pos_pitch.sflog.zip` |

Commands (`SF_ROOT_OVERRIDE`/`PYTHONPATH` are this working tree's sf-CLI workaround; see auto-memory
`sf-cli-points-to-other-clone.md`):

```bash
source setup_env.sh
export SF_ROOT_OVERRIDE=$(pwd) PYTHONPATH=$(pwd)/lib
sf sils build -t vehicle
sf sils scenario simulator/sils/scenarios/acro_flight.scn --target vehicle
sf sils scenario simulator/sils/scenarios/stab_flight.scn --target vehicle
sf sils scenario simulator/sils/scenarios/alt_flight.scn  --target vehicle
sf sils scenario simulator/sils/scenarios/pos_roll.scn    --target vehicle
sf sils scenario simulator/sils/scenarios/pos_pitch.scn   --target vehicle
```

All 5 PASSED every check (G1 log + G2/G3/G4 numeric metrics). The resulting
`simulator/sils/viz/out_scn_<name>/sils_<name>_<timestamp>.sflog.zip` files were copied to
`control/design/snn_attitude/data/<name>.sflog.zip`.

## 4. Using `build_dataset.py`

Aligns each `data/*.sflog.zip` to the 400 Hz IMU timebase via `sflog.aligned()`, restricts to the window
where the motor/PID output is valid (armed-to-disarmed flight), and writes `<name>_aligned.npz`.

```bash
./venv/bin/python build_dataset.py                  # convert every *.sflog.zip under data/
./venv/bin/python build_dataset.py data/acro_flight.sflog.zip   # or a specific file
```

Each `.npz`'s contents (column order also documented in its own `column_doc` key):

| Key | Shape | Content |
|-----|-------|---------|
| `t_s` | (N,) | Seconds since the flight window started |
| `seq` | (N,) | Control-cycle sequence number |
| `gyro_raw`/`accel_raw` | (N,3) | Raw IMU (rad/s, m/s^2) |
| `gyro`/`accel` | (N,3) | Bias/scale-corrected IMU (pre-ESKF-fusion) |
| `quat` | (N,4) | ESKF-estimated attitude quaternion [w,x,y,z] (body FRD -> NED) |
| `euler` | (N,3) | Same, roll/pitch/yaw [rad] (3-2-1 extraction) |
| `thrust`/`torque` | (N,)/(N,3) | PID output (pre-mixer, N / N*m) |
| `duty` | (N,4) | Mixer output [FR,RR,RL,FL], 0..1 |
| `rate_ref` | (N,3) | Inner-loop rate setpoint [rad/s] (ACRO's controlled state) |
| `angle_ref` | (N,2) | Outer-loop attitude setpoint roll/pitch [rad] (NaN in pure ACRO) |
| `total_thrust_ref` | (N,) | Outer-loop thrust setpoint [N] |
| `pilot_throttle`/`pilot_rpy` | (N,)/(N,3) | Raw stick command (throttle 0..1, roll/pitch/yaw -1..1) |
| `flight_mode` | (N,) | Flight-mode enum |
| `gyro_bias` | (N,3) | ESKF-estimated gyro bias [rad/s] (added in Stage 2b -- needed to reconstruct the controller's training input `state.angular_rate` = `gyro - gyro_bias`, see §6) |

## 5. Notes for Stage 2b

- Data quality: all 5 scenarios PASSED deterministically (`--noise off`), no crash or duty saturation.
- Attitude-variation coverage: yaw rotates up to 87.5 deg under ACRO; roll/pitch reach roughly +-10 deg
  under STABILIZE/POS_HOLD. **No large-angle (flip) samples are included** -- `api_flip_roll`/`api_flip_pitch`
  are candidates, but the flip maneuver likely uses a dedicated schedule rather than the ordinary
  attitude/rate PID (needs checking before mixing it in as an imitation target).
  - `alt_flight` (the near-static sample) has roll/pitch/yaw pinned exactly at 0.00 deg, since that scenario
  has no roll/pitch stick disturbance -- likely less useful for training diversity than the other 4.
- Data volume: 25,520 rows total across the 5 scenarios (400 Hz) ~= 63.8 s. Likely enough to start training
  a small network, but adding more scenarios (other per-axis ones, noisy re-runs) is worth considering to
  avoid overfitting.

## 6. Stage 2b: Training Loop (Implemented)

`dataset.py` (dataset construction), `train.py` (BPTT training loop) and `export_weights.py` (weight export)
are implemented, and both networks were actually trained. The math (CUBA-LIF neuron dynamics, surrogate
gradient, the paper's Eq.5 imitation loss) is entirely the pre-existing `cuba_lif_torch.py`/`losses.py`,
unchanged by this Stage.

### Input feature verification

Before training, this Stage checked in the C++ source that the dataset's columns match what
`snn_estimator.cpp`/`snn_controller.cpp` actually receive at inference (full citations in `dataset.py`'s
module docstring):

| Check | Conclusion | Basis |
|-------|-----------|-------|
| Does the estimator's `imu.gyro`/`imu.accel` correspond to `gyro`/`gyro_raw` or `accel`/`accel_raw`? | `gyro`/`accel` (byte-identical to the `_raw` columns on this firmware target -- verified with `np.allclose`, max diff 0.0) | `imu_task.cpp::applyImuTransform()` only unit-converts (no bias correction) before publishing to `sensor_imu`; `flight_log.yaml` documents `gyro_x`/`accel_x` as "estimator input" and the `_raw` columns as "equal on vehicle" |
| Does `CommandSetpoint.roll/pitch/yaw/throttle` match `pilot_rpy`/`pilot_throttle`'s scale? | Exact match, no conversion needed | `sf_telemetry/data_stream.cpp` copies `command_setpoint.latest()` verbatim into telemetry -- the SAME Pub-Sub value `control_task.cpp` passes to the controller |
| How to reconstruct the controller's `state.angular_rate` (ESKF's bias-corrected rate)? | `gyro - gyro_bias` | Matches `eskf_core.cpp::predict()`'s `ang_rate_ = gyro_raw - bg_`. Stage 2a's original column set lacked `gyro_bias`, so this Stage added it to `build_dataset.py` and re-ran all 5 scenarios (see §4's table; verified this was a pure addition, existing columns unchanged) |

### Network sizing / hyperparameters

Matches the C++ Stage 1 skeleton exactly (`train.py`'s `_NETWORK_SPECS`):

| | Estimator | Controller |
|---|---|---|
| input dim | 6 (gyro x3, accel/g x3) | 10 (angular_rate x3, euler x3, setpoint x4) |
| hidden | `CubaLifLayer(6->32, recurrent=True, n_fixed=0)` | `CubaLifLayer(10->16, recurrent=False, n_fixed=4)` |
| output | `LinearReadout(32->3)` (roll/pitch/yaw, rad) | `LinearReadout(16->4)` (thrust N, torque x3 N*m) |

`n_fixed=4` (the controller's fixed-integrator neuron count): the paper fixes 10 of its 150 neurons (~6.7%);
naive scaling for this 16-neuron network gives ~1, but 4 was used instead so each of the 4 distinct output
channels (thrust + 3 torques) gets some minimal slow/DC-tracking capacity (adjustable; rationale also in
`train.py`'s `_NETWORK_SPECS` comment).

Training hyperparameters (final run values, all CLI-overridable):

| Parameter | Value | Note |
|-----------|-------|------|
| window / stride | 400 / 100 steps | 1s / 0.25s @ 400Hz |
| shift | 6 steps | paper-style delay compensation; the last 6 steps of each window are excluded from the loss via `valid_mask` |
| optimizer | Adam, lr=1e-3 | |
| epochs | 100 | |
| grad_clip | `clip_grad_norm_` max_norm=1.0 | **mandatory** -- disabling it lets the 400-step BPTT's gradient norm reach the thousands and diverge (confirmed by a sanity check) |
| batch_size | 32 | 205 train windows, 33 val windows (`stab_flight` only) |

### Two numerical issues found during training (worked around in `train.py`, `cuba_lif_torch.py`/`losses.py` left unchanged)

1. **losses.py singularity**: `pearson_corr`'s `den = sqrt(...) + eps` adds `eps` OUTSIDE the `sqrt`, so when a
   window's prediction or target is exactly constant (zero variance), `torch.sqrt(0)`'s gradient is NaN
   (confirmed directly: `torch.sqrt(torch.tensor(0., requires_grad=True)).backward()`). `pos_roll`/
   `pos_pitch`/`alt_flight` are deliberately single-axis-excitation scenarios, so the off-axis torque is
   bit-for-bit 0.0 (deterministic SILS) -- combined with an untrained network's own all-zero (non-firing)
   output early in training, this produced NaN gradients starting from the very first real batch. Dropping
   the affected scenarios was rejected (it would eliminate ~80% of the training windows); instead,
   independent tiny noise (std=1e-6, 4-5 orders of magnitude below real signal / reporting precision) is
   added to BOTH `pred` and `target` on the training (backward) path only, which avoids the singularity
   without touching any validation/reporting path. See `train.py`'s module docstring for the full writeup.
2. **Output-channel scale imbalance** (not a losses.py bug -- a training-quality consequence of Eq.5 having
   no per-channel weighting): the controller's thrust (~0.1-0.4 N) and torque (~1e-4-1e-3 N*m) differ by
   2-3 orders of magnitude. An unnormalized 100-epoch pilot run showed thrust RMSE improving steadily while
   torque RMSE barely moved, ending ~20x above the torque target's own dynamic range. Fix: normalize each
   channel's target by its TRAINING-split std (`output_scale`, floored at 1e-4) before computing the loss;
   because the readout is linear with no bias, this is exactly reversible for every physical-unit use
   (RMSE here, and the exported weights -- scale each `readout.weight` row by the matching `output_scale`
   entry). See `train.py`'s module docstring for the full writeup.

### Running it

```bash
cd control/design/snn_attitude
./venv/bin/python train.py --target estimator
./venv/bin/python train.py --target controller
./venv/bin/python export_weights.py            # exports both at once
```

### Results

| Network | best epoch | val loss (Eq.5) | val RMSE |
|---------|-----------|------------------|----------|
| Estimator | 69/100 | 2.736 | attitude RMSE (pooled)=3.105 deg [roll=4.015 pitch=3.048 yaw=1.871] |
| Controller | 90/100 | 4.666 | thrust RMSE=0.0684 N, torque RMSE=0.000400 N*m |

For reference, the paper's Table I reports 3.03 deg (SNN estimator) vs 2.67 deg (PID baseline) -- whether
it is exactly the same "attitude RMSE" definition is unverified, and this port's training/validation data is
far smaller. Loss curves: `runs/estimator_loss.png` / `runs/controller_loss.png` (train/val loss, MSE, and
Pearson-correlation traces). Validation loss comes from only 33 windows (`stab_flight` alone) and is
correspondingly noisy epoch-to-epoch, especially the controller's Pearson correlation (a data-volume issue,
see §7).

### Exported weights

| File | Contents |
|------|----------|
| `runs/estimator_best.pt` / `runs/controller_best.pt` | trained `state_dict` (best val epoch) + `spec` + `output_scale` |
| `runs/estimator_weights.npz` / `runs/controller_weights.npz` | archival w_in, w_rec (estimator only), w_out (physical units), tau_mem, tau_syn, theta |
| `runs/estimator_weights_draft.hpp` / `runs/controller_weights_draft.hpp` | the same values as C++ `constexpr float` arrays, a draft (**NOT wired up**, see §8) |
| `runs/estimator_history.json` / `runs/controller_history.json` | full 100-epoch train/val loss, MSE, and correlation history |

## 7. Known Limitations

- **Validation is one scenario / 33 windows**: `stab_flight` alone, so val loss/correlation are noisy
  epoch-to-epoch (see §6's loss curves). Adding more validation scenarios or cross-validation is worth
  considering.
- **No large-angle (flip) samples**: inherited from Stage 2a (see §5). Current training data stays within
  roughly +-10 deg.
- **Output-scale-imbalance workaround's floor**: `output_scale`'s 1e-4 floor normalizes torque_pitch/
  torque_yaw channels that are LITERALLY zero-variance in the training scenarios (`pos_roll`/`pos_pitch`
  each pin the other axis at 0). The validation scenario (`stab_flight`) has real (small but nonzero)
  signal on those axes, so the normalized relative error looks large there, inflating the reported val
  loss's apparent magnitude (~4.6). The physical-unit RMSEs (thrust/torque) are the more meaningful numbers.
- **`build_dataset.py` re-run for `gyro_bias`**: this Stage added the ESKF-estimated gyro bias column that
  was missing from Stage 2a's npz files and re-converted all 5 scenarios (verified this was a pure addition,
  existing columns unchanged).

## 8. Notes for Stage 3

Wiring these weights into the C++ Stage 1 skeleton (`snn_estimator.cpp`/`snn_controller.cpp`/`sf_snn_core`)
is out of this Stage's scope. `export_weights.py`'s generated `*_weights_draft.hpp` headers document this in
full; the key points:

1. **No per-neuron tau/theta setter yet**: `CubaLifLayer::setUniformParams()` only sets one value for every
   neuron. The trained values differ per neuron, so a new method (e.g. `setNeuronParams(tau_mem[], tau_syn[],
   theta[])`) needs to be added to `sf_snn_core/cuba_lif_layer.{hpp,cpp}` before wiring these up.
2. **Double-scaling against the C++ readout's own placeholder constants**: `snn_estimator.cpp`'s
   `kAttitudeReadoutScale` (0.2), and `snn_controller.cpp`'s `kThrustReadoutGain` (0.05) /
   `kTorqueReadoutGain` (1e-3) / `kHoverThrustN` (+0.363N) are untrained-placeholder post-scaling; this
   Stage's exported `w_out` is ALREADY in physical units end-to-end, so applying it as-is would double-scale.
   Either neutralize those C++ constants (scale=1, gain=1, hover-prior=0) or pre-divide `w_out` further --
   a Stage 3 design decision (the additive `kHoverThrustN` term in particular cannot be exactly cancelled by
   weight rescaling alone, since a linear no-bias readout cannot represent a constant offset, so this was
   deliberately not auto-corrected here).
3. **`n_fixed=4`'s rationale is a judgment call, not a proportional derivation**: see §6; worth revisiting
   in light of training results.
4. No firmware changes were made in this Stage (`firmware/vehicle/` is untouched).
