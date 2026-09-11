# StampFly システム同定ツール

`sf sysid` サブコマンド群のバックエンド実装です。フライトデータから物理パラメータ（慣性・
モータ特性・空力抵抗・センサノイズ・プラント伝達関数）を同定します。

> **重要:** ツールは全て **sf CLI**（`sf sysid ...`）経由で使用してください。このディレクトリの
> Python モジュールを直接実行しません。

## 入力形式

`noise`／`inertia`／`motor`／`drag`／`fit`／`rate-fit` の入力は、いずれも**StampFlyフライト
ログv1一式**（`.sflog.zip`。信号ごとのCSVと`meta.json`/`schema.json`をzipにまとめたもの。
以下「一式」）です。`sf log wifi` で取得した一式、または SILS（Software-In-the-Loop
Simulation：実ファームをホストPC上でエミュレータ実行して評価する仕組み）が書き出す一式を
そのまま渡せます。

```bash
sf log wifi -d 30
sf sysid fit logs/flight_20260911T121243.sflog.zip --plot
```

一式は複数レート（400Hz IMU、50Hz操縦入力、1Hzバッテリ等）の信号を原レートのまま埋め値なしで
持つため、同定処理側が必要な時刻粒度に整列する。この整列（複数の信号を共通の時刻に揃えて1枚の
表にする処理）は `tools/sysid/loader.py` の `load_aligned(path, base="imu", method="hold")`
が `lib/sflog` の `FlightLog.load()` + `aligned()` を呼んでメモリ上だけで行い、ファイルには
書き出さない。1枚の表をファイルとして眺めたい・外部ツールへ渡したい場合だけ、明示的に

```bash
sf log convert logs/flight_20260911T121243.sflog.zip --aligned
```

で派生物（`<名前>_aligned400.csv`、同名 `.meta.json` に `derived: true` 付き）を作る。
同定ロジックは `load_aligned()` が返す DataFrame をそのまま使うため、ファイル化した整列CSVを
経由する必要はない。

## sf sysid コマンド一覧

| コマンド | 説明 | バックエンド |
|---------|------|-------------|
| `sf sysid fit` | 閉ループのtarget→gyro応答からプラント $G_p(s)=K/(s(\tau_m s+1))$ を同定 | `plant_fit.py` |
| `sf sysid rate-fit` | レートループの伝達関数 $G(s)=b\,e^{-Ls}/(s(Ts+1))$ をETFE（経験伝達関数推定）＋非線形フィットで同定 | `tools/log_analyzer/rate_sysid.py` |
| `sf sysid rate-tune` | 同定済みモデルからゲイン交差周波数・位相余裕仕様を満たすPIDゲインを設計 | `tools/log_analyzer/rate_sysid.py` |
| `sf sysid rate-excite` | Tello風APIで励振信号（チャープ・ダブレット）を飛行させる | `tools/log_analyzer/rate_sysid.py` |
| `sf sysid noise` | センサノイズ特性（Allan分散：静止データからノイズ密度・バイアス不安定性を求める手法） | `noise.py` |
| `sf sysid inertia` | ステップ応答から慣性モーメント $I_{xx}/I_{yy}/I_{zz}$ を同定 | `inertia.py` |
| `sf sysid motor` | 推力係数 $C_T$・トルク係数 $C_Q$・時定数 $\tau_m$ を同定 | `motor.py`、`steady_state.py` |
| `sf sysid drag` | 減速データから並進・回転の空力抵抗係数を同定 | `drag.py` |
| `sf sysid params` | 同定結果パラメータの表示・比較・書き出し（YAML/JSON、一式ではなくパラメータファイルが入力） | `params.py`、`defaults.py`、`_generated_params.py` |
| `sf sysid validate` | 同定結果の物理的整合性検査（ホバー推力釣り合い・慣性の大小関係・κ=Cq/Ct 等） | `validation.py` |
| `sf sysid plan` | フライト試験計画の生成（一式ではなく試験手順の文書を出力） | - |

`params`／`validate`／`plan`／`rate-tune`／`rate-excite` は一式を直接読まない（既に同定済みの
パラメータファイル、または試験計画・励振信号そのものを扱う）。

## モジュール一覧

| ファイル | 役割 |
|---------|------|
| `loader.py` | 一式読み込みの共通処理。`load_aligned()`（`lib/sflog` 経由で一式→整列DataFrame）、`sample_rate_hz()`（実測サンプリングレート推定） |
| `plant_fit.py` | `sf sysid fit` の実体。間接閉ループ方式（既定）／`control_output`／`duty`／`kp` の各入力復元モードを実装 |
| `noise.py` | Allan分散解析 |
| `inertia.py` | 慣性モーメント同定 |
| `motor.py` | モータ動特性同定（$C_T$/$C_Q$/$\tau_m$） |
| `steady_state.py` | モータ定常特性（トルク釣り合い）モデル |
| `drag.py` | 空力抵抗係数同定 |
| `params.py` | パラメータのロード・保存・マージ・差分・検証 |
| `defaults.py` | StampFly既定物理パラメータ（`docs/architecture/stampfly-parameters.md` 参照） |
| `_generated_params.py` | `control/models/stampfly_physical.yaml` から `sf params generate` で自動生成（手編集禁止） |
| `validation.py` | 同定結果の物理的整合性検査 |
| `visualizer.py` | Allan分散曲線・ステップ応答フィット・ノイズヒストグラム等のプロット |
| `test_bundle_commands.py`、`test_loader.py` | pytest: 一式ベースの読み込み・各コマンドの単体テスト |

## 典型的なワークフロー

フライトログを取得する（一式として保存）:

```bash
sf log wifi -d 30
```

プラントを同定する（結果をYAMLへ保存）:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip -o plant.yaml --plot
```

同定結果ファイルの物理的整合性を検査する:

```bash
sf sysid validate plant.yaml
```

各コマンドの詳細オプションは `docs/commands/sf-sysid.md`、および `sf sysid <サブコマンド>
--help` を参照してください。
