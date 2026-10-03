# 教育用サンプルログ / Educational Sample Logs

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

実機がなくても `sf log` 系・`sf sysid` 系の解析を練習できるように置いた、
「StampFly フライトログ一式」（`.sflog.zip`、形式の第1版）5本。形式は
`protocol/spec/flight_log.yaml`（基準ファイル）と `docs/reference/flight-log-format.md` を参照。

5本とも**シミュレーションで生成したデータ**であり、実機で取ったログではない
（`meta.json` の `"source": "sim"`）。生成したツールは
`stampfly_edu.generate_samples` 0.1.0（2026-09-11、`d85b7971` の時点）で、このリポジトリには含まれていない。
実機のログは `../flightlog/` にある。

## 2. ファイル一覧

| ファイル | 内容 | ストリームと行数 | 長さ |
|---------|------|----------------|------|
| `hover_30s.sflog.zip` | ホバリング | posvel / attitude / imu / motor 各 12,000、status 30 | 30.0 s |
| `rate_step_response.sflog.zip` | 角速度のステップ応答 | imu / motor 各 1,200 | 3.0 s |
| `altitude_step.sflog.zip` | 高度のステップ | imu / posvel 各 4,000、baro 500、tof_bottom 300 | 10.0 s |
| `square_path.sflog.zip` | 四角形の経路 | imu / posvel 各 830 | 16.6 s |
| `static_noise_60s.sflog.zip` | 静止状態のセンサ雑音 | imu 24,000、baro 3,000、tof_bottom 1,800 | 60.0 s |

5本とも `lib/sflog` の検査（`python3 -m sflog.check`）でエラーなし（2026-10-03 時点）。

## 3. 使い方

```bash
sf log check analysis/datasets/education/hover_30s.sflog.zip
sf log info  analysis/datasets/education/hover_30s.sflog.zip
sf log viz   analysis/datasets/education/hover_30s.sflog.zip
```

---

<a id="english"></a>

## 1. Overview

Five "StampFly flight-log bundles" (`.sflog.zip`, format version 1) for practicing
the `sf log` and `sf sysid` analyses without a vehicle. Format: see
`protocol/spec/flight_log.yaml` (source of truth) and
`docs/reference/flight-log-format.md`.

All five are **simulation-generated data**, not recordings from a real vehicle
(`"source": "sim"` in `meta.json`). They were produced by
`stampfly_edu.generate_samples` 0.1.0 (2026-09-11, at `d85b7971`), which is not
included in this repository. Real-vehicle logs are in `../flightlog/`.

## 2. Files

| File | Contents | Streams and rows | Duration |
|------|----------|------------------|----------|
| `hover_30s.sflog.zip` | Hover | posvel / attitude / imu / motor 12,000 each, status 30 | 30.0 s |
| `rate_step_response.sflog.zip` | Angular-rate step response | imu / motor 1,200 each | 3.0 s |
| `altitude_step.sflog.zip` | Altitude step | imu / posvel 4,000 each, baro 500, tof_bottom 300 | 10.0 s |
| `square_path.sflog.zip` | Square path | imu / posvel 830 each | 16.6 s |
| `static_noise_60s.sflog.zip` | Sensor noise at rest | imu 24,000, baro 3,000, tof_bottom 1,800 | 60.0 s |

All five pass the `lib/sflog` check (`python3 -m sflog.check`) with no errors (as of 2026-10-03).

## 3. Usage

```bash
sf log check analysis/datasets/education/hover_30s.sflog.zip
sf log info  analysis/datasets/education/hover_30s.sflog.zip
sf log viz   analysis/datasets/education/hover_30s.sflog.zip
```
