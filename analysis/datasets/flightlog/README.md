# 基準フライトログ一式 / Reference Flight-Log Bundles

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

`sf log` 系ツール・`sf sysid` 系ツール・CI が読み込み確認に使う、実機由来の
「StampFly フライトログ一式」（`.sflog.zip`）を置く。形式は
`protocol/spec/flight_log.yaml`（基準ファイル）と `docs/reference/flight-log-format.md` を参照。

## 2. ファイル一覧

| ファイル | 由来 | 内容 | 備考 |
|---------|------|------|------|
| `vehicle_hover_20260908T121243.sflog.zip` | firmware/vehicle、2026-09-08 の屋内ホバー 30 秒。`sf log wifi` の旧 JSONL 出力（`stampfly_udp_20260908T121243.jsonl`、9.0 MB）を `sf log convert` で変換 | 400 Hz 系 11,736 行（imu/attitude/posvel/rate_ref）、pilot 1,467、ctrl_ref 1,467、baro 1,249、flow 2,991、tof_bottom 882、mag 742、status 29 | `seq` は旧 JSONL にパケット通し番号が無いため捕捉順で付与。`sf log check` は警告 5 件（400 Hz 系に同一時刻の重複 1,561 件。計画文書 §7 の機体側の課題）、エラー 0 |
| `vehicle_roll_flip_20261002T184904.sflog.zip` | firmware/vehicle、2026-10-02 のロール宙返り試験飛行（旧名 `flip_test1`）。`sf log wifi` で取得（30 s、WiFi 経由 400 Hz） | imu 11,880 行ほか（400 Hz 系、pilot/ctrl_ref、baro、flow、tof、mag、status）。宙返りの窓 15.85〜16.25 s、ホバー窓 5.5〜15.5 s | `sf sils sysid-gate` の実機基準ログ（窓・閾値は `forward_check.yaml`）。同一時刻の IMU 行の再利用あり（順モデル照合で処理） |
| `vehicle_roll_flip_20261002T205207.sflog.zip` | 同上、同日の 2 本目（旧名 `flip_test2`）。`sf log wifi` で取得（30 s） | imu 11,920 行ほか。宙返りの窓 6.66〜7.10 s（ロール）、ホバー窓 4.4〜6.4 s | 同上。ピッチの窓 6.95〜7.45 s は参考表示のみ |

`forward_check.yaml` は `sf sils sysid-gate`（順モデル照合）の基準ログ・解析窓・合格基準を定める（`docs/architecture/simulation-policy.md` §5）。

## 3. 使い方

```bash
sf log check analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf log info  analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf log viz   analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf sysid fit analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip --plot
```

---

<a id="english"></a>

## 1. Overview

Real-vehicle "StampFly flight-log bundles" (`.sflog.zip`) used by the `sf log`
and `sf sysid` tools and by CI as read-compatibility fixtures. Format: see
`protocol/spec/flight_log.yaml` (source of truth) and
`docs/reference/flight-log-format.md`.

## 2. Files

| File | Origin | Contents | Notes |
|------|--------|----------|-------|
| `vehicle_hover_20260908T121243.sflog.zip` | firmware/vehicle, 30 s indoor hover on 2026-09-08; converted with `sf log convert` from the legacy JSONL capture (`stampfly_udp_20260908T121243.jsonl`, 9.0 MB) | 11,736 rows in each 400 Hz stream (imu/attitude/posvel/rate_ref), pilot 1,467, ctrl_ref 1,467, baro 1,249, flow 2,991, tof_bottom 882, mag 742, status 29 | `seq` is synthesized from capture order (the legacy JSONL has no packet sequence). `sf log check`: 5 warnings (1,561 repeated timestamps in the 400 Hz streams, see plan section 7), 0 errors |
| `vehicle_roll_flip_20261002T184904.sflog.zip` | firmware/vehicle, roll-flip test flight on 2026-10-02 (former name `flip_test1`). Captured with `sf log wifi` (30 s, 400 Hz over WiFi) | imu 11,880 rows and the other 400 Hz streams, pilot/ctrl_ref, baro, flow, tof, mag, status. Flip window 15.85-16.25 s, hover window 5.5-15.5 s | Real-vehicle reference log of `sf sils sysid-gate` (windows and thresholds in `forward_check.yaml`). Contains reused-IMU rows (handled by the forward-model check) |
| `vehicle_roll_flip_20261002T205207.sflog.zip` | Same, second flight of the day (former name `flip_test2`). Captured with `sf log wifi` (30 s) | imu 11,920 rows etc. Flip window 6.66-7.10 s (roll), hover window 4.4-6.4 s | Same. The pitch window 6.95-7.45 s is reference only |

`forward_check.yaml` defines the reference logs, analysis windows and pass criteria of `sf sils sysid-gate` (the forward-model check; `docs/architecture/simulation-policy.md` section 5).

## 3. Usage

```bash
sf log check analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf log info  analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf log viz   analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf sysid fit analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip --plot
```
