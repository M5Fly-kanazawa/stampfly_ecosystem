# 基準フライトログ一式 / Reference Flight-Log Bundles

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

`sf log` 系ツール・`sf sysid` 系ツール・CI が読み込み確認に使う、実機由来の
「StampFly フライトログ一式」（`.sflog.zip`）を置く。形式は
`protocol/spec/flight_log.yaml`（正本）と `docs/reference/flight-log-format.md` を参照。

## 2. ファイル一覧

| ファイル | 由来 | 内容 | 備考 |
|---------|------|------|------|
| `vehicle_hover_20260908T121243.sflog.zip` | firmware/vehicle、2026-09-08 の屋内ホバー 30 秒。`sf log wifi` の旧 JSONL 出力（`stampfly_udp_20260908T121243.jsonl`、9.0 MB）を `sf log convert` で変換 | 400 Hz 系 11,736 行（imu/attitude/posvel/rate_ref）、pilot 1,467、ctrl_ref 1,467、baro 1,249、flow 2,991、tof_bottom 882、mag 742、status 29 | `seq` は旧 JSONL にパケット通し番号が無いため捕捉順で付与。`sf log check` は警告 5 件（400 Hz 系に同一時刻の重複 1,561 件。計画文書 §7 の機体側の課題）、エラー 0 |

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

## 3. Usage

```bash
sf log check analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf log info  analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf log viz   analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip
sf sysid fit analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip --plot
```
