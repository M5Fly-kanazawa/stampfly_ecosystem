# simulator

StampFly仮想実験環境。

> **移植作業**: [MIGRATION_PLAN.md](MIGRATION_PLAN.md) を参照

## 概要

[stampfly_sim](https://github.com/kouhei1970/stampfly_sim) を基に、
現在の `firmware/vehicle/` と互換性のあるシミュレータを構築します。

## 目標ディレクトリ構成

```
simulator/
├── core/           # 物理エンジン（剛体力学、動力学）
├── sensors/        # センサモデル（IMU, Mag, Baro, ToF, OptFlow）
├── control/        # 制御系（PID, Rate/Angle制御）
├── interfaces/     # 外部接続（SIL/HIL, Protocol）
├── visualization/  # 3D可視化（VPython）
├── scenarios/      # テストシナリオ
├── configs/        # 設定ファイル（YAML）
├── assets/         # 3Dモデル、テクスチャ
└── scripts/        # 実行スクリプト
```

## 設計原則

1. **Protocol準拠**: `protocol/` で定義されたメッセージ形式でI/O
2. **実機互換**: センサノイズモデルは実機特性に基づく
3. **段階的実装**: Acro Mode → Angle Mode → Position Hold

## 現状

- [ ] Phase 1: 基盤移植（既存シミュレータの移植）
- [ ] Phase 2: センサ拡張（Baro, OptFlow追加）
- [ ] Phase 3: Protocol統合
- [ ] Phase 4: 制御系統合
- [ ] Phase 5: HIL対応

詳細は [MIGRATION_PLAN.md](MIGRATION_PLAN.md) を参照。

---

# simulator

StampFly virtual testing environment.

> **Migration**: See [MIGRATION_PLAN.md](MIGRATION_PLAN.md)

## Overview

Building a simulator compatible with current `firmware/vehicle/` based on
[stampfly_sim](https://github.com/kouhei1970/stampfly_sim).

## Design Principles

1. **Protocol-based I/O**: Uses message formats defined in `protocol/`
2. **Hardware-compatible**: Sensor noise models based on real hardware
3. **Incremental**: Acro Mode → Angle Mode → Position Hold
