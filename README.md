# StampFly Ecosystem

StampFly 機体を中心に、ドローン制御を **設計・実装・実験・解析・教育** の
すべての段階で一貫して扱うための **教育・研究用エコシステム**。

## ディレクトリ構成

```
stampfly-ecosystem/
├── docs/           # 人間が読むためのドキュメント
├── firmware/       # 組込みファームウェア
│   ├── vehicle/    # 機体ファームウェア（制御対象）
│   ├── controller/ # 送信機ファームウェア（HMI）
│   └── common/     # 共有コード
├── protocol/       # 通信プロトコル仕様（SSOT）
├── control/        # 制御設計資産
├── analysis/       # 実験データ解析
├── tools/          # 補助ツール
├── simulator/      # 仮想実験環境
├── examples/       # 最短で動く入口
└── third_party/    # 外部依存
```

## はじめに

1. [docs/getting-started.md](docs/getting-started.md) を読む
2. [examples/](examples/) から始める
3. 詳細は [PROJECT_PLAN.md](PROJECT_PLAN.md) を参照

---

# StampFly Ecosystem

An **educational and research ecosystem** for drone control engineering,
covering all stages: **design → implementation → experimentation → analysis → education**.

## Directory Structure

```
stampfly-ecosystem/
├── docs/           # Human-readable documentation
├── firmware/       # Embedded firmware
│   ├── vehicle/    # Vehicle firmware (plant)
│   ├── controller/ # Transmitter firmware (HMI)
│   └── common/     # Shared code
├── protocol/       # Communication protocol spec (SSOT)
├── control/        # Control design assets
├── analysis/       # Experiment data analysis
├── tools/          # Utility tools
├── simulator/      # Virtual testing environment
├── examples/       # Quick start examples
└── third_party/    # External dependencies
```

## Getting Started

1. Read [docs/getting-started.md](docs/getting-started.md)
2. Start with [examples/](examples/)
3. See [PROJECT_PLAN.md](PROJECT_PLAN.md) for details
