# StampFly Ecosystem

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 概要

StampFly Ecosystem は、ドローン制御工学のための教育・研究プラットフォームです。
設計 → 実装 → 実験 → 解析 → 教育の完全なワークフローをカバーします。

## クイックスタート

```bash
# 環境セットアップ
source setup_env.sh

# ファームウェアのビルドと書き込み
sf build vehicle
sf flash vehicle -m
```

詳しくは [クイックスタートガイド](getting-started.md) を参照してください。

## 主なコンテンツ

| セクション | 説明 |
|-----------|------|
| [入門](getting-started.md) | 環境構築〜初フライト |
| [アーキテクチャ](architecture/control-system.md) | 制御系設計・座標系・パラメータ |
| [sf CLI](commands/README.md) | コマンドラインツール |
| [ガイド](guides/safety.md) | 安全・トラブルシューティング |
| [教育](workshop/workshop_guide.md) | ワークショップ・大学講義 |
| [スライド資料](slides.md) | ワークショップ PDF スライド |
| [ドキュメント目録](DOCUMENT_INDEX.md) | 全ドキュメントの一覧 |

## リポジトリ構成

```
stampfly-ecosystem/
├── docs/          ドキュメント（このサイト）
├── firmware/      ファームウェア（vehicle / controller / workshop）
├── protocol/      通信プロトコル仕様
├── control/       制御設計（モデル・PID・MPC）
├── analysis/      データ解析（ノートブック・スクリプト）
├── tools/         開発ツール
├── simulator/     SILS/HIL テスト
└── examples/      学習用サンプル
```

---

<a id="english"></a>

## Overview

StampFly Ecosystem is an educational/research platform for drone control engineering.
It covers the complete workflow: design → implementation → experimentation → analysis → education.

## Quick Start

```bash
# Environment setup
source setup_env.sh

# Build and flash firmware
sf build vehicle
sf flash vehicle -m
```

See the [Getting Started Guide](getting-started.md) for details.

## Main Contents

| Section | Description |
|---------|-------------|
| [Getting Started](getting-started.md) | Environment setup to first flight |
| [Architecture](architecture/control-system.md) | Control system, coordinates, parameters |
| [sf CLI](commands/README.md) | Command-line tools |
| [Guides](guides/safety.md) | Safety & troubleshooting |
| [Education](workshop/workshop_guide.md) | Workshop & university courses |
| [Slides](slides.md) | Workshop PDF slides |
| [Document Index](DOCUMENT_INDEX.md) | Complete document listing |
