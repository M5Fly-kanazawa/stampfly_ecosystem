# sf doctor

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

開発環境の問題を診断し、解決方法を提案します。

## 2. 構文

```bash
sf doctor
```

## 3. チェック項目

| 項目 | 説明 |
|------|------|
| ESP-IDF | ESP-IDF のインストールとバージョン |
| Python | Python バージョンと依存パッケージ |
| Project | プロジェクト構造とファイル |
| Serial | シリアルポートの検出 |

## 4. 出力例

```
[INFO] Running environment diagnostics...

ESP-IDF:
  [OK] Found: /Users/user/esp/esp-idf
  [OK] Version: v5.5.2

Python:
  [OK] Version: 3.12.0
  [OK] pyserial installed
  [OK] numpy installed

Project:
  [OK] Root: /path/to/stampfly-ecosystem
  [OK] Vehicle firmware found
  [OK] Controller firmware found

Serial:
  [OK] Port found: /dev/tty.usbmodem14101

[OK] All checks passed!
```

## 5. トラブルシューティング

### ESP-IDF が見つからない

```bash
# ESP-IDF環境をアクティブ化
source ~/esp/esp-idf/export.sh
```

### pyserial がインストールされていない

```bash
pip install pyserial
```

---

<a id="english"></a>

## 1. Overview

Diagnose development environment issues and suggest solutions.

## 2. Syntax

```bash
sf doctor
```

## 3. Checks

| Item | Description |
|------|-------------|
| ESP-IDF | ESP-IDF installation and version |
| Python | Python version and dependencies |
| Project | Project structure and files |
| Serial | Serial port detection |
