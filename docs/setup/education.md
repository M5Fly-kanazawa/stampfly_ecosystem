# StampFly 教育環境 セットアップガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 前提条件

### ハードウェア

| 項目 | 要件 |
|------|------|
| PC | Python 3.10以上（推奨3.12）が動作する Windows/macOS/Linux |
| StampFly | 組み立て済み、バッテリー充電済み |
| WiFi | StampFly の WiFi AP に接続可能 |

### ソフトウェア

| ツール | バージョン | 用途 |
|--------|---------|------|
| Python | 3.10以上（推奨3.12） | 開発言語 |
| pip | 最新版 | パッケージ管理 |
| Git | 最新版 | コード管理 |
| ブラウザ | Chrome/Firefox | Jupyter Notebook |

## 2. インストール手順

### リポジトリのクローン

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

### Python 環境のセットアップ

```bash
# 仮想環境を作成（推奨）
python -m venv .venv
source .venv/bin/activate    # macOS/Linux
# .venv\Scripts\activate     # Windows

# 教育用パッケージをインストール
pip install -e ".[edu]"
```

### サンプルデータの生成

```bash
python -m stampfly_edu.generate_samples
```

### 動作確認

```bash
# Jupyter Notebook を起動
jupyter notebook analysis/notebooks/education/
```

`01_hello_stampfly.ipynb` を開き、最初のセルを実行してエラーがないことを確認してください。

## 3. StampFly への接続

### WiFi 接続

1. StampFly の電源を入れる
2. PC の WiFi 設定で `StampFly_XXXX` に接続
3. パスワード: `stampfly` (デフォルト)
4. IP アドレス: `192.168.4.1`

### 接続テスト

```python
from stampfly_edu import connect_or_simulate
drone = connect_or_simulate()
print(drone.get_battery())
drone.end()
```

## 4. トラブルシューティング

### よくある問題

| 問題 | 解決策 |
|------|--------|
| `ModuleNotFoundError: stampfly_edu` | `pip install -e ".[edu]"` を再実行 |
| WiFi 接続できない | StampFly のバッテリーを確認、PC の WiFi を再接続 |
| Jupyter が起動しない | `pip install jupyter` を実行 |
| サンプルデータがない | `python -m stampfly_edu.generate_samples` を実行 |

---

<a id="english"></a>

## 1. Prerequisites

### Hardware

| Item | Requirement |
|------|------------|
| PC | Windows/macOS/Linux with Python 3.10 or later (3.12 recommended) |
| StampFly | Assembled with charged battery |
| WiFi | Able to connect to StampFly WiFi AP |

### Software

| Tool | Version | Purpose |
|------|---------|---------|
| Python | 3.10 or later (3.12 recommended) | Development language |
| pip | Latest | Package management |
| Git | Latest | Code management |
| Browser | Chrome/Firefox | Jupyter Notebook |

## 2. Installation

### Clone Repository

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

### Python Environment Setup

```bash
# Create virtual environment (recommended)
python -m venv .venv
source .venv/bin/activate    # macOS/Linux
# .venv\Scripts\activate     # Windows

# Install education packages
pip install -e ".[edu]"
```

### Generate Sample Data

```bash
python -m stampfly_edu.generate_samples
```

### Verify Installation

```bash
jupyter notebook analysis/notebooks/education/
```

Open `01_hello_stampfly.ipynb` and run the first cell to verify no errors.

## 3. Connecting to StampFly

### WiFi Connection

1. Power on StampFly
2. Connect PC WiFi to `StampFly_XXXX`
3. Password: `stampfly` (default)
4. IP address: `192.168.4.1`

### Connection Test

```python
from stampfly_edu import connect_or_simulate
drone = connect_or_simulate()
print(drone.get_battery())
drone.end()
```

## 4. Troubleshooting

| Issue | Solution |
|-------|----------|
| `ModuleNotFoundError: stampfly_edu` | Re-run `pip install -e ".[edu]"` |
| Cannot connect WiFi | Check StampFly battery, reconnect PC WiFi |
| Jupyter won't start | Run `pip install jupyter` |
| No sample data | Run `python -m stampfly_edu.generate_samples` |
