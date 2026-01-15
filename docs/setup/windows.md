# Windows セットアップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

WindowsでのStampFly開発環境セットアップ手順です。WSL2の使用を強く推奨します。

## 2. 推奨構成

| 方法 | 推奨度 | 説明 |
|------|--------|------|
| WSL2 + Ubuntu | ★★★ | 最も安定、Linux版と同じ手順 |
| ネイティブ Windows | ★☆☆ | 制限あり、上級者向け |

## 3. WSL2セットアップ（推奨）

### WSL2のインストール

PowerShell（管理者）で実行:

```powershell
wsl --install -d Ubuntu-22.04
```

再起動後、Ubuntuを起動してユーザー設定を完了します。

### Ubuntu内でのセットアップ

WSL2 Ubuntuターミナルで、[Linuxガイド](linux.md)の手順に従ってください。

### USBデバイスの接続（WSL2）

WSL2でUSBデバイスを使用するには、usbipd-winが必要です:

1. Windows側でusbipd-winをインストール:
   - https://github.com/dorssel/usbipd-win/releases からダウンロード

2. PowerShell（管理者）でデバイスを接続:

```powershell
# デバイス一覧を表示
usbipd list

# デバイスをWSL2に接続（BUSIDは上記で確認）
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

3. WSL2側で確認:

```bash
ls /dev/ttyUSB*
```

## 4. ネイティブWindowsセットアップ（非推奨）

### ESP-IDF Windows Installerを使用

1. https://dl.espressif.com/dl/esp-idf/ からインストーラをダウンロード
2. ESP-IDF v5.5.2を選択してインストール
3. ESP-IDF PowerShellまたはCommand Promptを使用

### 制限事項

- sf CLIの一部機能が動作しない可能性があります
- シミュレータ機能は未対応です
- パスの扱いが異なるため問題が発生する場合があります

## 5. シリアルポートドライバ

CH9102F（M5Stack製品）用ドライバ:
- https://www.wch.cn/downloads/CH343SER_EXE.html

## 6. 動作確認（WSL2）

```bash
# ESP-IDF環境をアクティブ化
source ~/esp/esp-idf/export.sh

# プロジェクトディレクトリに移動
cd /mnt/c/path/to/stampfly-ecosystem

# 環境診断
sf doctor
```

## 7. トラブルシューティング

### USBデバイスがWSL2で認識されない

```powershell
# Windows側で再度接続
usbipd detach --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

### パフォーマンスが遅い

WSL2のLinuxファイルシステム内（`/home/user/`）で作業することを推奨します。`/mnt/c/`はパフォーマンスが低下します。

---

<a id="english"></a>

## 1. Overview

Setup instructions for StampFly development environment on Windows. WSL2 is strongly recommended.

## 2. Recommended Configuration

| Method | Rating | Description |
|--------|--------|-------------|
| WSL2 + Ubuntu | ★★★ | Most stable, same steps as Linux |
| Native Windows | ★☆☆ | Limited, for advanced users |

## 3. WSL2 Setup (Recommended)

### Install WSL2

Run in PowerShell (Administrator):

```powershell
wsl --install -d Ubuntu-22.04
```

After restart, launch Ubuntu and complete user setup.

### Setup in Ubuntu

In WSL2 Ubuntu terminal, follow the [Linux guide](linux.md).

### USB Device Connection (WSL2)

To use USB devices in WSL2, you need usbipd-win:

1. Install usbipd-win on Windows:
   - Download from https://github.com/dorssel/usbipd-win/releases

2. Connect device in PowerShell (Administrator):

```powershell
# List devices
usbipd list

# Connect to WSL2 (use BUSID from above)
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

3. Verify in WSL2:

```bash
ls /dev/ttyUSB*
```

## 4. Native Windows Setup (Not Recommended)

### Using ESP-IDF Windows Installer

1. Download installer from https://dl.espressif.com/dl/esp-idf/
2. Select ESP-IDF v5.5.2 and install
3. Use ESP-IDF PowerShell or Command Prompt

### Limitations

- Some sf CLI features may not work
- Simulator features are not supported
- Path handling differences may cause issues

## 5. Serial Port Driver

CH9102F (M5Stack products) driver:
- https://www.wch.cn/downloads/CH343SER_EXE.html

## 6. Verify Installation (WSL2)

```bash
# Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Navigate to project
cd /mnt/c/path/to/stampfly-ecosystem

# Run diagnostics
sf doctor
```
