<!--
SPDX-License-Identifier: MIT

Copyright (c) 2025 Kouhei Ito

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-->

# Stage 1: BMI270 SPI Basic Communication Test

## 目的

ESP-IDF SPI Masterドライバを使用してBMI270との基本的なSPI通信を確立し、以下を確認します：

- SPI通信の初期化
- 3バイトトランザクション実装の正確性
- CHIP_ID読み取り（期待値: 0x24）
- 通信の安定性

## ハードウェア

- **MCU**: ESP32-S3 (M5StampS3)
- **センサー**: BMI270 6軸IMU
- **通信**: SPI (10 MHz, Mode 0)

### ピン接続

| 信号 | ESP32-S3 GPIO | BMI270ピン |
|------|---------------|------------|
| MOSI | GPIO14 | SDx (SDI) |
| MISO | GPIO43 | SDO |
| SCK  | GPIO44 | SCx |
| CS   | GPIO46 | CSB |

## ビルド＆実行

### 1. 環境セットアップ

```bash
source ~/esp/esp-idf/export.sh
```

### 2. ターゲット設定（初回のみ）

```bash
cd examples/stage1_spi_basic
idf.py set-target esp32s3
```

### 3. ビルド

```bash
idf.py build
```

### 4. フラッシュ＆モニター

```bash
idf.py flash monitor
```

終了: `Ctrl+]`

## 期待される出力

```
I (xxx) BMI270_TEST: ========================================
I (xxx) BMI270_TEST:  BMI270 Stage 1: SPI Basic Communication
I (xxx) BMI270_TEST: ========================================

I (xxx) BMI270_TEST: Step 1: Initializing SPI...
I (xxx) BMI270_SPI: SPI bus initialized on host 1
I (xxx) BMI270_SPI: BMI270 SPI device added successfully
I (xxx) BMI270_SPI: GPIO - MOSI:14 MISO:43 SCLK:44 CS:46
I (xxx) BMI270_SPI: SPI Clock: 10000000 Hz
I (xxx) BMI270_TEST: ✓ SPI initialized successfully

I (xxx) BMI270_TEST: Step 2: Activating SPI mode...
I (xxx) BMI270_TEST: Performing dummy read to activate SPI mode...
I (xxx) BMI270_TEST: SPI mode activated

I (xxx) BMI270_TEST: Step 3: Testing CHIP_ID...
I (xxx) BMI270_TEST: Reading CHIP_ID...
I (xxx) BMI270_TEST: CHIP_ID = 0x24 (expected: 0x24)
I (xxx) BMI270_TEST: ✓ CHIP_ID verification SUCCESS

I (xxx) BMI270_TEST: Step 4: Testing communication stability...
I (xxx) BMI270_TEST: Testing communication stability (100 iterations)...
I (xxx) BMI270_TEST: Communication test results:
I (xxx) BMI270_TEST:   Success: 100/100 (100.0%)
I (xxx) BMI270_TEST:   Failed:  0/100
I (xxx) BMI270_TEST: ✓ Communication stability test PASSED

I (xxx) BMI270_TEST: ========================================
I (xxx) BMI270_TEST:  ✓ ALL TESTS PASSED
I (xxx) BMI270_TEST: ========================================
I (xxx) BMI270_TEST: Stage 1 completed successfully!
I (xxx) BMI270_TEST: BMI270 SPI communication is working correctly.
I (xxx) BMI270_TEST: Next step: Stage 2 - Initialization sequence
```

## 成功基準

- [x] CHIP_ID = 0x24 を読み取れる
- [x] 100回連続読み取りで100%成功
- [x] SPIエラーが発生しない

## 開発経過と重要な知見

### 問題1: 50%通信失敗（交互失敗パターン）

#### 現象
CHIP_ID読み取りが一つおきに失敗する完璧な交互パターン:
```
Success: rx[0]=0xFE rx[1]=0x00 rx[2]=0x24  ← 正常
Failure: rx[0]=0x93 rx[1]=0x00 rx[2]=0x92  ← 失敗
Success: rx[0]=0xFE rx[1]=0x00 rx[2]=0x24  ← 正常
Failure: rx[0]=0x93 rx[1]=0x00 rx[2]=0x92  ← 失敗
```

#### 試行錯誤の経緯

**試行1: 遅延時間の調整**
- 450µs遅延追加 → 効果なし（50%失敗継続）
- 1000µs遅延 → 効果なし（50%失敗継続）
- 5000µs遅延 → 効果なし（50%失敗継続）

**結論**: 遅延時間は根本原因ではない

#### 根本原因の発見

**決定的要因**: PMW3901（光学フローセンサ）との共有SPIバス

M5StampFlyのSPIバスは以下の2デバイスで共有:
- BMI270: CS = GPIO46
- PMW3901: CS = GPIO12 ← **これが浮いていた！**

PMW3901のCSが不定状態だったため、BMI270の通信に干渉していた。

#### 解決策

`bmi270_spi_init()`でPMW3901のCSを明示的にHIGHに固定:

```c
// bmi270_spi.c (lines 39-57)
if (config->gpio_other_cs >= 0) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->gpio_other_cs),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(config->gpio_other_cs, 1);  // HIGH = 非アクティブ
}
```

**結果**: 即座に100%成功達成

### 問題2: 遅延最適化実験

PMW3901 CS無効化後、各種遅延値でテスト:

| 遅延時間 | 成功率 | 評価 |
|---------|--------|------|
| 50µs    | 100%   | PASS - 最小動作可能値 |
| 100µs   | 100%   | PASS |
| 200µs   | 100%   | PASS |
| 450µs   | 100%   | PASS - データシート推奨値 |
| **1000µs** | **100%** | **PASS - 採用値** |
| 2000µs  | 100%   | PASS |
| 5000µs  | 100%   | PASS |

**採用値決定**: 1000µs
- データシート値(450µs)の約2倍で十分な安全マージン
- 速度と信頼性のバランスが良好
- 温度変動や個体差にも対応可能

### 問題3: ESP32-S3ターゲット未設定

#### 現象
ビルド時に警告:
```
WARNING: CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240 is deprecated
```

#### 原因
- sdkconfig.defaultsでターゲット未指定
- ESP32（デフォルト）でビルドされていた

#### 解決策
```
# sdkconfig.defaults
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y  # ESP-IDF v5.4.1形式
```

### 問題4: 3バイトトランザクションの理解

#### 誤解
CHIP_ID読み取りで0x92が返った際、2バイトトランザクションへの変更を試行

#### 正しい理解
BMI270のSPI Readプロトコルは**必ず3バイト**:
```
TX: [CMD: 0x80|addr] [Dummy] [Dummy]
RX: [Echo]           [Dummy] [DATA]  ← 3バイト目が有効データ
```

2バイト目ではなく**3バイト目を読む**のが正しい実装。

## 重要な学び

### 1. ハードウェア構成の完全把握が最優先
ソフトウェア最適化より先に、共有バス上の全デバイスを把握すべき。

### 2. BMI270は非常に高速
- 50µsでも100%動作可能
- データシート値(450µs)は保守的
- 実機テストで最適値を決定すべき

### 3. 段階的デバッグの有効性
1. プロトコル実装の正しさ確認（3バイト）
2. タイミング調整（遅延値テスト）
3. ハードウェア干渉の排除（PMW3901 CS）

### 4. 実験機能の分離設計
`bmi270_set_lowpower_delay_override()`により、本番コードを変更せずにテスト可能。

## トラブルシューティング

### CHIP_IDが0x00または0xFFを返す

**原因**: 配線ミスまたはSPI設定エラー

**対策**:
1. ピン接続を再確認
2. プルアップ/プルダウン抵抗の確認
3. 3.3V電源電圧の確認
4. オシロスコープで信号確認

### CHIP_IDが異なる値を返す

**原因**: BMI270が別のモードで動作中

**対策**:
1. 電源を再投入
2. ダミーリードを追加実行
3. SPIモード設定を確認（Mode 0またはMode 3）

### 通信が不安定（成功率 < 100%）

**原因**: 共有SPIバスの他デバイスCSが浮いている可能性が高い

**対策**:
1. **最優先**: 共有SPIバス上の全デバイスのCSを確認・無効化
2. SPI クロックを下げる（5 MHz程度）
3. 配線を短くする
4. デカップリングコンデンサを追加

**重要**: M5StampFlyではPMW3901(GPIO12)のCS無効化が必須！

## 次のステップ

Stage 1が成功したら、次は **Stage 2: BMI270初期化シーケンス** に進みます。

Stage 2では以下を実装します：
- 電源投入シーケンス
- 8KBコンフィグファイルのロード
- INTERNAL_STATUSポーリング
- 初期化完了確認

## 参考ドキュメント

- [docs/esp_idf_bmi270_spi_guide.md](../../docs/esp_idf_bmi270_spi_guide.md) - ESP-IDF SPI実装ガイド
- [docs/bmi270_doc_ja.md](../../docs/bmi270_doc_ja.md) - BMI270実装ガイド
- [docs/M5StamFly_spec_ja.md](../../docs/M5StamFly_spec_ja.md) - ハードウェア仕様
