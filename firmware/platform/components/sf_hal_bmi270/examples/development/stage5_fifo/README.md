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

# Stage 5: FIFO - バッチ読み取りによる効率的なデータ取得

## 概要

Stage 5では、BMI270のFIFO（First-In, First-Out）機能を段階的に実装し、SPI通信回数を削減した効率的なデータ取得を実現します。

## 実装ステップ

各ステップは独立したプロジェクトとして実装され、段階的に機能を追加していきます。

### [Step 1: step1_basic_read](step1_basic_read/) - FIFO基本読み取り

**目的**: FIFO機能の最小構成で動作確認

**実装内容**:
- FIFO設定（ACC+GYR、ヘッダーモード）
- FIFO_LENGTH ポーリング
- 1フレーム（13バイト）手動読み取り
- ヘッダー0x8C検証

**検証ポイント**:
- ✓ FIFO設定正常（0xD0）
- ✓ ヘッダー = 0x8C
- ✓ データ精度（acc_z≈1g, gyr≈0°/s）

---

### Step 2: step2_multi_frames - 複数フレーム読み取り

**目的**: 複数フレームの連続読み取りとパーシング

**実装内容**:
- バースト読み取り（100バイト以上）
- フレームパーシングループ
- フレーム数カウント

**検証ポイント**:
- ✓ 7〜8フレーム正常解析
- ✓ データ連続性確認
- ✓ FIFO_LENGTH = 0（読み取り後）

---

### Step 3: step3_flush - FIFOフラッシュとストリームモード

**目的**: FIFO動作モードの確認

**実装内容**:
- FIFOフラッシュコマンド（0xB0）
- ストリームモード確認
- オーバーフロー動作確認

**検証ポイント**:
- ✓ フラッシュ後 FIFO_LENGTH = 0
- ✓ ストリームモードで古いデータ上書き

---

### Step 4: step4_watermark_poll - ウォーターマーク（ポーリング）

**目的**: ウォーターマーク閾値の動作確認

**実装内容**:
- FIFO_WTM_0/WTM_1 設定（128バイト）
- INT_STATUS_1 ポーリング
- ウォーターマーク到達検出

**検証ポイント**:
- ✓ 128バイトで FWM_INT ビット立つ
- ✓ レジスタリードバック値一致

---

### Step 5: step5_watermark_int - ウォーターマーク割り込み

**目的**: 割り込みベースのFIFO読み取り

**実装内容**:
- INT_MAP_DATA 設定
- GPIO割り込みハンドラー
- 割り込み頻度確認

**検証ポイント**:
- ✓ INT1ピン割り込み発生
- ✓ 割り込み頻度が計算値と一致

---

### Step 6: step6_optimized - 最適化（ヘッダーレスモード）

**目的**: 実用的なFIFO運用

**実装内容**:
- ヘッダーレスモード（12バイト/フレーム）
- 512バイトバッチ処理
- Teleplot出力
- パフォーマンス測定

**検証ポイント**:
- ✓ 512バイト安定動作
- ✓ SPI通信削減（100回/秒 → 2.6回/秒 @100Hz）

---

## 最終目標

### 性能指標

| 項目 | Stage 4<br>(割り込み) | Stage 5<br>(FIFO @100Hz) | Stage 5<br>(FIFO @1600Hz) |
|:---|:---:|:---:|:---:|
| SPI通信回数/秒 | 100回 | **2.6回** | **38回** |
| 割り込み回数/秒 | 100回 | **2.6回** | **38回** |
| CPU負荷 | 低 | **最低** | **最低** |
| データ遅延 | < 10µs | 390ms | 26ms |

### 最大性能（1600Hz運用）

- **ODR**: ACC/GYR 1600Hz
- **FIFOモード**: ヘッダーレス（12バイト/フレーム）
- **ウォーターマーク**: 504バイト（42フレーム）
- **データレート**: 19.2 KB/sec
- **割り込み頻度**: 約38回/秒

## ビルド方法

各ステップのディレクトリで：

```bash
source ~/esp/esp-idf/export.sh
cd examples/stage5_fifo/stepX_????/
idf.py build flash monitor
```

## 参考ドキュメント

- [docs/bmi270_doc_ja.md](../../docs/bmi270_doc_ja.md) - BMI270実装ガイド
  - 4.8節: FIFO機能
- BMI270公式データシート（Bosch Sensortec）
