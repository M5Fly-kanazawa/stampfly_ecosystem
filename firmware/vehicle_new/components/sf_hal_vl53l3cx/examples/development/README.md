# StampFly ToF サンプルプロジェクト集

このフォルダには、段階的にVL53L3CX ToFセンサーの機能をテストできる独立したESP-IDFプロジェクトが含まれています。

## 使い方

各サンプルフォルダは独立したESP-IDFプロジェクトです。そのフォルダ内で直接ビルド・実行できます。

```bash
# Stage 1に移動
cd stage1_i2c_scan

# ESP-IDF環境を有効化
. ~/esp/esp-idf/export.sh

# ターゲット設定（初回のみ）
idf.py set-target esp32s3

# ビルド・フラッシュ・モニタ
idf.py build flash monitor
```

各プロジェクトは親ディレクトリの `stampfly_tof` コンポーネントを自動的に参照します。

## サンプル一覧

### ✅ Stage 1: I2Cバススキャン

**ファイル**: [stage1_i2c_scan/main.c](stage1_i2c_scan/main.c)

**目的**: I2C通信の基本動作確認

**テスト内容**:
- I2Cマスター初期化（SDA=GPIO3, SCL=GPIO4, 400kHz）
- XSHUTピン制御（前方：ON、底面：OFF）
- I2Cバススキャン（0x03～0x77）

**期待結果**:
```
Device found at address 0x29
  -> VL53L3CX detected at default address!
```

**必要なもの**:
- StampFly ToFコンポーネント（基本構成のみ）
- ESP-IDFのI2Cドライバ

**ビルド確認**: ✅ 成功

---

### ✅ Stage 2: レジスタ読み書きテスト

**ファイル**: [stage2_register_test/main.c](stage2_register_test/main.c)

**目的**: VL53L3CXレジスタへの直接アクセス

**テスト内容**:
- プラットフォーム層I2C関数実装
- Model ID (0x010F) 読み出し → 期待値: 0xEA
- Module Type (0x0110) 読み出し → 期待値: 0xAA（注: 0xCCはVL53L1の値）

**期待結果**:
```
Model ID (0x010F): 0xEA [OK]
Module Type (0x0110): 0xAA [OK]
```

**必要なもの**:
- vl53lx_platform.c/h（プラットフォーム層）
- 基本的な型定義ヘッダー

**ビルド確認**: ✅ 成功
**実機テスト**: ✅ 成功

---

### 🔲 Stage 3: デバイス初期化（未実装）

**目的**: VL53LX API初期化シーケンス確認

**テスト内容**:
- `VL53LX_WaitDeviceBooted()`
- `VL53LX_DataInit()`
- `VL53LX_GetDeviceInfo()`

**必要なもの**:
- VL53LXコアドライバ（最小構成）
- プラットフォーム層完成版

---

### 🔲 Stage 4: ポーリング測定（未実装）

**目的**: 基本的な距離測定（タイミングバジェット33ms）

**テスト内容**:
- 測定開始
- `VL53LX_GetMeasurementDataReady()` でポーリング
- `VL53LX_GetMultiRangingData()` でデータ取得
- 距離・信号強度・ステータス表示

**期待結果**:
- 約33msごとに測定完了
- 距離データ: 0～3000mm
- RangeStatus = 0（有効）

---

### 🔲 Stage 5: 割り込み測定（未実装）

**目的**: GPIO割り込みによる効率的なデータ取得

**テスト内容**:
- GPIO割り込みハンドラ登録
- FreeRTOSセマフォ使用
- `VL53LX_ClearInterruptAndStartMeasurement()`

**利点**:
- CPU使用率低減
- リアルタイム性向上
- 消費電力削減

---

### 🔲 Stage 6: 2センサー統合（未実装）

**目的**: 前方・底面センサーの同時動作

**テスト内容**:
- XSHUTシーケンス制御
- I2Cアドレス変更（底面: 0x29 → 0x30）
- 2つのセンサー独立測定
- 個別割り込み処理

**アドレス変更シーケンス**:
1. 両方のXSHUT → LOW（スタンバイ）
2. 底面XSHUT → HIGH
3. 底面のアドレスを0x30に変更
4. 前方XSHUT → HIGH（0x29で動作）

---

## 実装の進め方

1. **Stage 1から順番に実装・テスト**
   各ステージで動作確認してから次に進むことを推奨

2. **ハードウェアでの動作確認**
   各ステージのビルド後、実機でテストして期待通りの結果が得られることを確認

3. **問題が発生した場合**
   各サンプルのREADME.mdを参照し、トラブルシューティングを実施

4. **カスタマイズ**
   サンプルコードをベースに、自分のアプリケーションに合わせて改良

## トラブルシューティング

### ビルドエラー

- `stampfly_tof_config.h not found`
  → コンポーネントが正しくコピーされているか確認

- `driver/i2c.h not found`
  → ESP-IDF環境が正しくセットアップされているか確認

### 実行時エラー

- I2C初期化失敗
  → GPIO番号が正しいか確認
  → 他のペリフェラルとの競合がないか確認

- デバイス未検出
  → I2C配線確認
  → プルアップ抵抗確認（1.8kΩ～4.7kΩ）
  → センサー電源確認
  → XSHUT制御確認

## 次のステップ

以下の順で進めてください：

1. ✅ Stage 1実行 → デバイス検出確認
2. ✅ Stage 2実行 → レジスタ読み出し確認
3. 🔲 Stage 3実装 → API初期化確認
4. 🔲 Stage 4実装 → 距離測定動作確認
5. 🔲 Stage 5実装 → 割り込み動作確認
6. 🔲 Stage 6実装 → 2センサー統合確認

各ステージの詳細な実装計画は、プロジェクトルートの [README.md](../README.md) を参照してください。
