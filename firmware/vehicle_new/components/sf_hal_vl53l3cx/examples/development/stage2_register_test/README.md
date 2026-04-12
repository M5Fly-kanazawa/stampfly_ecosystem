# Stage 2: VL53L3CX Register Read/Write Test

## 概要

VL53L3CX ToFセンサーのレジスタへの直接アクセスをテストします。
VL53LXプラットフォーム層（I2C読み書き関数）の動作確認を行います。

## テスト内容

- VL53LXプラットフォーム層の初期化
- レジスタ読み出し（Model ID、Module Type）
- 期待値との比較

## 期待される結果

```
Model ID (0x010F): 0xEA [OK]
Module Type (0x0110): 0xAA [OK]
Mask Revision (0x0111): 0x?? (informational)
```

**注意**: Module Type 0xCCはVL53L1の値です。VL53L3CXの正しい値は0xAAです。

## 使用方法

### 1. このフォルダに移動
```bash
cd examples/stage2_register_test
```

### 2. ESP-IDF環境を有効化
```bash
. ~/esp/esp-idf/export.sh
```

### 3. ターゲット設定（初回のみ）
```bash
idf.py set-target esp32s3
```

### 4. ビルド
```bash
idf.py build
```

### 5. フラッシュとモニタ
```bash
idf.py flash monitor
```

## 実装の詳細

### プラットフォーム層

- **ヘッダー**: `include/vl53lx_platform.h`
- **実装**: `src/vl53lx_platform.c`

プラットフォーム層は、VL53LXドライバとESP-IDF I2C master APIの橋渡しを行います：

- `VL53LX_PlatformInit()` - I2Cデバイスの登録
- `VL53LX_ReadByte()` - 8ビットレジスタ読み出し
- `VL53LX_ReadWord()` - 16ビットレジスタ読み出し（ビッグエンディアン）
- `VL53LX_WriteByte()` - 8ビットレジスタ書き込み
- `VL53LX_WriteWord()` - 16ビットレジスタ書き込み（ビッグエンディアン）

### VL53L3CXレジスタ

| レジスタ名 | アドレス | 期待値 | 説明 |
|-----------|---------|--------|------|
| Model ID | 0x010F | 0xEA | デバイスモデル識別子 |
| Module Type | 0x0110 | 0xAA | モジュールタイプ識別子（VL53L3CX） |
| Mask Revision | 0x0111 | - | マスクリビジョン（参考情報） |

## トラブルシューティング

### レジスタ読み出しに失敗する場合

- I2C通信の問題:
  - Stage 1が正常に完了していることを確認
  - I2C配線を確認（SDA=GPIO3, SCL=GPIO4）
  - プルアップ抵抗を確認（2-5kΩ推奨）

- 期待値と異なる値が読み出される場合:
  - VL53L3CX以外のデバイスに接続されている可能性
  - I2Cアドレス(0x29)が正しいか確認
  - XSHUTピンの設定を確認（前方：HIGH、底面：LOW）

## 次のステップ

Stage 2が成功したら、Stage 3（デバイス初期化）に進めます：
- VL53LXドライバコアの統合
- デバイスブート待機
- デバイス情報取得
