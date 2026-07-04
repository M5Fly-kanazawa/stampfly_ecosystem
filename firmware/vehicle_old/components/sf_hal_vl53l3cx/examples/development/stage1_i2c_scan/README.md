# Stage 1: I2C Bus Scan - VL53L3CX Device Detection

## 目的
ESP32-S3のI2C通信が正常に動作し、VL53L3CXセンサーがバス上で検出できることを確認します。

## 動作内容
1. I2Cマスターの初期化（SDA=GPIO3, SCL=GPIO4, 400kHz）
2. XSHUT ピンの制御
   - 前方ToF（GPIO9）: HIGH（有効化）
   - 底面ToF（GPIO7）: LOW（無効化）
3. I2Cバススキャン（0x03～0x77）
4. 検出されたデバイスのアドレスを表示

## 期待される結果
```
Device found at address 0x29
  -> VL53L3CX detected at default address!
```

## 使用方法

このフォルダは独立したESP-IDFプロジェクトです。このフォルダ内で直接ビルドできます。

### 1. このフォルダに移動
```bash
cd examples/stage1_i2c_scan
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

## プロジェクト構成

```
stage1_i2c_scan/
├── CMakeLists.txt      # プロジェクトレベルのCMake
├── main/
│   ├── CMakeLists.txt  # mainコンポーネントのCMake
│   └── main.c          # メインプログラム
└── README.md           # このファイル
```

このプロジェクトは親ディレクトリの `stampfly_tof` コンポーネントを自動的に参照します。

## トラブルシューティング

### デバイスが検出されない場合
- I2C配線を確認（SDA=GPIO3, SCL=GPIO4）
- プルアップ抵抗を確認（1.8kΩ～4.7kΩ推奨）
- センサーの電源供給を確認
- XSHUT ピンが正しく設定されているか確認

### I2C初期化エラーの場合
- GPIOピン番号が正しいか確認
- 他のペリフェラルとのピン競合がないか確認

## 次のステップ
Stage 1が成功したら、Stage 2（レジスタ読み書きテスト）に進んでください。
