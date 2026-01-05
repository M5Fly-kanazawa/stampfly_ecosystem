# StampFly ハードウェア仕様書

このドキュメントは、StampFly（ESP32-S3ベースのドローン）のハードウェア仕様とピン配置をまとめたものです。

## 基本仕様

### メインコントローラー
- **MCU**: M5StampS3 (ESP32-S3FN8)
- **CPU**: デュアルコア Xtensa LX7 @ 240 MHz
- **Flash**: 8 MB
- **PSRAM**: 2 MB

### サイズと重量
- **サイズ**: 81.5 × 81.5 × 31 mm
- **重量**: 36.8 g

## センサー構成

### IMU (慣性計測装置)
- **BMI270**: 6軸ジャイロ・加速度センサー
- **更新頻度**: 400Hz (2.5ms毎)
- **インターフェース**: I2C

### 磁気センサー
- **BMM150**: 3軸磁力計
- **インターフェース**: I2C

### 気圧センサー
- **BMP280**: 気圧・温度センサー
- **インターフェース**: I2C

### 距離センサー
- **VL53L3**: ToF距離センサー × 2個
- **インターフェース**: I2C

### オプティカルフローセンサー
- **PMW3901MB-TXQT**: 光学式モーションセンサー
- **インターフェース**: SPI
- **製造元**: PixArt

## ピン配置

### SPI通信ピン (PMW3901用)
```
MISO (Master In Slave Out):  GPIO 43
MOSI (Master Out Slave In):  GPIO 14
SCLK (Serial Clock):         GPIO 44
CS   (Chip Select):          GPIO 12  (注: GPIO 46はBMI270 IMUのCS)
```

### I2C通信ピン (IMU、磁気、気圧、距離センサー用)
```
SDA (Serial Data):           GPIO 3
SCL (Serial Clock):          GPIO 4
```

### その他のGPIO
- **WS2812 RGB LED**: GPIO 38
- **ブザー**: (ピン番号は要確認)
- **リセットボタン**: ハードウェアリセット

## PMW3901 オプティカルフローセンサー詳細

### 通信プロトコル
- **インターフェース**: SPI
- **最大クロック速度**: 2 MHz
- **データ長**: 8ビット
- **タイミング**: 各SPI転送後に200μs遅延が必要

### レジスタ情報
- **チップID (0x00)**: 0x49 (検証用)
- **インバースID (0x5F)**: 0xB8 (検証用)
- **モーションデータ**: 16ビット符号付き整数
  - Delta X Low: 0x03
  - Delta X High: 0x04
  - Delta Y Low: 0x05
  - Delta Y High: 0x06

### 初期化シーケンス
1. CS ピンのトグル (LOW→HIGH→LOW→HIGH、各1ms遅延)
2. 45ms 待機
3. ソフトリセット (レジスタ0x3A に 0x5A を書き込み)
4. レジスタバンクを切り替えながら60以上のレジスタを設定
5. チップIDとインバースIDを検証

### モーションデータ読み取り
- **更新頻度**: 約100Hz推奨
- **データ形式**: int16_t (X, Y)
- **読み取り方法**: レジスタ0x03-0x06から4バイトを連続読み取り

### 速度計算式
```
velocity_x = -(0.0254 * deltaX * Altitude / 11.914) / opt_interval
velocity_y = -(0.0254 * deltaY * Altitude / 11.914) / opt_interval
```
- `deltaX`, `deltaY`: センサーから読み取った移動量
- `Altitude`: 高度 [m]
- `opt_interval`: サンプリング間隔 [秒]
- 結果の単位: [m/s]

### フレームキャプチャモード
- **解像度**: 35×35ピクセル
- **データ形式**: 8ビットグレースケール
- **読み取り**: レジスタ0x59でステータス確認後、0x58から読み取り

## Grove コネクタ
- **数**: 2個
- **用途**: 周辺機器拡張用
- **ピン配置**: 要確認

## 電源
- **バッテリー**: BETA FPV製 1S 300 mAh LiPo
- **充電**: 専用充電器使用

## モーター
- **タイプ**: 高速コアレスモータ
- **数**: 4個
- **制御**: PWM

## 通信
- **デフォルト**: ESP-NOW (Atom Joystick との通信用)
- **その他**: WiFi, Bluetooth (ESP32-S3の機能として利用可能)

## 参考リポジトリ
- M5Stack公式: https://github.com/m5stack/M5StampFly
- M5Fly金沢: https://github.com/M5Fly-kanazawa/StampFly
- 開発版 (hakodate ブランチ): https://github.com/kouhei1970/StampFly_sandbox/tree/hakodate

## 注意事項
1. PMW3901のSPI通信は各転送後に200μs以上の遅延が必要
2. I2Cバスには複数のセンサーが接続されているため、I2Cアドレスの競合に注意
3. バッテリーの膨張・過充電に注意
4. プロペラ検査時は必ず保護メガネを着用

---
更新日: 2025-11-14
ベース情報元: StampFly_sandbox (hakodate branch) のソースコード解析
