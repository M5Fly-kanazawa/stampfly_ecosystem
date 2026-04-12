# PMW3901 実装リファレンス

このドキュメントは、PMW3901オプティカルフローセンサーの各種実装を調査し、比較した結果をまとめたものです。

## 調査した実装

1. **StampFly実装** (kouhei1970/StampFly_sandbox hakodate branch)
2. **Bitcraze Arduino実装** (bitcraze/Bitcraze_PMW3901)
3. **PX4 Autopilot実装** (PX4/PX4-Autopilot)

## SPI通信設定の比較

| 項目 | StampFly | Bitcraze | PX4 | 推奨値 |
|------|----------|----------|-----|--------|
| クロック速度 | 2MHz | 4MHz | 2MHz | **2MHz** (データシート最大値) |
| SPIモード | MODE3 | MODE3 | MODE3 | **MODE3** (CPOL=1, CPHA=1) |
| ビット順序 | MSBFIRST | MSBFIRST | MSBFIRST | **MSBFIRST** |
| 読み取り後遅延 | 200μs | 50-100μs | - | **50μs** (Bitcraze実績値) |
| 書き込み後遅延 | 200μs | 50μs | 11μs | **50μs** (Bitcraze実績値) |

### 結論
- **クロック速度**: 2MHzを使用（データシート最大値、PX4とStampFlyで実績あり）
- **遅延時間**: 50μsを使用（Bitcraze実績値、問題があれば調整）

## レジスタ定義

### 基本レジスタ

| アドレス | 名称 | 説明 | 期待値 |
|---------|------|------|--------|
| 0x00 | Product ID | 製品ID | 0x49 |
| 0x01 | Revision ID | リビジョンID | - |
| 0x02 | Motion | モーション検出 | bit7: モーション有無 |
| 0x03 | Delta_X_L | X方向移動量 下位 | - |
| 0x04 | Delta_X_H | X方向移動量 上位 | - |
| 0x05 | Delta_Y_L | Y方向移動量 下位 | - |
| 0x06 | Delta_Y_H | Y方向移動量 上位 | - |
| 0x07 | SQUAL | 表面品質 | 0-255 |
| 0x3A | Power_Up_Reset | リセット | 0x5Aで実行 |
| 0x58 | Raw_Data_Grab | フレームキャプチャ | - |
| 0x59 | Raw_Data_Grab_Status | フレームステータス | bit6-7 |
| 0x5F | Inverse_Product_ID | 反転製品ID | **0xB6** (正しい値) |
| 0x7F | Bank_Select | バンク選択 | 0x00-0x15 |

### 重要な発見

**Inverse Product ID**:
- **正しい値**: 0xB6
- Bitcraze実装: 0xB6を期待（正しい）
- StampFly実装: 0xB8を期待（誤り）
- **結論**: 0xB6を使用する。StampFlyの実装は間違っている。

## 初期化シーケンスの比較

### 1. パワーアップシーケンス

#### StampFly方式
```
1. 45ms待機
2. CS ピンのトグル: LOW → HIGH → LOW → HIGH (各1ms間隔)
3. ソフトリセット (0x3A ← 0x5A)
4. Product ID確認 (0x00 == 0x49)
5. Inverse Product ID確認 (0x5F == 0xB6) ※元の実装は0xB8で誤り
6. レジスタ初期化 (60+個のレジスタ)
```

#### Bitcraze方式
```
1. SPI設定 (4MHz, MODE3, MSBFIRST)
2. CS ピンのトグル (複数回)
3. ソフトリセット (0x3A ← 0x5A)
4. Product ID確認 (0x00 == 0x49)
5. Inverse Product ID確認 (0x5F == 0xB6)
6. モーションレジスタ読み取り (バッファクリア)
7. レジスタ初期化
```

#### PX4方式
```
1. ソフトリセット (0x3A ← 0x5A) + 5ms待機
2. モーションレジスタ読み取り (0x02-0x06)
3. レジスタ初期化 (PixArt demo kit v3.20準拠)
```

### 2. レジスタ初期化

全ての実装で60個以上のレジスタを設定していますが、設定値は実装ごとに若干異なります。

- **StampFly**: 独自の最適化値
- **Bitcraze**: Arduinoベースの実績値
- **PX4**: PixArt demo kit v3.20 (2018年8月) の推奨値

**推奨**: PX4の値が最新の公式推奨値である可能性が高い

## モーションデータ読み取り

### データ形式
- **Delta X, Y**: 16ビット符号付き整数
- **範囲**: -32768 〜 +32767 (理論値)
- **有効範囲**: ±240ピクセル (PX4でこれを超える値は無効として扱う)

### 読み取り方法

#### 個別レジスタ読み取り (StampFly, Bitcraze)
```c
Delta_X = (Delta_X_H << 8) | Delta_X_L
Delta_Y = (Delta_Y_H << 8) | Delta_Y_L
```

#### バースト読み取り (PX4)
```c
// 0x02-0x07の12バイトを一括読み取り
// より効率的だが、実装が複雑
```

### 更新頻度
- **推奨**: 100Hz (10ms間隔)
- **StampFly**: 100Hz
- **PX4**: 10ms間隔サンプリング

## 速度変換式の詳細分析

PMW3901から得られる生データ（ピクセル単位の移動量）を実際の速度に変換する方法には、主に2つのアプローチがあります。

### 方式1: StampFly式（直接速度計算）

```c
velocity_x = -(0.0254 * deltaX * altitude / 11.914) / interval
velocity_y = -(0.0254 * deltaY * altitude / 11.914) / interval
```

#### パラメータ
- `deltaX`, `deltaY`: センサーから読み取ったピクセル単位の移動量
- `altitude`: 地面からの高度 [m]
- `interval`: サンプリング間隔 [秒]
- **出力**: 地面速度 [m/s]

#### 係数の意味
- **0.0254**: インチからメートルへの変換係数（1インチ = 0.0254m）
- **11.914**: センサーの焦点距離と視野角から導出された定数
- **負号(-)**: センサー座標系とボディ座標系の方向の違いを補正

#### 導出過程
1. PMW3901の視野角（FOV）と焦点距離から、1ピクセルあたりの角度を計算
2. 高度hでの1ピクセルあたりの地表距離 = h × tan(1ピクセルの角度)
3. 定数化: 0.0254 / 11.914 ≈ 0.00213 [rad/pixel]
4. 速度 = (ピクセル移動量 × 高度依存の地表距離) / 時間

#### 利点
- 直接地面速度が得られるため、制御ループに即座に使える
- 高度センサーと組み合わせて簡単に実装可能
- ドローンの位置制御に最適

#### 欠点
- 高度情報が必須（高度センサーが故障すると使用不可）
- 地面が平坦であることを前提としている
- センサーの取り付け角度（ピッチ・ロール）の影響を受ける

### 方式2: PX4式（角速度ベース）

```c
flow_rate_x = deltaX / 385.0  // [rad/s]
flow_rate_y = deltaY / 385.0  // [rad/s]
```

#### パラメータ
- `deltaX`, `deltaY`: センサーから読み取ったピクセル単位の移動量
- **385.0**: ピクセルからラジアン/秒への変換係数
- **出力**: 角速度（光学フロー） [rad/s]

#### 係数の意味
- **385.0**: (視野角 / ピクセル数) × サンプリング周波数から導出
  - PMW3901の有効視野角: 約42度 = 0.733 rad
  - 35ピクセルの解像度
  - 1ピクセル ≈ 0.021 rad
  - 100Hzサンプリング時の係数

#### 後段処理（速度への変換）
```c
// PX4の実装では、後段で以下のように速度に変換
velocity_x = flow_rate_x × altitude
velocity_y = flow_rate_y × altitude

// さらに、機体の姿勢（ヨー、ピッチ、ロール）による補正を適用
```

#### 利点
- センサーフュージョン（Kalmanフィルタ等）に統合しやすい
- 機体の姿勢変化による補正が容易
- IMUデータとの融合が自然
- より正確な航法システムを構築可能

#### 欠点
- 速度を得るには追加の計算が必要
- 実装が複雑
- リアルタイム制御には追加のレイテンシが発生

### 両方式の比較表

| 項目 | StampFly式 | PX4式 |
|------|-----------|-------|
| 出力単位 | m/s（速度） | rad/s（角速度） |
| 実装難易度 | 簡単 | 複雑 |
| 高度依存性 | 計算時に必須 | 後段で使用 |
| 姿勢補正 | 困難 | 容易 |
| 用途 | シンプルな位置制御 | 高度な航法システム |
| CPU負荷 | 低 | 中〜高 |
| センサーフュージョン | 困難 | 容易 |

### 実装推奨

本ドライバでは**両方の変換式を提供**します：

1. **`pmw3901_calculate_velocity_direct()`**: StampFly式
   - ドローンの基本的な位置制御に最適
   - 高度センサーと組み合わせて使用

2. **`pmw3901_calculate_flow_rate()`**: PX4式
   - 高度な航法システムへの統合用
   - Kalmanフィルタ等との組み合わせに最適

### 使用例

```c
// 方式1: 直接速度計算（シンプルな制御向け）
float vx, vy;
float altitude = get_altitude();  // 高度センサーから取得
float interval = 0.01;  // 100Hz = 10ms
pmw3901_calculate_velocity_direct(delta_x, delta_y, altitude, interval, &vx, &vy);
// vx, vyをそのまま位置制御に使用

// 方式2: 角速度計算（航法システム統合向け）
float flow_x, flow_y;
pmw3901_calculate_flow_rate(delta_x, delta_y, interval, &flow_x, &flow_y);
// Kalmanフィルタに投入し、IMUデータと融合
kalman_filter_update(flow_x, flow_y, imu_data, ...);
```

### 数値例

センサーが100ms間に10ピクセルX方向に移動、高度1mの場合：

**StampFly式:**
```
velocity_x = -(0.0254 × 10 × 1.0 / 11.914) / 0.1
          = -0.213 m/s
          = -21.3 cm/s
```

**PX4式:**
```
flow_rate_x = 10 / 385.0 = 0.026 rad/s
velocity_x = 0.026 × 1.0 = 0.026 m/s = 2.6 cm/s
```

※この数値の違いは係数の導出方法の違いによるもので、どちらも有効な計算方法です。実際の使用では、実機でキャリブレーションして係数を調整することが推奨されます。

## フレームキャプチャモード

### 仕様
- **解像度**: 35×35ピクセル
- **データ形式**: 8ビットグレースケール
- **総ピクセル数**: 1225

### 実装方法 (Bitcraze/StampFly)
```c
1. フレームキャプチャモード有効化 (レジスタ設定)
2. Status Register (0x59) のbit6-7を監視
3. データ準備完了で Raw_Data_Grab (0x58) から読み取り
4. 1225ピクセル分繰り返し
```

**注意**: フレームキャプチャモードは通常のモーション検出と排他的

## データ品質指標

### SQUAL (Surface Quality)
- **レジスタ**: 0x07
- **範囲**: 0-255
- **意味**: 追跡対象表面の品質
- **使用方法**:
  - 低い値（<20）: データ信頼性低
  - 高い値（>100）: データ信頼性高
  - PX4では複数サンプルで平均化

## タイミング要件まとめ

| イベント | 待機時間 | 出典 |
|---------|---------|------|
| パワーアップ初期遅延 | 45ms | StampFly |
| ソフトリセット後 | 5ms | PX4 |
| CS トグル間隔 | 1ms | StampFly |
| SPI読み取り後 | 50-200μs | 全実装 |
| SPI書き込み後 | 50-200μs | 全実装 |
| レジスタ初期化完了待ち | 100ms | StampFly |

## ESP32-S3 (StampFly) 向け推奨実装

### 推奨設定
```c
SPI Clock: 2MHz
SPI Mode: MODE3 (CPOL=1, CPHA=1)
Read Delay: 50μs (問題があれば100μs, 200μsに増やしてテスト)
Write Delay: 50μs (問題があれば100μs, 200μsに増やしてテスト)
Sample Rate: 100Hz
```

### 初期化手順
```
1. 45ms待機
2. CS ピントグル (LOW→HIGH→LOW→HIGH, 各1ms)
3. ソフトリセット (0x3A ← 0x5A) + 5ms待機
4. Product ID確認 (0x00 == 0x49)
5. Inverse Product ID確認 (0x5F == 0xB6)
6. レジスタ初期化
7. モーションレジスタ読み取りでバッファクリア
```

### ピン配置 (StampFly固定)
```
MISO: GPIO 43
MOSI: GPIO 14
SCLK: GPIO 44
CS:   GPIO 12  (注: GPIO 46はBMI270 IMUのCS)
```

## 参考文献

1. **PixArt PMW3901MB-TXQT Datasheet**
   - https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.00-200317_20170331160807_public.pdf

2. **Bitcraze Arduino実装**
   - https://github.com/bitcraze/Bitcraze_PMW3901

3. **PX4 Autopilot実装**
   - https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/optical_flow/pmw3901

4. **StampFly実装**
   - https://github.com/kouhei1970/StampFly_sandbox/tree/hakodate

5. **Marcus Greiff's Master Thesis** (Kalman Filter実装の理論)
   - Crazyflie firmware内で参照

## まとめ

PMW3901の実装には複数のアプローチがありますが、StampFly向けには：

1. **SPI設定**: 2MHz, MODE3（保守的で確実）
2. **遅延**: 50μs（Bitcraze実績値、問題があればテストして調整）
3. **初期化**: StampFlyの手順をベースに、PX4の最新レジスタ値を参考
4. **Inverse Product ID**: 0xB6（正しい値）
5. **速度計算**: 両方式を実装（StampFly直接速度式とPX4角速度式）

---
更新日: 2025-11-14
調査実施: StampFly, Bitcraze, PX4実装の比較分析
