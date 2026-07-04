# Third-Party Notices for vehicle_new

vehicle_new ファームウェア本体は MIT License で配布されます（プロジェクトルートの [`LICENSE`](../../LICENSE) を参照）。本ファームウェアは以下の第三者コンポーネントを利用しており、それぞれ独自のライセンスが適用されます。

The vehicle_new firmware itself is distributed under the MIT License (see project root [`LICENSE`](../../LICENSE)). It depends on the following third-party components, each governed by its own license.

---

## 1. Bosch Sensortec センサドライバ群

### 1.1 BMI270 IMU C driver
- **コンポーネント**: `components/sf_hal_bmi270/` の C driver 部分
- **出所**: Bosch Sensortec official BMI270 SensorAPI (https://github.com/boschsensortec/BMI270_SensorAPI)
- **ライセンス**: BSD-3-Clause
- **改変**: ESP-IDF 環境向けの周辺コード追加。アルゴリズム本体は未改変

### 1.2 BMM150 Magnetometer C driver
- **コンポーネント**: `components/sf_hal_bmm150/` の C driver 部分（あれば）
- **出所**: Bosch Sensortec official BMM150 SensorAPI (https://github.com/boschsensortec/BMM150_SensorAPI)
- **ライセンス**: BSD-3-Clause

### 1.3 BMP280 Barometer C driver
- **コンポーネント**: `components/sf_hal_bmp280/` の C driver 部分（あれば）
- **出所**: Bosch Sensortec official BMP2 SensorAPI (https://github.com/boschsensortec/BMP2-Sensor-API)
- **ライセンス**: BSD-3-Clause

各 C driver の元ファイル先頭ヘッダおよび `LICENSE` ファイルに従う。本リポジトリで追加した C++ wrapper（`*_wrapper.cpp/hpp`、`bmi270_wrapper`, `bmm150`, `bmp280` 等）は MIT License で配布される。

---

## 2. STMicroelectronics VL53L3CX ToF driver

- **コンポーネント**: `components/sf_hal_vl53l3cx/src/vl53lx/` および同 `include/vl53lx/`
- **出所**: STMicroelectronics VL53L3CX BareDriver (公式)
- **ライセンス**: **GPL-2.0+ OR BSD-3-Clause（dual license）**
- **本プロジェクトでの選択**: **BSD-3-Clause を選択**（GPL の伝染を回避するため）
- C++ wrapper（`vl53l3cx_wrapper.cpp/hpp`）は MIT License

ST 公式の dual license 条項により、利用者は GPL-2.0+ または BSD-3-Clause のいずれかを選択できる。本プロジェクトは **BSD-3-Clause** を選択する。これにより：
- vehicle_new バイナリの再配布時に GPL ソース開示義務は発生しない
- 派生作品も MIT または互換ライセンスで自由に配布可能

VL53L3CX driver 自体の詳細ライセンス文は `components/sf_hal_vl53l3cx/LICENSE` を参照。

---

## 3. PixArt PMW3901 OptFlow

- **コンポーネント**: `components/sf_hal_pmw3901/`
- **元実装の参考**: PixArt PMW3901MB-TXQT データシート、Bitcraze / PX4 公式 driver の実装パターン
- **ライセンス**: MIT License（StampFly 独自実装）
- **備考**: アルゴリズムは公式データシートに準拠。Bitcraze / PX4 のコードを直接借用していない

---

## 4. Texas Instruments INA3221 Power Monitor

- **コンポーネント**: `components/sf_hal_power/`
- **元実装の参考**: Texas Instruments INA3221 データシート
- **ライセンス**: MIT License（StampFly 独自実装）

---

## 5. Espressif ESP-IDF / Managed Components

### 5.1 ESP-IDF
- **出所**: Espressif Systems (https://github.com/espressif/esp-idf)
- **ライセンス**: Apache License 2.0
- **使用方法**: ビルド時のフレームワークとして使用。ソース直接同梱なし

### 5.2 led_strip (managed component)
- **コンポーネント**: `managed_components/espressif__led_strip/`
- **出所**: ESP-IDF Component Manager 経由で自動取得
- **ライセンス**: Apache License 2.0

---

## 6. ライセンス選択の根拠（要約）

| 第三者要素 | 元ライセンス | 本プロジェクトの扱い |
|-----------|------------|--------------------|
| Bosch BMI270/BMM150/BMP280 | BSD-3-Clause | そのまま継承 |
| ST VL53L3CX | GPL-2.0+ OR BSD-3-Clause | **BSD-3-Clause を選択** |
| PixArt PMW3901 | データシート参照のみ、独自実装 | MIT |
| TI INA3221 | データシート参照のみ、独自実装 | MIT |
| ESP-IDF / led_strip | Apache 2.0 | そのまま使用 |

これらの組み合わせにより、vehicle_new バイナリ全体は **MIT + BSD-3-Clause + Apache 2.0** の互換ライセンスのみで構成され、GPL コードを含まない。

---

## 7. 起点ファームウェアからの派生

vehicle_new は M5Stack 社が公開する **M5StampFly 公式ファームウェア** (https://github.com/m5stack/M5StampFly, MIT License, Copyright (c) Kouhei Ito) を起点として、関心の分離・教育性向上・SIL 検証の観点から再設計したものである。本プロジェクトの著作者と M5StampFly 公式ファームウェアの主たる著作者は同一（Kouhei Ito）であるため、ライセンス継承上の追加義務は発生しない。

詳細は [`README.md`](README.md) の Credits / Influences 節を参照。
