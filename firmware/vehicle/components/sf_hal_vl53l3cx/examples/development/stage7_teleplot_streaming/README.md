# Stage 7: VL53L3CX Teleplotストリーミング

## 概要

このステージでは、ToFセンサーからの連続測定データをTeleplot形式でストリーミングし、リアルタイム可視化を行います。

## Telep lotとは

[Teleplot](https://github.com/nesnes/teleplot-vscode)は、シリアルポート経由で送信されるデータをリアルタイムでグラフ表示するVSCode拡張機能です。

### データフォーマット

```
>variable_name:value
```

例：
```
>bottom_distance:245
>bottom_signal:15.32
>bottom_status:0
>front_distance:210
>front_signal:18.45
>front_status:0
```

## 機能

### エンドレス測定

Stage 6までは測定回数が制限されていましたが、Stage 7では無限ループで連続測定します。

```c
while (1) {
    // 割り込み待機
    if (xSemaphoreTake(semaphore, timeout) == pdTRUE) {
        // データ取得＆Teleplot出力
        printf(">bottom_distance:%u\n", distance);
        printf(">bottom_signal:%.2f\n", signal);
        printf(">bottom_status:%u\n", status);
    }
}
```

### FreeRTOSタスクによる並列測定

両センサーが有効な場合、2つの独立したタスクで同時測定します。

```c
xTaskCreate(bottom_sensor_task, "bottom_tof", 4096, NULL, 5, NULL);
xTaskCreate(front_sensor_task, "front_tof", 4096, NULL, 5, NULL);
```

### 出力変数

#### 底面ToF

| 変数名 | 説明 | 単位 |
|--------|------|------|
| `bottom_distance` | 測定距離 | mm |
| `bottom_signal` | 信号強度 | Mcps |
| `bottom_status` | 測定ステータス | - |

#### 前方ToF（オプション）

| 変数名 | 説明 | 単位 |
|--------|------|------|
| `front_distance` | 測定距離 | mm |
| `front_signal` | 信号強度 | Mcps |
| `front_status` | 測定ステータス | - |

## 電源要件

**デフォルト設定**: 底面ToFのみ有効（USB給電で動作）

```c
#define ENABLE_FRONT_SENSOR  0  // 0: Bottom only (USB), 1: Both sensors (Battery required)
```

- **`ENABLE_FRONT_SENSOR = 0`**: 底面ToFのみ（USB給電のみで動作）**← デフォルト**
- **`ENABLE_FRONT_SENSOR = 1`**: 前方+底面ToF（バッテリー必要）

## ビルド＆実行方法

### 1. プロジェクトをビルド

```bash
cd examples/stage7_teleplot_streaming
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

### 2. Teleplotをインストール

VSCodeで以下の拡張機能をインストール：
- 拡張機能ID: `alexnesnes.teleplot`

または：
1. VSCodeを開く
2. 左のサイドバーで拡張機能アイコンをクリック
3. "Teleplot"を検索
4. インストール

### 3. Teleplotで可視化

1. VSCodeでコマンドパレット（`Cmd+Shift+P` / `Ctrl+Shift+P`）を開く
2. "Teleplot: Start" を選択
3. シリアルポートを選択（例: `/dev/cu.usbserial-*` または `COM*`）
4. ボーレートを `115200` に設定
5. データがリアルタイムでグラフ表示されます

## 期待される出力

### コンソール出力（底面ToFのみ）

```
Stage 7: Teleplot Streaming
VL53L3CX ToF Sensors
==================================
I2C master initialized successfully
SDA: GPIO3, SCL: GPIO4
Starting I2C address change sequence...
  1. Both sensors shutdown
  2. Bottom sensor enabled at default 0x29
  3. Bottom sensor address changed to 0x30
  4. Front sensor DISABLED (set ENABLE_FRONT_SENSOR=1 to enable)
I2C address change sequence complete
Bottom ToF: GPIO7 (0x30) [ENABLED - USB powered]
Front ToF: GPIO9 [DISABLED]
INT pins initialized
Bottom INT: GPIO6
Initializing BOTTOM sensor...
BOTTOM: ✓ Device booted
BOTTOM: ✓ Data initialized
BOTTOM: ✓ Product Type: 0xAA, Rev: 1.1
Using default measurement parameters
==================================
Starting continuous streaming
Interrupt mode, Teleplot format
Bottom sensor only (USB powered)
==================================
Streaming tasks started. Use Teleplot to visualize data.
BOTTOM: Starting continuous measurements...
>bottom_distance:245
>bottom_signal:15.32
>bottom_status:0
>bottom_distance:247
>bottom_signal:15.28
>bottom_status:0
>bottom_distance:246
>bottom_signal:15.30
>bottom_status:0
...
```

### Teleplotグラフ表示

Teleplotでは以下のグラフが表示されます：

#### 底面ToFのみの場合
- `bottom_distance` - 距離のリアルタイムグラフ
- `bottom_signal` - 信号強度のグラフ
- `bottom_status` - ステータスのグラフ

#### 両センサー有効の場合
- `bottom_distance` / `front_distance` - 両センサーの距離
- `bottom_signal` / `front_signal` - 両センサーの信号強度
- `bottom_status` / `front_status` - 両センサーのステータス

## 前方ToFを有効にする方法

1. `main/main.c`を編集:
```c
#define ENABLE_FRONT_SENSOR  1  // 0→1に変更
```

2. **バッテリーを接続**

3. リビルド＆フラッシュ:
```bash
idf.py build flash monitor
```

4. Teleplotで6つのグラフが表示されます

## センサー構成

| センサー | XSHUT | INT | I2Cアドレス | 電源 |
|---------|-------|-----|-----------|------|
| 底面ToF | GPIO7 | GPIO6 | 0x30 | USB給電 |
| 前方ToF | GPIO9 | GPIO8 | 0x29 | バッテリー必要 |

## 測定パラメータのカスタマイズ

### タイミングバジェット変更

```c
#define TIMING_BUDGET_MS     33  // デフォルト: 33ms

// 高速測定（精度低下）
#define TIMING_BUDGET_MS     20

// 高精度測定（低速）
#define TIMING_BUDGET_MS     50
```

タイミングバジェットを変更した場合は、センサー初期化後に設定を追加：

```c
status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&bottom_dev, TIMING_BUDGET_MS * 1000);
if (status != VL53LX_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to set timing budget");
}
```

### 測定レート

現在の実装では割り込みベースのため、タイミングバジェット（約33ms）ごとに新しいデータが取得されます。

測定レート ≈ 1000ms / 33ms ≈ 30 Hz

## Telep lotの便利な機能

### 複数グラフの表示

Teleplotは自動的に各変数を個別のグラフに表示します。

### Y軸の自動スケーリング

距離データ（0-4000mm）と信号強度（0-100 Mcps）が自動的に適切なスケールで表示されます。

### データのエクスポート

Teleplotから測定データをCSV形式でエクスポートできます：
1. グラフ上で右クリック
2. "Export to CSV"を選択

### ズーム＆パン

- マウスホイール: ズーム
- ドラッグ: パン
- ダブルクリック: 自動フィット

## トラブルシューティング

### Teleplotに何も表示されない

**確認事項**:
1. 正しいシリアルポートを選択しているか
2. ボーレートが115200に設定されているか
3. ESP32からデータが送信されているか（`idf.py monitor`で確認）
4. Teleplotのフォーマット（`>variable:value`）が正しいか

### グラフがカクカクする

**原因**: 測定レートが速すぎる可能性

**対処**:
```c
// タスク内で遅延を追加
vTaskDelay(pdMS_TO_TICKS(10));  // 10ms遅延
```

### 両センサーのデータが混在する

**原因**: 正常動作です。2つのタスクが独立して動作しています。

**対処**: Teleplotは自動的に変数名で分離して表示します。

### メモリ不足エラー

```
E (xxxx) task_wdt: Task watchdog got triggered.
```

**原因**: タスクスタックサイズが不足

**対処**:
```c
// タスク作成時のスタックサイズを増やす
xTaskCreate(bottom_sensor_task, "bottom_tof", 8192, NULL, 5, NULL);  // 4096→8192
```

## 実用例

### ドローンの高度＆障害物監視

```
底面ToF: 高度測定（0-2000mm）
前方ToF: 障害物検出（0-4000mm）
```

Teleplotで両方の距離を同時にモニタリングすることで、ドローンの飛行状況を可視化できます。

### データロギング

1. Teleplotでデータを可視化
2. "Export to CSV"でデータをエクスポート
3. Excelやpythonで後処理

### 閾値監視

Telep lotで設定した閾値を超えた場合に警告を表示できます（Teleplotの機能）。

## パフォーマンス

### CPU使用率

割り込みベースのため、CPU使用率は低いままです：
- 底面ToFのみ: 約5-10%
- 両センサー: 約10-15%

### メモリ使用量

- タスクスタック: 4KB × センサー数
- VL53LX構造体: 約4KB（両センサー）
- セマフォ: 約200B × センサー数

合計: 約10-15KB

## まとめ

Stage 7では：
- ✅ Teleplot形式のリアルタイムストリーミング
- ✅ エンドレス連続測定
- ✅ FreeRTOSタスクによる並列動作
- ✅ USB給電のみで動作（底面ToF）
- ✅ グラフによる可視化

これでStampFly ToFドライバの実用的なデータ可視化が可能になりました！
