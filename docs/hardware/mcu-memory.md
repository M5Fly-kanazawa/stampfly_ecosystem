# StampFly マイコンモジュールのメモリ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

StampFly のマイコンモジュール（M5StampS3。チップは ESP32-S3FN8）について、メモリの仕様と、今のファームウェア（`firmware/vehicle/`）での使われ方をまとめる。機能を足すときに「メモリが足りるか」「どこに置くか」を判断する基準とする。

事実は出どころを区別して書く。

| 出どころ | 意味 |
|----------|------|
| 実機から読み出した値 | 機体 1 台に esptool・espefuse で読み出しのみ行った結果（2026-11-21） |
| 公式資料の記述 | M5Stack・Espressif の公開文書（URL つき、閲覧 2026-11-21） |
| ビルドから | 今のファームのビルド結果（コミット 53553678、ESP-IDF v5.5.2） |
| 実機で測った値 | 動かした機体での測定（2026-11-20 と 2026-11-21） |
| 開発者の説明 | 開発者から聞いた内容（2026-11-21） |

数値の単位は元の資料のままにしている。B はバイト、KB は資料の表記のまま、KiB は 1024 B を指す（1 KiB = 1024 B）。計算で求めた値には「計算」と添える。

### 対象読者

- 機体ファームウェアに機能を足す人、処理時間を詰める人
- マイコンのメモリの余裕を確かめたい人

### 結論の要約

| 項目 | 内容 |
|------|------|
| 外付け RAM（PSRAM） | 無い（eFuse の PSRAM の欄が None。データシートの ESP32-S3FN8 の型番表も PSRAM なし） |
| フラッシュ | 8 MB をチップのパッケージに内蔵（Quad SPI） |
| 内蔵 RAM | 512 KB のうち、今のファームで起動後に空いているのは約 37 KB（実測 36,736 B） |
| メモリの置き方と時間 | 400 Hz の処理の 1 周期の時間が、置き方で大きく変わる（実験では 1359〜1367 µs から 366 µs まで） |

## 2. モジュールとチップの仕様

### 仕様の表

| 項目 | 値 | 出どころ |
|------|-----|----------|
| StampFly に載るモジュール | M5StampS3。A の付く M5StampS3A が新しい版。どちらもメモリの仕様は同じ | 開発者 |
| チップ | ESP32-S3FN8（QFN56、7×7 mm） | 公式（Espressif データシート） |
| チップの表示 | `ESP32-S3 (QFN56) (revision v0.2)`、Embedded Flash 8MB (GD)、水晶 40 MHz | 実機 |
| CPU | Xtensa 32-bit LX7 デュアルコア、最大 240 MHz | 公式 |
| CPU 周波数の設定 | 240 MHz（起動時の表示 240000000 Hz） | ビルド・実機 |
| 内蔵フラッシュ | 8 MB（Quad SPI）、周囲温度 −40〜85 °C、VDD_SPI 3.3 V | 公式 |
| 外付け RAM（PSRAM） | 無し（データシートの型番表で PSRAM 欄が「—」） | 公式 |
| ROM | 384 KB | 公式 |
| SRAM | 512 KB | 公式 |
| RTC の SRAM | 16 KB | 公式 |
| 電源回路 | 5 V から 3.3 V を作る回路を内蔵。DC-DC は MUN3CAD01-SC、入力 5 V（StampS3 の製品文書） | 公式 |

### StampS3 と StampS3A

| 観点 | 内容 | 出どころ |
|------|------|----------|
| 関係 | StampS3A が新しい版。メモリの仕様は同じ | 開発者 |
| 公式が挙げる違い | RGB LED の電源制御、アンテナ、ブートボタンの大きさ、スリープ時の消費電力（USB 給電で S3A が 88.82 µA、S3 が 400.67 µA） | 公式 |
| メモリについての公式の記述 | メモリの違いへの言及は無い。「メモリ仕様が同一」と明記した文も無い（どちらも同じ SoC・8 MB フラッシュが並ぶ） | 公式 |
| PSRAM の記載 | どちらの製品文書にも無い | 公式 |

### StampFly の版と載っているモジュール

| 版 | 製品文書の記述 | URL |
|----|----------------|-----|
| v1.0 | Stamp-S3 を搭載 | https://docs.m5stack.com/en/app/StampFly |
| v1.1 | 「uses the Stamp-S3A」 | https://docs.m5stack.com/en/products/sku/K138-V11 |

手元の機体がどちらの版かは、この文書では確かめていない。

### 参考資料

| 資料 | URL |
|------|-----|
| M5Stack 製品文書 StampS3 | https://docs.m5stack.com/en/core/StampS3 |
| StampS3 の回路図 | https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/522/Sch_M5StampS3_v0.2.pdf |
| M5Stack 製品文書 StampS3A | https://docs.m5stack.com/en/core/Stamp-S3A |
| StampS3A の回路図 | https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1150/Sch_StampS3_v0.3.3.pdf |
| M5Stack 製品文書 StampFly（v1.0） | https://docs.m5stack.com/en/app/StampFly |
| M5Stack 製品文書 StampFly（v1.1） | https://docs.m5stack.com/en/products/sku/K138-V11 |
| M5Stack ハード設計ファイル | https://github.com/m5stack/M5_Hardware |
| Espressif ESP32-S3 Series Datasheet（Version 2.2、改訂履歴 2026-03-05） | https://documentation.espressif.com/esp32-s3_datasheet_en.pdf |
| ESP-IDF 文書 Memory Types | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/memory-types.html |
| ESP-IDF 文書 Speed Optimization | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/performance/speed.html |
| ESP-IDF 文書 Minimizing RAM Usage | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/performance/ram-usage.html |

ESP-IDF 文書は stable 版の表示を読んだもので、機体のファームは v5.5.2 である。版の違いによる記述の差は確かめていない（8 章）。

## 3. 実機から読み出した値

### 読み出しの方法

| 項目 | 内容 |
|------|------|
| 日付 | 2026-11-21 |
| 対象 | 機体 1 台 |
| 方法 | esptool と espefuse の読み出しのみ（書き込み・eFuse の焼き込みはしていない） |
| 記録 | `logs/checkout/chip_info_20261121.txt`（Git の管理外） |

### 値の表

| 項目 | 値 |
|------|-----|
| チップ | ESP32-S3 (QFN56) (revision v0.2) |
| 機能 | WiFi, BLE, Embedded Flash 8MB (GD) |
| 水晶 | 40 MHz |
| フラッシュ ID | Manufacturer c8、Device 4017 |
| 検出したフラッシュの大きさ | 8 MB |
| eFuse のフラッシュの設定 | quad（4 データ線）、3.3 V |
| eFuse FLASH_CAP / FLASH_VENDOR / FLASH_TEMP | 8M / GD / 85C |
| eFuse PSRAM_CAP / PSRAM_VENDOR / PSRAM_TEMP | None / None / None |
| PSRAM_CAPACITY（計算値） | 0 |
| PKG_VERSION / WAFER_VERSION_MAJOR / MINOR / BLK_VERSION_MINOR | 0 / 0 / 2 / 3 |
| 起動時の表示（ESP-IDF v5.5.2、ファーム 53553678） | chip revision v0.2、SPI Flash Size 8MB、cpu freq 240000000 Hz、detected chip gd、flash io dio |

eFuse とは、チップ内の 1 回だけ書き込める記憶素子のことである。

### 「外付け RAM が無い」と言える根拠

| 根拠 | 内容 | 出どころ |
|------|------|----------|
| eFuse の PSRAM の欄 | PSRAM_CAP、PSRAM_VENDOR、PSRAM_TEMP がすべて None、PSRAM_CAPACITY（計算値）が 0 | 実機 |
| データシートの型番表 | ESP32-S3FN8 は In-Package Flash が 8 MB（Quad SPI）、In-Package PSRAM が「—」 | 公式 |
| ビルドの設定 | CONFIG_SPIRAM は未設定 | ビルド |

実機の eFuse と、公式の型番表の記述が一致している。読み出したのは 1 台だけである。

## 4. メモリの構成

### 内蔵メモリ（データシート）

| 種類 | 大きさ | 役割 |
|------|--------|------|
| ROM | 384 KB | 書き換えられない記憶領域 |
| SRAM | 512 KB | 変数やコードを置く RAM |
| RTC の SRAM | 16 KB | RTC 用の SRAM |

SRAM（Static RAM）とは、チップの中にある高速な RAM のことである。

### 命令キャッシュとデータキャッシュ

キャッシュとは、フラッシュ上のコードやデータを、よく使うものだけ手前の高速なメモリに写しておく仕組みのことである。

| 項目 | 内容 | 出どころ |
|------|------|----------|
| 構成 | 命令キャッシュとデータキャッシュがあり、2 つの CPU コアで共有する | 公式 |
| 命令キャッシュの大きさの選択肢 | 16 KB または 32 KB | 公式 |
| データキャッシュの大きさの選択肢 | 32 KB または 64 KB | 公式 |
| 今の設定（命令） | 16 KB（0x4000、8 way、ライン 32 B） | ビルド |
| 今の設定（データ） | 32 KB（0x8000） | ビルド |

### データとコードで同じ RAM を分け合う

ESP-IDF の文書は次のように述べている（英語の原文）。

- 「Any internal SRAM which is not used for Instruction RAM will be made available as DRAM」
- 「The maximum statically allocated DRAM size is reduced by the IRAM size of the compiled application.」
- 「Increasing the cache size will also result in reducing the amount of available RAM.」（キャッシュを大きくすると使える RAM が減る）

DRAM はデータを置く RAM、IRAM は命令（コード）を置く RAM のことである。IRAM に置いたコードの分だけ、データに使える RAM が減る。

### フラッシュの区画（パーティション表）

パーティション表とは、フラッシュの中を用途ごとに区切った一覧のことである。

| 区画 | 開始 | 大きさ | 用途 |
|------|------|--------|------|
| nvs | 0x9000 | 0x6000（24 KB） | — |
| phy_init | 0xF000 | 0x1000 | — |
| factory | 0x10000 | 0x300000（3 MB） | アプリ（ファーム本体） |
| storage（spiffs） | 0x310000 | 0x200000（2 MB） | spiffs（フラッシュ上のファイル領域） |

表の最後の区画 storage の終わりは 0x510000（計算）である。それより後ろの使い方は確かめていない。

### 内蔵 RAM の使われ方の概観

数値は 5 章の表（ビルド）による。帯の長さは DIRAM 合計 341,760 B に対する割合（計算）の概略である。

```
DIRAM 341,760 B（データとコードで共用する領域）
|-----------|---|-----------|---------------|
  .bss        .data  .text       残り
  固定の変数  初期値  RAM に置いた  動的な確保に
  93,688 B    つき    コード        回る領域（地図上）
  (27.4 %)    変数    91,451 B     132,917 B
              23,704 B (26.8 %)    (38.9 %)
              (6.9 %)
```

「残り 132,917 B」はビルド時の地図上の値で、起動後にヒープ（実行中に確保できるメモリの置き場）として実測した空き 36,736 B（6 章）とは測り方が異なる。この文書では 2 つを引き算で結び付けていない。

## 5. 今のファームウェアでの使われ方

### ビルドからの内訳

コミット 53553678、ESP-IDF v5.5.2、`idf_size.py` の表示による。

| 領域 | 合計 | 使用 | 残り | 内訳 |
|------|------|------|------|------|
| DIRAM（データとコードで共用） | 341,760 B | 208,843 B（61.11 %） | 132,917 B | .bss 93,688 B、.text 91,451 B、.data 23,704 B |
| IRAM（コード専用） | 16,384 B | 16,384 B | 0 B（計算） | .text 15,356 B、.vectors 1,028 B |
| RTC SLOW | 8,192 B | 32 B | — | — |
| RTC FAST | 8,192 B | 24 B | — | — |
| フラッシュ（アプリ） | 3 MB（factory 区画） | 約 1.2 MB（バイナリ） | — | コード .text 856,080 B、データ .rodata 239,244 B |

.bss とは起動時にゼロで初期化される固定の変数の領域、.data とは初期値つきの固定の変数の領域のことである。

### .bss と .data の多いライブラリ

| ライブラリ | .bss | .data |
|-----------|------|-------|
| libsf_app_hooks.a | 32,470 B | 1,392 B |
| libmain.a | 30,022 B | 2,123 B |
| libsf_core.a | 14,683 B | 436 B |
| libnet80211.a（WiFi） | 7,606 B | 938 B |
| liblwip.a | 2,470 B | 12 B |
| libpp.a（WiFi） | 1,242 B | 2,640 B |
| libfreertos.a | 757 B | 3,106 B |
| libhal.a | 4 B | 3,425 B |
| libphy.a | 86 B | 3,272 B |
| libspi_flash.a | 24 B | 2,148 B |

### タスクのスタックの設定値

スタックとは、関数呼び出しの途中経過や局所変数を置く、タスクごとの作業領域のことである。タスクとは、FreeRTOS（組み込み向けの OS）が並行して動かす処理の単位のことである。

`firmware/vehicle/main/config.hpp` の `STACK_*`。

| タスク | 設定値 |
|--------|--------|
| IMU | 16,384 B |
| CONTROL | 8,192 B |
| STATE | 4,096 B |
| OPTFLOW | 8,192 B |
| MAG | 8,192 B |
| BARO | 8,192 B |
| COMM | 4,096 B |
| TOF | 8,192 B |
| TELEMETRY | 4,096 B |
| POWER | 4,096 B |
| BUTTON | 4,096 B |
| NOTIFY | 4,096 B |
| CLI（コンソールの入力を読むタスク） | 8,192 B |
| LOG | 6,144 B |
| API | 6,144 B |
| TELLO_STATE | 4,096 B |
| 合計（計算） | 106,496 B（104 KiB） |

ESP-IDF 側のスタックの設定値は、main タスク 3,584 B、esp_timer 3,584 B、イベント 4,096 B、lwIP の TCP/IP 3,072 B、IPC 1,280 B（コアごと）、アイドル 1,536 B（コアごと）である。上の合計には含めていない。

### RAM にコードを置く設定が有効な ESP-IDF の機能

| 設定 | 値 |
|------|-----|
| CONFIG_ESP_WIFI_IRAM_OPT | y |
| CONFIG_ESP_WIFI_RX_IRAM_OPT | y |
| CONFIG_FREERTOS_IN_IRAM | y |
| CONFIG_ESP_SYSTEM_IN_IRAM | y |
| CONFIG_SPI_MASTER_IN_IRAM | y |
| CONFIG_ESP_TIMER_IN_IRAM | y |

自前のコードはすべてフラッシュ上にあり、`IRAM_ATTR` の指定は 0 件である。WiFi の受信バッファの設定は、静的 10 個、動的受信 32 個、動的送信 32 個である。

### 起動時のヒープの領域

基準のファームの起動時の表示（実機）による。

| 開始アドレス | 大きさ | 種別の表示 |
|-------------|--------|-----------|
| 3FCBB090 | 0x2E680（185 KiB） | RAM |
| 3FCE9710 | 0x5724（21 KiB） | RAM |
| 3FCF0000 | 0x8000（32 KiB） | DRAM |
| 600FE000 | 0x1FE8（7 KiB） | RTCRAM |

4 領域の合計は 245 KiB（計算。表示の丸めのため厳密ではない）である。

### 起動直後の約 245 KiB と起動後の約 37 KB

| 量 | 値 | 出どころ |
|-----|-----|----------|
| 起動直後のヒープの合計 | 約 245 KiB = 約 250,880 B（計算） | 実機の起動表示から計算 |
| 起動 10 s 後の空き | 36,736 B（約 35.9 KiB、計算） | 実機で測定（6 章） |
| 差 | 約 214,144 B（約 209 KiB、計算） | 計算 |
| スタックの設定値の合計 | 106,496 B（5 章の表から計算）。差の約 5 割（計算） | ビルド |

差の行き先の候補としてスタックの設定値の合計を並べた。この文書で確かめていないことは次のとおり。

- スタックが起動時のヒープから確保されること
- 245 KiB の中の RTCRAM 7 KiB が、測定した空きの数え方に含まれるか
- WiFi などの動的な確保の内訳（測っていない。未確認）

## 6. 実機で測った空き

### 測り方

| 項目 | 内容 |
|------|------|
| 日付 | 2026-11-21 |
| 状態 | 床の上、ARM していない、電池あり、全センサ動作、USB コンソールのみ接続 |
| 測る時刻 | 起動 10 s 後と 40 s 後 |
| 値の意味 | `status` コマンドの `heap` 行。MALLOC_CAP_INTERNAL と MALLOC_CAP_8BIT を満たす内蔵 RAM の空き |
| 注意 | `heap` 行は実験ブランチ exp/imu-cycle-budget-20261120 のコミット 3b8b0212 で追加した。fix/flight-safety にはまだ無い |

### 4 つのファームの結果

| ファーム | 空き（10 s / 40 s） | 起動以降の最小 | 最大の連続した塊 | 起動時の最大のヒープ領域 | コンソール |
|----------|--------------------|----------------|------------------|--------------------------|-----------|
| 基準（53553678 + heap 行） | 36,736 / 36,776 B | 34,116 B | 23,552 B | 185 KiB | 動く |
| 命令キャッシュ 32 KB | 20,420 / 20,388 B | 18,292 B | 10,240 B | 169 KiB | 動く |
| ImuTask と ESKF のコードを IRAM に置く（+19.3 KB、.iram0.text 106,807 から 126,087 B） | 17,616 / 17,552 B | 15,488 B | 7,680 B | 166 KiB | 動く |
| 上の 2 つを同時 | 読めず | — | — | 150 KiB | コンソールのタスクを作れず初期化に失敗（`console REPL init failed: ESP_FAIL`） |

別の試行（2026-11-20）の結果は次のとおり。

| 置いたコード | 結果 |
|-------------|------|
| IRAM に +30.9 KB | コンソールの初期化に失敗 |
| IRAM に +66.8 KB | `esp_wifi_init` が ESP_ERR_NO_MEM で失敗し、再起動を繰り返した |

### 追加で使える量の目安

| 比較 | 計算 |
|------|------|
| 基準の空き（10 s）と、命令キャッシュ 32 KB の空き（10 s）の差 | 36,736 − 20,420 = 16,316 B |
| 基準の空き（10 s）と、IRAM に +19.3 KB 置いた空き（10 s）の差 | 36,736 − 17,616 = 19,120 B |

事実の並べ方は次のとおり。

- 基準から約 16 KB 減る変更（キャッシュ 32 KB）と、約 19 KB 減る変更（IRAM に +19.3 KB）は、それぞれ単独ではコンソールを含めて動いた。
- IRAM に +30.9 KB 置いた変更は、コンソールの初期化に失敗した。
- 上の 2 つを同時に行った変更は、コンソールの初期化に失敗した。

この測定から「追加できる量の上限」は決めていない。測っていない点が残っており、数値で言い切れる範囲を超える。

### 未確認

| 項目 | 状態 |
|------|------|
| WiFi の CLI（TCP 23）や `sf log wifi` の取得をつないだときの空き | 未確認 |
| ARM 中（飛行中の状態）の空き | 未確認 |
| 各タスクのスタックの実際の使用量 | 未確認 |

## 7. メモリの置き方と処理時間

**状態: 検討中。** 次の表は実験の結果であり、採用するかどうかは決まっていない。

### 命令キャッシュに収まらないコード

| 項目 | 内容 |
|------|------|
| 1 周期に必ず実行するコードの大きさ | ImuTask 約 8.8 KB、ESKF 約 8 KB。ほかに制御・ドライバ・数学関数 |
| 命令キャッシュ（両コア共有） | 16 KB |
| 判断 | 上のコードの合計が 16 KB の命令キャッシュに収まらない |
| 自前のコードの置き場所 | すべてフラッシュ上（`IRAM_ATTR` の指定は 0 件） |

ESKF とは、機体の姿勢・位置を推定するフィルタ（誤差状態カルマンフィルタ）のことである。

### 置き方を変えたときの実測

| 条件 | ImuTask の 1 周期の処理時間の平均 | 予算超えの周期 | 補足 |
|------|-----------------------------------|----------------|------|
| 基準 | 1359〜1367 µs（4 系列） | 0.69〜1.45 % | |
| 命令キャッシュ 32 KB | 693 / 701 µs（2 系列） | 0 % | |
| ImuTask と ESKF を IRAM | 483 µs | 0 % | 制御タスクの起床（待ち状態から実行可能になること）の遅れ 0 % |
| 両方 | 366 µs | 記載なし | コンソールは起動しない |

測定条件は、2026-11-20、床の上、ARM していない、ハブ経由の USB である。ImuTask は 400 Hz で動き、1 周期の予算は 2500 µs（400 Hz の周期は 1/400 s = 2500 µs で、計算）である。詳細の記録は Git の管理外（`logs/checkout/`）にある。実験スイッチは実験ブランチのコミット f6db9203 から 3b8b0212 にある。

### 速度とメモリの引き換え

ESP-IDF の文書は次のように述べている。

- 「IRAM is a limited resource, and using more IRAM may reduce available DRAM」（IRAM は限られた資源で、多く使うと使える DRAM が減りうる）
- 「Increasing the cache size will also result in reducing the amount of available RAM.」

6 章の空きと本章の処理時間を合わせると、処理時間が短くなる置き方ほど、起動後の空きが減っている。

## 8. 未確認の事項と次に確かめること

| 項目 | 状態 | 次に確かめること |
|------|------|------------------|
| タスクのスタックの実際の使用量 | 未確認 | 各タスクのスタックの最小の空き（使用量の最大値）を実機で読む |
| WiFi 接続時（TCP 23、`sf log wifi`）の空き | 未確認 | 接続した状態で `status` の `heap` 行を読む |
| ARM 中の空き | 未確認 | ARM した状態の空きを読む（飛行中の有線操作は避け、WiFi 経由で読む） |
| 動的な確保の内訳（WiFi など） | 未確認（測っていない） | ヒープの確保元ごとの量を測る |
| 5 章の「差」にスタックが含まれること | 未確認 | スタックの確保先を文書とコードで確かめる |
| 測定した空きに RTCRAM 7 KiB が含まれるか | 未確認 | `heap` 行の対象領域を確かめる |
| ハードウェアのライセンス名 | 未確認（製品文書に記載が見つからなかった） | M5Stack の設計ファイル（https://github.com/m5stack/M5_Hardware）を確かめる |
| USB の方式（USB Serial/JTAG か）の公式の明記 | 未確認 | 公式の記述を探す |
| 3.3 V 電源の出力電流 | 未確認 | 回路図・部品の資料で確かめる |
| StampS3A の電源部品 | 未確認 | StampS3A の回路図で確かめる |
| ESP-IDF 文書の版の違い（stable 版と v5.5.2） | 未確認 | v5.5.2 の文書で引用箇所を照合する |
| 手元の機体が StampFly v1.0 か v1.1 か、StampS3 か StampS3A か | 未確認 | 機体の表示・基板で確かめる |
| 読み出しが 1 台だけであること | 1 台のみ | 他の機体でも読み出す |
| RAM を多く使う置き方（命令キャッシュの拡大・コードを IRAM に置く）を採用するか | 検討中 | 空きと処理時間の表（6・7 章）を基準に判断する |

---

<a id="english"></a>

# StampFly Microcontroller Module Memory

## 1. Overview

### About This Document

This document summarizes the memory specification of the StampFly microcontroller module (M5StampS3, chip ESP32-S3FN8) and how the current firmware (`firmware/vehicle/`) uses it. It is the reference for judging "is there enough memory" and "where to place things" when adding features.

Facts are written with their source distinguished.

| Source | Meaning |
|--------|---------|
| Read from the real device | Results of read-only esptool / espefuse access to one vehicle (2026-11-21) |
| Official material | Public documents of M5Stack and Espressif (with URL, viewed 2026-11-21) |
| From the build | Build result of the current firmware (commit 53553678, ESP-IDF v5.5.2) |
| Measured on the real device | Measurements on a running vehicle (2026-11-20 and 2026-11-21) |
| Developer's explanation | What the developer told us (2026-11-21) |

Units are kept as in the original material. B is bytes, KB follows the source's notation, and KiB means 1024 B (1 KiB = 1024 B). Values obtained by calculation are marked "calculated".

### Target Audience

- People who add features to the vehicle firmware or tighten its processing time
- People who want to check how much memory is left on the microcontroller

### Summary of Conclusions

| Item | Content |
|------|---------|
| External RAM (PSRAM) | None (the PSRAM fields in the eFuse are None; the part-number table of the datasheet also lists no PSRAM for the ESP32-S3FN8) |
| Flash | 8 MB inside the chip package (Quad SPI) |
| Internal RAM | Of 512 KB, about 37 KB is free after boot with the current firmware (measured 36,736 B) |
| Memory placement and time | The time of one 400 Hz processing cycle depends strongly on placement (in the experiments, from 1359-1367 µs down to 366 µs) |

## 2. Module and Chip Specification

### Specification Table

| Item | Value | Source |
|------|-------|--------|
| Module on the StampFly | M5StampS3. M5StampS3A (with an A) is the newer version. The memory specification is the same for both | Developer |
| Chip | ESP32-S3FN8 (QFN56, 7×7 mm) | Official (Espressif datasheet) |
| Chip report | `ESP32-S3 (QFN56) (revision v0.2)`, Embedded Flash 8MB (GD), crystal 40 MHz | Real device |
| CPU | Xtensa 32-bit LX7 dual core, up to 240 MHz | Official |
| CPU frequency setting | 240 MHz (boot message 240000000 Hz) | Build / real device |
| Internal flash | 8 MB (Quad SPI), ambient temperature −40 to 85 °C, VDD_SPI 3.3 V | Official |
| External RAM (PSRAM) | None (PSRAM column is "—" in the datasheet part-number table) | Official |
| ROM | 384 KB | Official |
| SRAM | 512 KB | Official |
| SRAM in RTC | 16 KB | Official |
| Power circuit | Integrates a 5 V to 3.3 V circuit. DC-DC is MUN3CAD01-SC, input 5 V (StampS3 product documentation) | Official |

### StampS3 and StampS3A

| Aspect | Content | Source |
|--------|---------|--------|
| Relation | StampS3A is the newer version. The memory specification is the same | Developer |
| Differences listed by the official documentation | RGB LED power control, antenna, size of the boot button, power consumption in sleep (on USB power, 88.82 µA for S3A and 400.67 µA for S3) | Official |
| Official statements about memory | No mention of a memory difference. No sentence states that the memory specification is identical (both list the same SoC and 8 MB flash) | Official |
| PSRAM description | Not in either product documentation | Official |

### StampFly Versions and the Module on Them

| Version | Product documentation statement | URL |
|---------|---------------------------------|-----|
| v1.0 | Carries the Stamp-S3 | https://docs.m5stack.com/en/app/StampFly |
| v1.1 | "uses the Stamp-S3A" | https://docs.m5stack.com/en/products/sku/K138-V11 |

Which version the vehicle at hand is has not been checked in this document.

### References

| Material | URL |
|----------|-----|
| M5Stack product documentation StampS3 | https://docs.m5stack.com/en/core/StampS3 |
| StampS3 schematic | https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/522/Sch_M5StampS3_v0.2.pdf |
| M5Stack product documentation StampS3A | https://docs.m5stack.com/en/core/Stamp-S3A |
| StampS3A schematic | https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1150/Sch_StampS3_v0.3.3.pdf |
| M5Stack product documentation StampFly (v1.0) | https://docs.m5stack.com/en/app/StampFly |
| M5Stack product documentation StampFly (v1.1) | https://docs.m5stack.com/en/products/sku/K138-V11 |
| M5Stack hardware design files | https://github.com/m5stack/M5_Hardware |
| Espressif ESP32-S3 Series Datasheet (Version 2.2, revision history 2026-03-05) | https://documentation.espressif.com/esp32-s3_datasheet_en.pdf |
| ESP-IDF documentation Memory Types | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/memory-types.html |
| ESP-IDF documentation Speed Optimization | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/performance/speed.html |
| ESP-IDF documentation Minimizing RAM Usage | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/performance/ram-usage.html |

The ESP-IDF pages were read in their stable-version display, while the vehicle firmware uses v5.5.2. Differences caused by the version difference have not been checked (chapter 8).

## 3. Values Read from the Real Device

### How They Were Read

| Item | Content |
|------|---------|
| Date | 2026-11-21 |
| Target | One vehicle |
| Method | Read-only esptool and espefuse (no writes, no eFuse burning) |
| Record | `logs/checkout/chip_info_20261121.txt` (outside Git) |

### Value Table

| Item | Value |
|------|-------|
| Chip | ESP32-S3 (QFN56) (revision v0.2) |
| Features | WiFi, BLE, Embedded Flash 8MB (GD) |
| Crystal | 40 MHz |
| Flash ID | Manufacturer c8, Device 4017 |
| Detected flash size | 8 MB |
| Flash setting in eFuse | quad (4 data lines), 3.3 V |
| eFuse FLASH_CAP / FLASH_VENDOR / FLASH_TEMP | 8M / GD / 85C |
| eFuse PSRAM_CAP / PSRAM_VENDOR / PSRAM_TEMP | None / None / None |
| PSRAM_CAPACITY (calculated value) | 0 |
| PKG_VERSION / WAFER_VERSION_MAJOR / MINOR / BLK_VERSION_MINOR | 0 / 0 / 2 / 3 |
| Boot message (ESP-IDF v5.5.2, firmware 53553678) | chip revision v0.2, SPI Flash Size 8MB, cpu freq 240000000 Hz, detected chip gd, flash io dio |

An eFuse is a write-once storage element inside the chip.

### Why We Can Say There Is No External RAM

| Basis | Content | Source |
|-------|---------|--------|
| PSRAM fields in the eFuse | PSRAM_CAP, PSRAM_VENDOR and PSRAM_TEMP are all None, and PSRAM_CAPACITY (calculated value) is 0 | Real device |
| Part-number table of the datasheet | ESP32-S3FN8 has In-Package Flash of 8 MB (Quad SPI) and In-Package PSRAM "—" | Official |
| Build setting | CONFIG_SPIRAM is not set | Build |

The eFuse of the real device and the official part-number table agree. Only one vehicle was read.

## 4. Memory Organization

### Internal Memory (Datasheet)

| Type | Size | Role |
|------|------|------|
| ROM | 384 KB | Non-rewritable memory |
| SRAM | 512 KB | RAM for variables and code |
| SRAM in RTC | 16 KB | SRAM for the RTC |

SRAM (Static RAM) is the fast RAM inside the chip.

### Instruction Cache and Data Cache

A cache is a mechanism that copies the frequently used code and data from flash into fast memory close to the CPU.

| Item | Content | Source |
|------|---------|--------|
| Structure | An instruction cache and a data cache, shared by the two CPU cores | Official |
| Instruction cache size options | 16 KB or 32 KB | Official |
| Data cache size options | 32 KB or 64 KB | Official |
| Current setting (instruction) | 16 KB (0x4000, 8 way, 32 B line) | Build |
| Current setting (data) | 32 KB (0x8000) | Build |

### Data and Code Share the Same RAM

The ESP-IDF documentation states the following (quoted from the original).

- "Any internal SRAM which is not used for Instruction RAM will be made available as DRAM"
- "The maximum statically allocated DRAM size is reduced by the IRAM size of the compiled application."
- "Increasing the cache size will also result in reducing the amount of available RAM."

DRAM is the RAM that holds data and IRAM is the RAM that holds instructions (code). The RAM available for data decreases by the amount of code placed in IRAM.

### Flash Regions (Partition Table)

A partition table is a list that divides the flash by purpose.

| Partition | Start | Size | Purpose |
|-----------|-------|------|---------|
| nvs | 0x9000 | 0x6000 (24 KB) | — |
| phy_init | 0xF000 | 0x1000 | — |
| factory | 0x10000 | 0x300000 (3 MB) | Application (firmware itself) |
| storage (spiffs) | 0x310000 | 0x200000 (2 MB) | spiffs (file area on the flash) |

The end of the last partition, storage, is 0x510000 (calculated). How the flash after it is used has not been checked.

### Overview of Internal RAM Use

The numbers come from the table in chapter 5 (build). The bar length is a rough ratio to the DIRAM total of 341,760 B (calculated).

```
DIRAM 341,760 B (area shared by data and code)
|-----------|---|-----------|---------------|
  .bss        .data  .text       remaining
  fixed       fixed  code placed  area left for
  variables   vars   in RAM       dynamic allocation
  93,688 B    with   91,451 B     (on the map)
  (27.4 %)    initial (26.8 %)    132,917 B
              values              (38.9 %)
              23,704 B
              (6.9 %)
```

The "remaining 132,917 B" is a value on the build-time map and is measured differently from the free 36,736 B measured as heap (memory pool allocatable at run time) after boot (chapter 6). This document does not connect the two by subtraction.

## 5. How the Current Firmware Uses It

### Breakdown from the Build

From commit 53553678, ESP-IDF v5.5.2, `idf_size.py` output.

| Region | Total | Used | Remaining | Breakdown |
|--------|-------|------|-----------|-----------|
| DIRAM (shared by data and code) | 341,760 B | 208,843 B (61.11 %) | 132,917 B | .bss 93,688 B, .text 91,451 B, .data 23,704 B |
| IRAM (code only) | 16,384 B | 16,384 B | 0 B (calculated) | .text 15,356 B, .vectors 1,028 B |
| RTC SLOW | 8,192 B | 32 B | — | — |
| RTC FAST | 8,192 B | 24 B | — | — |
| Flash (application) | 3 MB (factory partition) | about 1.2 MB (binary) | — | code .text 856,080 B, data .rodata 239,244 B |

.bss is the area of fixed variables initialized to zero at start-up, and .data is the area of fixed variables with initial values.

### Libraries with Large .bss and .data

| Library | .bss | .data |
|---------|------|-------|
| libsf_app_hooks.a | 32,470 B | 1,392 B |
| libmain.a | 30,022 B | 2,123 B |
| libsf_core.a | 14,683 B | 436 B |
| libnet80211.a (WiFi) | 7,606 B | 938 B |
| liblwip.a | 2,470 B | 12 B |
| libpp.a (WiFi) | 1,242 B | 2,640 B |
| libfreertos.a | 757 B | 3,106 B |
| libhal.a | 4 B | 3,425 B |
| libphy.a | 86 B | 3,272 B |
| libspi_flash.a | 24 B | 2,148 B |

### Configured Task Stack Sizes

A stack is the per-task work area that holds the progress of function calls and local variables. A task is the unit of processing that FreeRTOS (an embedded operating system) runs concurrently.

`STACK_*` in `firmware/vehicle/main/config.hpp`.

| Task | Setting |
|------|---------|
| IMU | 16,384 B |
| CONTROL | 8,192 B |
| STATE | 4,096 B |
| OPTFLOW | 8,192 B |
| MAG | 8,192 B |
| BARO | 8,192 B |
| COMM | 4,096 B |
| TOF | 8,192 B |
| TELEMETRY | 4,096 B |
| POWER | 4,096 B |
| BUTTON | 4,096 B |
| NOTIFY | 4,096 B |
| CLI (task that reads console input) | 8,192 B |
| LOG | 6,144 B |
| API | 6,144 B |
| TELLO_STATE | 4,096 B |
| Total (calculated) | 106,496 B (104 KiB) |

The ESP-IDF side stack settings are: main task 3,584 B, esp_timer 3,584 B, event 4,096 B, lwIP TCP/IP 3,072 B, IPC 1,280 B (per core), idle 1,536 B (per core). They are not included in the total above.

### ESP-IDF Features with Code Placed in RAM Enabled

| Setting | Value |
|---------|-------|
| CONFIG_ESP_WIFI_IRAM_OPT | y |
| CONFIG_ESP_WIFI_RX_IRAM_OPT | y |
| CONFIG_FREERTOS_IN_IRAM | y |
| CONFIG_ESP_SYSTEM_IN_IRAM | y |
| CONFIG_SPI_MASTER_IN_IRAM | y |
| CONFIG_ESP_TIMER_IN_IRAM | y |

All of the project's own code is in flash, and there are 0 `IRAM_ATTR` specifications. The WiFi receive buffer settings are 10 static, 32 dynamic receive and 32 dynamic transmit.

### Heap Regions at Boot

From the boot message of the baseline firmware (real device).

| Start address | Size | Type shown |
|---------------|------|-----------|
| 3FCBB090 | 0x2E680 (185 KiB) | RAM |
| 3FCE9710 | 0x5724 (21 KiB) | RAM |
| 3FCF0000 | 0x8000 (32 KiB) | DRAM |
| 600FE000 | 0x1FE8 (7 KiB) | RTCRAM |

The total of the four regions is 245 KiB (calculated; not exact because the display is rounded).

### About 245 KiB Right After Boot and About 37 KB After Boot

| Quantity | Value | Source |
|----------|-------|--------|
| Total heap right after boot | about 245 KiB = about 250,880 B (calculated) | Calculated from the boot message of the real device |
| Free 10 s after boot | 36,736 B (about 35.9 KiB, calculated) | Measured on the real device (chapter 6) |
| Difference | about 214,144 B (about 209 KiB, calculated) | Calculated |
| Total of the configured stack sizes | 106,496 B (calculated from the table in chapter 5); about half of the difference (calculated) | Build |

The total of the configured stack sizes is listed as one candidate destination of the difference. Items not confirmed in this document are as follows.

- That the stacks are allocated from the heap present at boot
- Whether the 7 KiB of RTCRAM among the 245 KiB is counted in the measured free amount
- The breakdown of dynamic allocations such as WiFi (not measured; unconfirmed)

## 6. Free Memory Measured on the Real Device

### How It Was Measured

| Item | Content |
|------|---------|
| Date | 2026-11-21 |
| State | On the floor, not armed, battery connected, all sensors running, only the USB console connected |
| Measurement time | 10 s and 40 s after boot |
| Meaning of the value | The `heap` line of the `status` command: free internal RAM satisfying MALLOC_CAP_INTERNAL and MALLOC_CAP_8BIT |
| Note | The `heap` line was added in commit 3b8b0212 of the experiment branch exp/imu-cycle-budget-20261120. It is not yet in fix/flight-safety |

### Results for Four Firmwares

| Firmware | Free (10 s / 40 s) | Minimum since boot | Largest contiguous block | Largest heap region at boot | Console |
|----------|--------------------|--------------------|--------------------------|-----------------------------|---------|
| Baseline (53553678 + heap line) | 36,736 / 36,776 B | 34,116 B | 23,552 B | 185 KiB | Works |
| Instruction cache 32 KB | 20,420 / 20,388 B | 18,292 B | 10,240 B | 169 KiB | Works |
| ImuTask and ESKF code placed in IRAM (+19.3 KB, .iram0.text 106,807 to 126,087 B) | 17,616 / 17,552 B | 15,488 B | 7,680 B | 166 KiB | Works |
| Both of the above at once | Could not be read | — | — | 150 KiB | Console task could not be created and initialization failed (`console REPL init failed: ESP_FAIL`) |

Results of another trial (2026-11-20) are as follows.

| Code placed | Result |
|-------------|--------|
| +30.9 KB in IRAM | Console initialization failed |
| +66.8 KB in IRAM | `esp_wifi_init` failed with ESP_ERR_NO_MEM and the board restarted repeatedly |

### Guide to How Much More Can Be Used

| Comparison | Calculation |
|------------|-------------|
| Difference between the baseline free amount (10 s) and the instruction cache 32 KB free amount (10 s) | 36,736 − 20,420 = 16,316 B |
| Difference between the baseline free amount (10 s) and the free amount with +19.3 KB in IRAM (10 s) | 36,736 − 17,616 = 19,120 B |

The facts, lined up, are as follows.

- A change that reduces free memory by about 16 KB from the baseline (cache 32 KB) and one that reduces it by about 19 KB (+19.3 KB in IRAM) each ran on its own, including the console.
- The change placing +30.9 KB in IRAM failed console initialization.
- The change doing both of the above at once failed console initialization.

No "upper limit of what can be added" is set from these measurements. Points remain unmeasured, beyond what can be stated numerically.

### Unconfirmed

| Item | Status |
|------|--------|
| Free memory with the WiFi CLI (TCP 23) or a `sf log wifi` capture connected | Unconfirmed |
| Free memory while armed (flight state) | Unconfirmed |
| Actual stack usage of each task | Unconfirmed |

## 7. Memory Placement and Processing Time

**Status: under consideration.** The tables below are experimental results, and whether to adopt any of them has not been decided.

### Code That Does Not Fit in the Instruction Cache

| Item | Content |
|------|---------|
| Size of code that must run every cycle | ImuTask about 8.8 KB, ESKF about 8 KB. Plus control, drivers and math functions |
| Instruction cache (shared by both cores) | 16 KB |
| Judgment | The total of the code above does not fit in the 16 KB instruction cache |
| Placement of the project's own code | All in flash (0 `IRAM_ATTR` specifications) |

ESKF is the filter that estimates the attitude and position of the vehicle (error-state Kalman filter).

### Measurements with Different Placements

| Condition | Mean processing time of one ImuTask cycle | Cycles over budget | Note |
|-----------|-------------------------------------------|--------------------|------|
| Baseline | 1359-1367 µs (4 series) | 0.69-1.45 % | |
| Instruction cache 32 KB | 693 / 701 µs (2 series) | 0 % | |
| ImuTask and ESKF in IRAM | 483 µs | 0 % | Control-task wake-up (leaving the waiting state to become runnable) delay 0 % |
| Both | 366 µs | Not recorded | Console does not start |

The measurement conditions are 2026-11-20, on the floor, not armed, USB through a hub. ImuTask runs at 400 Hz and the budget of one cycle is 2500 µs (the period of 400 Hz is 1/400 s = 2500 µs, calculated). The detailed record is outside Git (`logs/checkout/`). The experiment switches are in the experiment-branch commits f6db9203 to 3b8b0212.

### Trade-off between Speed and Memory

The ESP-IDF documentation states the following.

- "IRAM is a limited resource, and using more IRAM may reduce available DRAM"
- "Increasing the cache size will also result in reducing the amount of available RAM."

Putting the free memory of chapter 6 together with the processing times of this chapter, the placements that shorten the processing time are the ones that reduce the free memory after boot.

## 8. Unconfirmed Items and What to Check Next

| Item | Status | What to check next |
|------|--------|--------------------|
| Actual stack usage of the tasks | Unconfirmed | Read the minimum free stack (maximum usage) of each task on the real device |
| Free memory with WiFi connected (TCP 23, `sf log wifi`) | Unconfirmed | Read the `heap` line of `status` in the connected state |
| Free memory while armed | Unconfirmed | Read the free memory in the armed state (avoid wired operation in flight; read it over WiFi) |
| Breakdown of dynamic allocations (WiFi etc.) | Unconfirmed (not measured) | Measure the amount per allocation source in the heap |
| That the stacks are included in the "difference" in chapter 5 | Unconfirmed | Check where stacks are allocated in the documentation and code |
| Whether the measured free amount includes the 7 KiB of RTCRAM | Unconfirmed | Check the regions covered by the `heap` line |
| Name of the hardware licence | Unconfirmed (not found in the product documentation) | Check the M5Stack design files (https://github.com/m5stack/M5_Hardware) |
| Official statement of the USB method (whether USB Serial/JTAG) | Unconfirmed | Look for an official statement |
| Output current of the 3.3 V supply | Unconfirmed | Check in the schematics and component documents |
| Power components of StampS3A | Unconfirmed | Check in the StampS3A schematic |
| Differences between ESP-IDF documentation versions (stable and v5.5.2) | Unconfirmed | Compare the quoted passages with the v5.5.2 documentation |
| Whether the vehicle at hand is StampFly v1.0 or v1.1, and StampS3 or StampS3A | Unconfirmed | Check the markings and the board of the vehicle |
| Only one vehicle was read | One vehicle only | Read other vehicles too |
| Whether to adopt placements that trade internal RAM for speed (cache, IRAM) | Under consideration | Decide using the tables of free memory and processing time (chapters 6 and 7) |
