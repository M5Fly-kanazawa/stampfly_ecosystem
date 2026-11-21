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
| 最適化の余地 | SRAM の領域は使い切っているが、既定の構成で使わない SNN（スパイキングニューラルネットワーク）が約 31 KB を占める。スタックは設定値 106,496 B に対し、床の上の使用量の最大の合計が 41,884 B（8 章） |

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

ESP-IDF 文書は stable 版の表示を読んだもので、機体のファームは v5.5.2 である。版の違いによる記述の差は確かめていない（9 章）。

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

### 内蔵 SRAM 512 KiB の内訳

基準のファーム（53553678）の起動時の表示のアドレスからの計算による。

| 領域 | 大きさ | 使い道 | 出どころ |
|------|--------|--------|----------|
| 0x40370000〜0x40378000 のうち下の部分 | 16 KiB | 命令キャッシュ用と見られる（推定。Espressif の技術資料とは未照合） | 計算 |
| 0x40374000 から（IRAM） | 16 KiB | コード専用の領域 | リンカの定義 |
| 0x3FC88000〜0x3FCBB090 | 204.1 KiB | ファームの固定分 | 計算 |
| 〜0x3FCE9710（ヒープの領域 1） | 185.6 KiB | ヒープ | 起動時の表示から計算 |
| 〜0x3FCEEE34（ヒープの領域 2） | 21.8 KiB | ヒープ。領域 1 との間に隙間は無い | 起動時の表示から計算 |
| 〜0x3FCF0000 | 4.4 KiB | ヒープに入らない。ROM の予約と見られる（推定） | 計算 |
| 0x3FCF0000 からの 64 KiB のうち | 32 KiB | ヒープの領域 3 | 起動時の表示 |
| 同上の残り | 32 KiB | データキャッシュ用と見られる（推定） | 計算 |
| 合計 | 32 + 416 + 64 = 512 KiB | | 計算 |

0x40370000〜0x40378000 の 32 KiB は、上の 16 KiB が IRAM、下の 16 KiB が命令キャッシュ用と見られる。0x3FC88000〜0x3FCF0000 は 416 KiB である。使われていない領域は見つからなかった。推定とした 3 か所（命令キャッシュ用、ROM の予約、データキャッシュ用）は、8 章・9 章の確認事項に挙げる。

### 内蔵 RAM の使われ方の概観

数値は 5 章の表（ビルド）による。上の内訳（512 KiB 全体）とは別の見方で、DIRAM の合計 341,760 B だけを扱う。2 つの数値は突き合わせていない。帯の長さは DIRAM 合計 341,760 B に対する割合（計算）の概略である。

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

### 大きな変数（.bss）

基準のビルドの ELF（実行ファイル）から。行番号は `firmware/vehicle/` 以下。SNN とは、ニューロンの発火を模したニューラルネットワーク（スパイキングニューラルネットワーク）の推定器・制御器のことである。関数内 static とは、関数の中で static をつけて宣言した変数のことで、その関数が呼ばれなくても固定の領域に RAM を占める。リングとは、固定長の領域を循環して使う記憶の仕組みのことである。

| 変数 | RAM | 内容と根拠 |
|------|-----|-----------|
| `sf::app::stock::estimator()::snn` | 15,568 B | SNN の推定器。`components/sf_app_hooks/stock_hooks.cpp:93-95` の関数内 static |
| `sf::app::stock::controller()::snn_controller` | 15,464 B | SNN の制御器。同 :63-66 |
| `g_tof_bottom`、`g_tof_front` | 各 10,904 B | VL53L3CX のドライバの作業領域（ST の VL53LX のデバイス構造体を含む）。`tasks/tof_task.cpp:108,112` |
| `g_mag_cal` | 6,060 B | 地磁気の較正器。標本バッファ float×500×3 = 6,000 B が大半（`components/sf_hal_bmm150/include/mag_calibration.hpp:85,185-187`、`tasks/mag_task.cpp:72`） |
| `sf::log_stream` | 4,876 B | 400 Hz のログの待ち行列（深さ 32 = 80 ms 分。`topics.hpp:77`） |
| `s_console_ring`、`s_wifi_ring` | 各 4,116 B | ログシンクのリング（各 4,096 B + 管理。`sf_core/log_sink.cpp:53-54`、`log_sink.hpp:72,80`） |

`bmi270_config_file`（8,192 B）と `sf::params::table`（4,032 B）は、nm（シンボルの一覧を出すツール）の種別は D/d（初期値つきのデータ）だが、アドレスが 0x3C10xxxx でフラッシュ上にあり、RAM は使っていない。nm の種別だけでは RAM かフラッシュかを判断できない。

SNN について。

- 既定の構成は controller.type = 0（PID）、estimator.type = 0（ESKF）で、SNN は使わない（`params.cpp:283-292`、`stock_hooks.cpp:68-69,96-98`）。
- 選ばれなくても関数内 static のため RAM を占める。コメントは「ヒープを使わず静的に生成する」方針を述べている（`stock_hooks.cpp:88-92`）。
- SNN は未学習の固定乱数を入れた研究用の枠組みで、コードの冒頭に、SILS で墜落するのが想定どおりと書かれている（`snn_controller.hpp` の冒頭）。
- 大きさの大半は、上限（入力 16、ニューロン 48）で確保した配列である。実際の構成は 10 入力・16 ニューロンである（`cuba_lif_layer.hpp:77-78`、`snn_controller.hpp:58-59`）。
- 既存の文書 `firmware/vehicle/docs/m5stampfly_firmware_comparison.md`（A4、132-146 行）が「研究用の機能が既定のビルドに入っている」と指摘している。

### タスクのスタックの設定値と実測

スタックとは、関数呼び出しの途中経過や局所変数を置く、タスクごとの作業領域のことである。タスクとは、FreeRTOS（組み込み向けの OS）が並行して動かす処理の単位のことである。

設定値は `firmware/vehicle/main/config.hpp` の `STACK_*`。実測は 2026-11-21、実験ブランチのコミット f2f4b199 の `stack` コマンドによる（記録 `logs/checkout/stack_floor_1.txt`、Git の管理外）。単位は B。空きの最小は、起動以降で空きが最も小さくなったときの残りである。使用量の最大は、設定値から空きの最小を引いた値（計算）である。

| タスク | 設定値 | 空きの最小 | 使用量の最大 | コア | 優先度 |
|--------|--------|-----------|--------------|------|--------|
| ImuTask | 16,384 | 11,864 | 4,520 | 1 | 24 |
| ControlTask | 8,192 | 5,232 | 2,960 | 1 | 23 |
| StateTask | 4,096 | 1,668 | 2,428 | 0 | 22 |
| FlowTask | 8,192 | 5,600 | 2,592 | 0 | 20 |
| MagTask | 8,192 | 5,848 | 2,344 | 0 | 18 |
| BaroTask | 8,192 | 6,016 | 2,176 | 0 | 16 |
| TofTask | 8,192 | 5,752 | 2,440 | 0 | 14 |
| PowerTask | 4,096 | 1,568 | 2,528 | 0 | 12 |
| CommTask | 4,096 | 1,224 | 2,872 | 0 | 15 |
| TelemetryTask | 4,096 | 1,728 | 2,368 | 0 | 13 |
| ButtonTask | 4,096 | 1,752 | 2,344 | 0 | 10 |
| NotifyTask | 4,096 | 1,788 | 2,308 | 0 | 8 |
| CLITask（コンソールの入力を読むタスク） | 8,192 | 5,540 | 2,652 | 0 | 5 |
| LogTask | 6,144 | 3,484 | 2,660 | 0 | 5 |
| ApiTask | 6,144 | 3,984 | 2,160 | 0 | 6 |
| TelloStateTask | 4,096 | 1,564 | 2,532 | 0 | 6 |
| 16 個の合計（計算） | 106,496（104 KiB） | 64,612 | 41,884 | | |

ESP-IDF が作るタスクの空きの最小（設定値は取得できない）は次のとおり。

| タスク | 空きの最小 |
|--------|-----------|
| esp_timer | 3,072 B |
| ipc0 | 528 B |
| ipc1 | 536 B |
| IDLE0 | 728 B |
| IDLE1 | 720 B |
| wifi | 4,136 B |
| sys_evt | 3,340 B |
| tiT（lwIP） | 1,488 B |
| console_repl | 5,692 B |

ESP-IDF 側のスタックの設定値は、main タスク 3,584 B、esp_timer 3,584 B、イベント 4,096 B、lwIP の TCP/IP 3,072 B、IPC 1,280 B（コアごと）、アイドル 1,536 B（コアごと）である。上の合計には含めていない。

同じ起動での空き（`status` の `heap` 行）は 36,516 B、起動以降の最小 34,416 B、最大の塊 23,552 B であった。

測定の条件と限界は次のとおり。

- 条件: 床の上、ARM していない、電池あり、全センサ動作、USB コンソールのみ、起動 75 s 後。WiFi の接続なし、較正なし。
- ARM、飛行、WiFi の CLI や取得、地磁気の較正を行うと、使用量は増えうる（未確認）。
- スタックの設定値を決めた根拠は、LogTask（printf に約 1.5 KB）以外、コメントにも設計文書にも見つからなかった。
- `config.hpp:104` のコメントは合計を「92KB」と書くが、16 個の合計は 106,496 B（計算）で合わない。

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

### 既定から変えているビルド設定

`firmware/vehicle/sdkconfig.defaults`（コメントと空行を除いて 28 行）のうち、メモリに関係するもの。

| 設定 | 値 |
|------|-----|
| CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM | 10 |
| CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM | 32 |
| CONFIG_ESP_WIFI_AMPDU_TX_ENABLED | n |
| CONFIG_ESP_WIFI_AMPDU_RX_ENABLED | n |
| CONFIG_ESP_WIFI_IRAM_OPT | y |
| CONFIG_SPI_MASTER_IN_IRAM | y |
| CONFIG_I2C_ISR_IRAM_SAFE | y |
| CONFIG_GPIO_CTRL_FUNC_IN_IRAM | y |
| CONFIG_LEDC_CTRL_FUNC_IN_IRAM | y |
| CONFIG_RMT_ISR_IRAM_SAFE | y |
| CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE | 4096 |
| CONFIG_NVS_DEFAULT_PART_SIZE | 0x6000 |
| CONFIG_COMPILER_OPTIMIZATION_PERF | y |

命令キャッシュ・データキャッシュの大きさと、400 Hz の制御経路のコードの置き場所についての指定は無い。ESP-IDF の既定のままで、自前のコードはすべてフラッシュ上にある。

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
| ARM・飛行・WiFi 接続・較正のときのタスクのスタックの使用量 | 未確認（床の上の値は 5 章に測った） |

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

## 8. 最適化の候補

**状態: 検討中。どれも採用は決まっていない。** 見込みは条件が異なるため、足し算で 1 つの数字にまとめない。

### 候補の表

| 候補 | 見込み | 動作への影響 | 確かめること |
|------|--------|--------------|--------------|
| SNN を既定のビルドから外す（ビルド時の選択にする）。実験ブランチで実装済み（コミット e67a2fb9、`CONFIG_SF_ENABLE_SNN`、既定は無効）。実機で測った空き（2026-11-21、床の上、ARM していない、起動 10 s 後 / 40 s 後）は 67,504 / 67,516 B、起動以降の最小 66,228 B、最大の連続した塊 31,744 B で、外す前（36,736 B、6 章）より約 30.8 KB 多い（計算） | 約 31 KB（15,568 + 15,464 B、計算） | 既定の構成では変化なし。既定のビルドでは SNN を選べなくなる | SNN を使う SILS のシナリオ・テストの有無、教育・研究用として残す方針 |
| SNN の配列の上限を実際の構成に合わせる | 1 個あたり 8〜10 KB（概算、未検証） | SNN は選べるまま | 上限を使う別の構成が無いか |
| タスクのスタックを実測に合わせる | 下の試算で 33,792 B（床の上の実測に基づく） | あふれの危険。CommTask は逆に増やす候補 | ARM・飛行・WiFi 接続・較正を行った後の実測 |
| 地磁気の較正用の標本バッファを較正のときだけ確保する | 約 6,000 B | 較正の開始時に確保に失敗しうる | 較正は地上だけか、標本数 500 の根拠 |
| 前方 ToF が無い機体でドライバの作業領域を確保しない | 最大 10,904 B（前方 ToF の無い機体だけ） | 搭載機では変わらない | 搭載の判定の時点 |
| WiFi のバッファ数・WiFi の処理を RAM に置く設定の見直し | 未確認（測っていない） | 通信の余裕が減る | テレメトリ・ログの取得に要る量 |
| ログのリング・待ち行列を縮める | 2.4〜4 KB 程度 | ログの欠落への余裕が減る | 欠落の実測 |

### スタックの試算

**目安であり、決定ではない。** 5 章の床の上の実測（ARM・WiFi 接続・較正なし）に、次の規則を当てた試算である。

- 規則: 使用量の最大の 1.5 倍以上、かつ空きの最小 1,536 B 以上になるように、1,024 B の倍数へ切り上げる。

| タスク | 今 | 試算 |
|--------|-----|------|
| ImuTask | 16,384 B | 7,168 B |
| ControlTask | 8,192 B | 5,120 B |
| FlowTask | 8,192 B | 5,120 B |
| MagTask | 8,192 B | 4,096 B |
| BaroTask | 8,192 B | 4,096 B |
| TofTask | 8,192 B | 4,096 B |
| CLITask | 8,192 B | 5,120 B |
| LogTask | 6,144 B | 5,120 B |
| ApiTask | 6,144 B | 4,096 B |
| 合計の減り（計算） | | 33,792 B |

- 上の表にないタスクは、この規則では今の値のままである。
- CommTask は、この規則だと今の 4,096 B では足りず 5,120 B が要る（今の空きの最小は 1,224 B）。上の合計の減りには、この増加分を含めていない。

### 空けたメモリの使い道の候補

7 章の「命令キャッシュ 32 KB」（追加 約 16 KB）と「ImuTask と ESKF を IRAM」（追加 約 19 KB）の併用。併用したビルドは、今の空きではコンソールを起動できなかった（6 章）。

## 9. 未確認の事項と次に確かめること

| 項目 | 状態 | 次に確かめること |
|------|------|------------------|
| ARM・飛行・WiFi 接続・較正の後のタスクのスタックの使用量 | 未確認（床の上の値は 5 章に測った） | それぞれの状態の後に `stack` コマンドで空きの最小を読む |
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
| SNN を使う SILS のシナリオ・テストの有無 | 未確認 | シナリオとテストを調べる |
| WiFi のバッファ（テレメトリ・ログの取得）に要る量 | 未確認 | 取得に必要な量を測る |
| SRAM の内訳の推定 3 か所（命令キャッシュ用 16 KiB、ROM の予約 4.4 KiB、データキャッシュ用 32 KiB） | 未確認（推定。Espressif の技術資料と未照合） | Espressif の技術資料と照合する |
| SNN の配列の上限を使う別の構成の有無 | 未確認 | SNN のコードを調べる |
| 地磁気の較正を地上だけで行うか、標本数 500 の根拠 | 未確認 | 設計文書とコードで確かめる |
| 前方 ToF の搭載の判定の時点 | 未確認 | ToF のコードで確かめる |
| ログの欠落への余裕（リング・待ち行列を縮めたときの欠落） | 未確認 | 欠落を実測する |
| スタックの設定値の根拠（LogTask 以外） | 見つからなかった | 設計した人に確かめる |
| RAM を多く使う置き方（命令キャッシュの拡大・コードを IRAM に置く）を採用するか | 検討中 | 空きと処理時間の表（6・7 章）と最適化の候補（8 章）を基準に判断する |

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
| Room for optimization | Every region of the SRAM is in use, but the SNN (spiking neural network), which the default configuration does not use, occupies about 31 KB. Against configured stacks of 106,496 B, the sum of the maximum usage on the floor is 41,884 B (chapter 8) |

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

The ESP-IDF pages were read in their stable-version display, while the vehicle firmware uses v5.5.2. Differences caused by the version difference have not been checked (chapter 9).

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

### Breakdown of the 512 KiB Internal SRAM

Calculated from the addresses in the boot message of the baseline firmware (53553678).

| Region | Size | Use | Source |
|--------|------|-----|--------|
| Lower part of 0x40370000-0x40378000 | 16 KiB | Appears to be for the instruction cache (estimate; not checked against Espressif technical material) | Calculated |
| From 0x40374000 (IRAM) | 16 KiB | Code-only area | Linker definition |
| 0x3FC88000-0x3FCBB090 | 204.1 KiB | Fixed part of the firmware | Calculated |
| to 0x3FCE9710 (heap region 1) | 185.6 KiB | Heap | Calculated from the boot message |
| to 0x3FCEEE34 (heap region 2) | 21.8 KiB | Heap. No gap to region 1 | Calculated from the boot message |
| to 0x3FCF0000 | 4.4 KiB | Not part of the heap. Appears to be a ROM reservation (estimate) | Calculated |
| Of the 64 KiB from 0x3FCF0000 | 32 KiB | Heap region 3 | Boot message |
| Rest of the above | 32 KiB | Appears to be for the data cache (estimate) | Calculated |
| Total | 32 + 416 + 64 = 512 KiB | | Calculated |

Of the 32 KiB at 0x40370000-0x40378000, the upper 16 KiB is IRAM and the lower 16 KiB appears to be for the instruction cache. 0x3FC88000-0x3FCF0000 is 416 KiB. No unused region was found. The three places marked as estimates (instruction cache, ROM reservation, data cache) are listed among the items to check in chapters 8 and 9.

### Overview of Internal RAM Use

The numbers come from the table in chapter 5 (build). This is a different view from the breakdown above (the whole 512 KiB) and covers only the DIRAM total of 341,760 B. The two sets of numbers are not matched against each other. The bar length is a rough ratio to the DIRAM total of 341,760 B (calculated).

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

### Large Variables (.bss)

From the ELF (executable file) of the baseline build. Line numbers are relative to `firmware/vehicle/`. An SNN is an estimator or controller built as a neural network that imitates neuron firing (a spiking neural network). A function-local static is a variable declared with `static` inside a function; it occupies RAM in a fixed area even if the function is never called. A ring is a storage mechanism that reuses a fixed-length area cyclically.

| Variable | RAM | Content and basis |
|----------|-----|-------------------|
| `sf::app::stock::estimator()::snn` | 15,568 B | SNN estimator. Function-local static at `components/sf_app_hooks/stock_hooks.cpp:93-95` |
| `sf::app::stock::controller()::snn_controller` | 15,464 B | SNN controller. Same file :63-66 |
| `g_tof_bottom`, `g_tof_front` | 10,904 B each | Work area of the VL53L3CX driver (includes the device structure of ST's VL53LX). `tasks/tof_task.cpp:108,112` |
| `g_mag_cal` | 6,060 B | Magnetometer calibrator. Mostly the sample buffer float×500×3 = 6,000 B (`components/sf_hal_bmm150/include/mag_calibration.hpp:85,185-187`, `tasks/mag_task.cpp:72`) |
| `sf::log_stream` | 4,876 B | Queue of the 400 Hz log (depth 32 = 80 ms; `topics.hpp:77`) |
| `s_console_ring`, `s_wifi_ring` | 4,116 B each | Rings of the log sinks (4,096 B each + management; `sf_core/log_sink.cpp:53-54`, `log_sink.hpp:72,80`) |

`bmi270_config_file` (8,192 B) and `sf::params::table` (4,032 B) have the nm (a tool that lists symbols) type D/d (initialized data), but their addresses are 0x3C10xxxx, which is in flash, and they use no RAM. The nm type alone cannot tell RAM from flash.

About the SNN.

- The default configuration is controller.type = 0 (PID) and estimator.type = 0 (ESKF); the SNN is not used (`params.cpp:283-292`, `stock_hooks.cpp:68-69,96-98`).
- Because they are function-local statics, they occupy RAM even when not selected. The comment states a policy of creating them statically without using the heap (`stock_hooks.cpp:88-92`).
- The SNN is a research framework with untrained fixed random values; the head of the code says that crashing in SILS is expected (head of `snn_controller.hpp`).
- Most of the size is arrays allocated for the upper limits (16 inputs, 48 neurons). The actual configuration is 10 inputs and 16 neurons (`cuba_lif_layer.hpp:77-78`, `snn_controller.hpp:58-59`).
- The existing document `firmware/vehicle/docs/m5stampfly_firmware_comparison.md` (A4, lines 132-146) points out that "research features are in the default build".

### Configured Task Stack Sizes and Measurements

A stack is the per-task work area that holds the progress of function calls and local variables. A task is the unit of processing that FreeRTOS (an embedded operating system) runs concurrently.

The settings are the `STACK_*` in `firmware/vehicle/main/config.hpp`. The measurements are from 2026-11-21 with the `stack` command of commit f2f4b199 on the experiment branch (record `logs/checkout/stack_floor_1.txt`, outside Git). Units are B. The minimum free is the remainder at the moment when the free stack was smallest since boot. The maximum usage is the setting minus the minimum free (calculated).

| Task | Setting | Minimum free | Maximum usage | Core | Priority |
|------|---------|--------------|---------------|------|----------|
| ImuTask | 16,384 | 11,864 | 4,520 | 1 | 24 |
| ControlTask | 8,192 | 5,232 | 2,960 | 1 | 23 |
| StateTask | 4,096 | 1,668 | 2,428 | 0 | 22 |
| FlowTask | 8,192 | 5,600 | 2,592 | 0 | 20 |
| MagTask | 8,192 | 5,848 | 2,344 | 0 | 18 |
| BaroTask | 8,192 | 6,016 | 2,176 | 0 | 16 |
| TofTask | 8,192 | 5,752 | 2,440 | 0 | 14 |
| PowerTask | 4,096 | 1,568 | 2,528 | 0 | 12 |
| CommTask | 4,096 | 1,224 | 2,872 | 0 | 15 |
| TelemetryTask | 4,096 | 1,728 | 2,368 | 0 | 13 |
| ButtonTask | 4,096 | 1,752 | 2,344 | 0 | 10 |
| NotifyTask | 4,096 | 1,788 | 2,308 | 0 | 8 |
| CLITask (task that reads console input) | 8,192 | 5,540 | 2,652 | 0 | 5 |
| LogTask | 6,144 | 3,484 | 2,660 | 0 | 5 |
| ApiTask | 6,144 | 3,984 | 2,160 | 0 | 6 |
| TelloStateTask | 4,096 | 1,564 | 2,532 | 0 | 6 |
| Total of the 16 (calculated) | 106,496 (104 KiB) | 64,612 | 41,884 | | |

The minimum free stack of the tasks created by ESP-IDF (the settings cannot be obtained) is as follows.

| Task | Minimum free |
|------|--------------|
| esp_timer | 3,072 B |
| ipc0 | 528 B |
| ipc1 | 536 B |
| IDLE0 | 728 B |
| IDLE1 | 720 B |
| wifi | 4,136 B |
| sys_evt | 3,340 B |
| tiT (lwIP) | 1,488 B |
| console_repl | 5,692 B |

The ESP-IDF side stack settings are: main task 3,584 B, esp_timer 3,584 B, event 4,096 B, lwIP TCP/IP 3,072 B, IPC 1,280 B (per core), idle 1,536 B (per core). They are not included in the total above.

In the same boot, the free memory (the `heap` line of `status`) was 36,516 B, the minimum since boot 34,416 B and the largest block 23,552 B.

The conditions and limits of the measurement are as follows.

- Conditions: on the floor, not armed, battery connected, all sensors running, USB console only, 75 s after boot. No WiFi connection, no calibration.
- Arming, flight, the WiFi CLI or capture, and magnetometer calibration may increase the usage (unconfirmed).
- No basis for the configured stack sizes was found in comments or design documents, except for LogTask (about 1.5 KB for printf).
- The comment at `config.hpp:104` states the total as "92KB", but the total of the 16 is 106,496 B (calculated), which does not match.

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

### Build Settings Changed from the Defaults

Of `firmware/vehicle/sdkconfig.defaults` (28 lines excluding comments and blank lines), the memory-related ones.

| Setting | Value |
|---------|-------|
| CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM | 10 |
| CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM | 32 |
| CONFIG_ESP_WIFI_AMPDU_TX_ENABLED | n |
| CONFIG_ESP_WIFI_AMPDU_RX_ENABLED | n |
| CONFIG_ESP_WIFI_IRAM_OPT | y |
| CONFIG_SPI_MASTER_IN_IRAM | y |
| CONFIG_I2C_ISR_IRAM_SAFE | y |
| CONFIG_GPIO_CTRL_FUNC_IN_IRAM | y |
| CONFIG_LEDC_CTRL_FUNC_IN_IRAM | y |
| CONFIG_RMT_ISR_IRAM_SAFE | y |
| CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE | 4096 |
| CONFIG_NVS_DEFAULT_PART_SIZE | 0x6000 |
| CONFIG_COMPILER_OPTIMIZATION_PERF | y |

There is no specification of the instruction and data cache sizes, nor of where the code of the 400 Hz control path is placed. They remain at the ESP-IDF defaults, and all of the project's own code is in flash.

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
| Stack usage of the tasks during arming, flight, WiFi connection and calibration | Unconfirmed (floor values are measured in chapter 5) |

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

## 8. Optimization Candidates

**Status: under consideration. None of them has been decided for adoption.** The expected savings are not added into a single number because their conditions differ.

### Table of Candidates

| Candidate | Expected saving | Effect on behavior | What to check |
|-----------|-----------------|--------------------|---------------|
| Remove the SNN from the default build (make it a build-time choice). Implemented on the experiment branch (commit e67a2fb9, `CONFIG_SF_ENABLE_SNN`, off by default). Free memory measured on the real vehicle (2026-11-21, on the floor, not armed, 10 s / 40 s after boot) is 67,504 / 67,516 B, minimum since boot 66,228 B, largest block 31,744 B, about 30.8 KB more than before (36,736 B, chapter 6; calculated) | About 31 KB (15,568 + 15,464 B, calculated) | No change in the default configuration. The SNN can no longer be selected in the default build | Whether any SILS scenario or test uses the SNN; the policy of keeping it for education and research |
| Fit the SNN array upper limits to the actual configuration | 8-10 KB each (rough estimate, unverified) | The SNN stays selectable | Whether another configuration uses the upper limits |
| Fit the task stacks to the measurements | 33,792 B in the estimate below (based on measurements on the floor) | Risk of overflow. CommTask is rather a candidate to increase | Measurements after arming, flight, WiFi connection and calibration |
| Allocate the magnetometer calibration sample buffer only during calibration | About 6,000 B | Allocation may fail when calibration starts | Whether calibration happens only on the ground; the basis for 500 samples |
| Do not allocate the driver work area on vehicles without a front ToF | Up to 10,904 B (only on vehicles without a front ToF) | No change on equipped vehicles | At what point the presence is determined |
| Review the number of WiFi buffers and the settings that place WiFi processing in RAM | Unconfirmed (not measured) | Less margin for communication | Amount needed for telemetry and log capture |
| Shrink the log rings and queue | About 2.4-4 KB | Less margin against log loss | Measurement of log loss |

### Stack Estimate

**This is a guide, not a decision.** It applies the following rule to the floor measurements of chapter 5 (no arming, no WiFi connection, no calibration).

- Rule: round up to a multiple of 1,024 B so that the stack is at least 1.5 times the maximum usage and the minimum free is at least 1,536 B.

| Task | Now | Estimate |
|------|-----|----------|
| ImuTask | 16,384 B | 7,168 B |
| ControlTask | 8,192 B | 5,120 B |
| FlowTask | 8,192 B | 5,120 B |
| MagTask | 8,192 B | 4,096 B |
| BaroTask | 8,192 B | 4,096 B |
| TofTask | 8,192 B | 4,096 B |
| CLITask | 8,192 B | 5,120 B |
| LogTask | 6,144 B | 5,120 B |
| ApiTask | 6,144 B | 4,096 B |
| Total reduction (calculated) | | 33,792 B |

- Tasks not in the table keep their current values under this rule.
- For CommTask, this rule says the current 4,096 B is not enough and 5,120 B is needed (the current minimum free is 1,224 B). This increase is not included in the total reduction above.

### Candidate Uses of the Freed Memory

Using together the "instruction cache 32 KB" (about 16 KB more) and "ImuTask and ESKF in IRAM" (about 19 KB more) of chapter 7. The build with both could not start the console with the current free memory (chapter 6).

## 9. Unconfirmed Items and What to Check Next

| Item | Status | What to check next |
|------|--------|--------------------|
| Task stack usage after arming, flight, WiFi connection and calibration | Unconfirmed (floor values are measured in chapter 5) | Read the minimum free stack with the `stack` command after each state |
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
| Whether any SILS scenario or test uses the SNN | Unconfirmed | Check the scenarios and tests |
| Amount needed for WiFi buffers (telemetry and log capture) | Unconfirmed | Measure the amount needed for capture |
| Three estimates in the SRAM breakdown (16 KiB for the instruction cache, 4.4 KiB ROM reservation, 32 KiB for the data cache) | Unconfirmed (estimates; not checked against Espressif technical material) | Check against Espressif technical material |
| Whether another configuration uses the SNN array upper limits | Unconfirmed | Check the SNN code |
| Whether magnetometer calibration happens only on the ground; the basis for 500 samples | Unconfirmed | Check in design documents and code |
| At what point the presence of the front ToF is determined | Unconfirmed | Check in the ToF code |
| Margin against log loss (loss when the rings and queue are shrunk) | Unconfirmed | Measure the loss |
| Basis for the configured stack sizes (other than LogTask) | Not found | Ask the designer |
| Whether to adopt placements that trade internal RAM for speed (cache, IRAM) | Under consideration | Decide using the tables of free memory and processing time (chapters 6 and 7) and the optimization candidates (chapter 8) |
