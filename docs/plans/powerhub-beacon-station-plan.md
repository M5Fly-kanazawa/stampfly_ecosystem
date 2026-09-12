# M5Stack PowerHub を TDMA ビーコン専用局にする計画（10 月 4 日「プログラミングチャレンジ」向け）

状態: **計画中**（Phase 0〜5 の日程表のみで、本書上に着手・進捗の記録なし。9/22 を代替案切替の判断日とする期限あり）。作成 2026-09-09、最終更新 2026-09-09。

作成: 2026-09-09（同日、行事名を「プログラミングチャレンジ」に確定）。

発端: 複数のコントローラ（送信機）を同じ空間で使うとき、StampFly は TDMA（時分割多元接続:
1 つの無線チャンネルを 20ms の枠に区切り、各コントローラに 2ms ずつ送信時間を割り当てる方式）で
送信の衝突を避けている。その時刻基準になる「ビーコン」は Device ID 0 のコントローラだけが送る。
つまり同期の要が参加者の手元の 1 台に握られており、その 1 台の電源が落ちる・別のチャンネルに
居るだけで、他のコントローラは同期を失う。DXH 高校教員講座（2026-07-18/19）では 10 台を
1/6/11 チャンネルに分散したが ID 0 は 1 台だけだったので、3 チャンネル中 2 チャンネルは
ビーコン無しで運用していたことになる。10 月 4 日の「プログラミングチャレンジ」（小中学生向けの
プログラミング講習会）では、講師側が管理する専用のビーコン局を各チャンネルに置いて、この依存を
断つ。参加者が小中学生なので、コントローラの ID・チャンネルの設定は全て運営側が事前に済ませ、
当日は子どもが触らない構成にする。

方針の前提（2026-09-08/09 ユーザー確認）:

- ビーコン局のハードウェアは **M5Stack PowerHub**（SKU C148 / 電池付き K148。メイン MCU が
  ESP32-S3、サブ MCU が STM32G031 の電源管理用）。外部アンテナ（SMA）と着脱電池（NP-F550）を
  持ち、講習会場に据え置くのに向く。
- PowerHub の **公式機能は残す**（前面 USB・Grove・RS485/CAN の電源 ON/OFF、電池や各系統の
  電圧電流監視、LED 表示、ボタン操作）。そのため、M5Stack 公式の ESP32-S3 ファームウェア
  （ESP-IDF 製、MIT ライセンス）を本リポジトリに取り込み、そこへビーコン送出機能を加える。
- STM32 側の公式ファームウェアは書き換えない。
- ビーコン局は **ビーコンだけ**を送る。ID 0 コントローラが担っていた操縦（スロット 0 の
  制御パケット送信）は肩代わりしない。
- チャンネルは PC に USB 接続して **sf CLI から任意（1〜13）に変更**できること（2026-09-09）。
- 講習では **複数の参加者が同時にコントローラで操縦する場面がある**ため、ビーコン局は必要（2026-09-09 確定）。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| ビーコンの実体 | 2 バイト固定 `0xBE 0xAC` を 20ms 周期でブロードキャスト（`firmware/controller/components/espnow_tdma/espnow_tdma.c`）。受信側（ID 1〜9 のコントローラ）は送信元 MAC を検証せず、ペアリングも不要。機体はビーコンを無視する（14 バイト以外を破棄） |
| PowerHub の適性 | 無線・USB・アプリロジックは ESP32-S3 が担当。STM32 は電源管理専用で、ESP32 からの命令が無くても自律動作する（ESP32 電源・充電・電源ボタンは STM32 が起動時に無条件で有効化）。ESP-IDF v5.4.1 で自作ファームを書ける |
| 公式ファームの衝突点 | ESP-NOW は未使用。ただし Wi-Fi を「初回設定用アクセスポイント（チャンネル 1 固定）+ BLE 設定」と「家庭ルータへの接続 + クラウド連携（MQTT）」に使う。ESP-NOW は固定チャンネルが前提なので、**ビーコン動作中はこれらを起動しない排他運用**にする |
| 方針 | `firmware/powerhub/` に公式ファームを取り込み（取り込み元のコミットを記録し、変更は最小限）、`main/hal/utils/beacon/` にビーコン部品を追加。設定 `beacon_channel`（0 = 公式動作のまま、1〜13 = ビーコン局）で切り替え、値は PC から `sf powerhub channel N` で変える。ビーコンの定数は `firmware/common/protocol/` に置き、コントローラと共有する |
| 構成 | 使用チャンネルごとに PowerHub 1 台（最大 3 台 + 予備 1 台）。参加者のコントローラは全て ID 1〜9。会場に ID 0 のコントローラを置かない |
| 見積り | Phase 0〜5 で 6〜8 日（実働）。最大の不確定要素は公式ファームが本リポジトリの ESP-IDF v5.5 でビルドできるか（Phase 0 で先に確認） |
| 期限 | 2026-10-04（日）「プログラミングチャレンジ」（小中学生向けプログラミング講習会）。9 月 22 日を代替案（ビーコン専用の最小ファーム）への切替判断日とする |

## 1. 現状の事実

### ビーコンと子機の挙動（コントローラファーム）

| 項目 | 事実 | 根拠 |
|------|------|------|
| 送信条件 | `g_tdma_device_id == 0` のときだけ `beacon_task` が生成され、20ms 周期タイマーで送る | `espnow_tdma.c` 279〜301, 426〜442 行 |
| パケット | `{0xBE, 0xAC}` 2 バイト、宛先 `FF:FF:FF:FF:FF:FF`、暗号化なし | 同 38, 175, 186 行、`protocol/spec/messages.yaml` 451〜465 行 |
| 受信側 | ID≠0 のコントローラが受信時刻をフレーム開始時刻にする。送信元 MAC は見ない | 同 156〜167 行 |
| ビーコン未受信時 | 子機は送信を止めない。起動時刻を起点にした自分だけのタイマーで送り続ける（他機と位相は揃わない） | 同 194〜276 行（`first_beacon_received` を参照しない） |
| ID 0 が外部ビーコンを受けたとき | 何もしない。自分のビーコンも送り続ける（二重ビーコンになる） | 同 157 行の `g_tdma_device_id != 0` 条件 |
| 子機のチャンネル | ペアリング時に機体が広告するチャンネル（機体の `wifi.channel` パラメータ）に自動で合う。ID 0 だけがメニューで変更できる | `espnow_tdma.c` 134〜149 行、`main.cpp` 1508 行ほか、`docs/guides/controller.md` 79〜80 行 |
| 仕様書の不整合 | `protocol/spec/espnow_tdma.yaml` 142〜147 行は子機を vehicle と書くが、実装で聞くのは他のコントローラ | 本計画で修正する |

### PowerHub のハードウェアと公式ファーム

| 項目 | 事実 |
|------|------|
| MCU | ESP32-S3-WROOM-1U-N16R8（外部アンテナ、フラッシュ 16MB）+ STM32G031G8U6（I2C スレーブ、電源管理） |
| STM32 の自律性 | 起動時に I2C 初期化より前に ESP32 電源・LED 電源・充電を ON。側面ボタン（単押し = ESP32 リセット、二度押し = 電源 OFF、3 秒長押し = ダウンロードモード）を自律処理。ESP32 からの生存信号は不要 |
| 公式 ESP32 ファーム | `m5stack/M5PowerHub-UserDemo`（MIT）。ESP-IDF v5.4.1、自前コード約 6,200 行。`app`（表示・ボタン・設定）と `main/hal`（ESP32 固有）に層分け。依存: arduino-esp32 3.2.0（IDF コンポーネント）、asynctcp、espasyncwebserver、mooncake（独自スクリプトで取得） |
| 無線の使い方 | 未設定時: AP「PowerHub-XXXX」（チャンネル 1 固定）+ BLE（BluFi）で 10 秒間設定を待つ。設定済み: 家庭ルータへ STA 接続しクラウド（MQTT）へ送信。Wi-Fi スキャンあり。**ESP-NOW は 0 件** |
| ボタン UI | SELECT/OK で設定モードに入り、USB/UART/BUS/I2C の電源を巡回選択して ON/OFF。LED で状態表示。SELECT 4 秒長押しで工場出荷状態 |
| 設定保存 | LittleFS（フラッシュ上の小さなファイルシステム）の `/config.json` |
| 周期処理 | メインループ 1ms、1 周に I2C 約 15 回（電圧電流 7 系統 + 電源状態 7 + LED 32 バイト）。I2C は 400kHz でミューテックス保護 |

## 2. 設計判断

### 取り込み方式: ソースを複製し、変更を局所化する

| 選択肢 | 判断 |
|--------|------|
| git submodule で公式リポジトリを参照 | 不採用。公式は mooncake を独自スクリプトで取得する構成で、submodule にしてもビルド前手順が残る。sf CLI の「`firmware/<target>/CMakeLists.txt` があればビルド対象」という規約にも乗らない |
| **ソース複製（採用）** | `firmware/powerhub/` に公式 `powerhub/` プロジェクトを平坦に複製。mooncake / mooncake_log は `third_party/` に複製し `EXTRA_COMPONENT_DIRS` で参照。`firmware/powerhub/UPSTREAM.md` に取り込み元 URL・コミット・変更ファイル一覧を記録し、上流更新時に差分を当て直せるようにする |

変更を加える上流ファイルは次の 3 か所に限定し、それ以外は無改変とする。

| ファイル | 変更 |
|----------|------|
| `main/hal/hal_esp32.cpp` の `init()` 末尾 | `beacon_channel != 0` なら `beacon::start(channel)` を呼ぶ（1〜3 行） |
| `app/apps/app_ezdata/app_ezdata.cpp` の `onOpen()` | `beacon_channel != 0` なら AP・BluFi・ルータ接続を起動しない（条件分岐 1 つ） |
| `app/hal/hal.h` の `Config_t` と `config.json` 読み書き | `int beacon_channel = 0` を追加 |

### 無線の排他: ビーコン中は公式の Wi-Fi/BLE 機能を止める

ESP32-S3 の無線は 1 つで、ESP-NOW は「今 Wi-Fi が居るチャンネル」で送る。公式ファームの
ルータ接続はチャンネルをルータが決め、Wi-Fi スキャンは全チャンネルを巡る。どちらもビーコンの
固定チャンネルと両立しない。ビーコン局にクラウド連携は不要なので、`beacon_channel != 0` の
ときは AP・BluFi・ルータ接続・スキャンを一切起動しない。`beacon_channel == 0` なら公式動作の
ままで、PowerHub を普通の製品として使える。

Wi-Fi の初期化は公式が Arduino の `WiFi.mode()` 経由で行っているため、ビーコン部品も同じ経路で
`WiFi.mode(WIFI_STA)` → `esp_wifi_set_channel()` → `esp_now_init()` の順に進め、`esp_wifi_init`
の二重呼び出しを避ける。

### チャンネルの決め方: PC に USB 接続し sf CLI から任意に変更する（2026-09-09 ユーザー指定）

ビルド時固定やボタン操作ではなく、PowerHub を PC に USB で接続して sf CLI から 1〜13 の
任意チャンネルに変える。機体側の `param set wifi.channel N` と同じ文法にそろえ、講師が
覚える操作を増やさない。

| 層 | 内容 |
|----|------|
| ファーム側のコマンド口 | `main/hal/utils/console/` に文字コマンドの受け口を追加。USB Serial/JTAG（ESP32-S3 内蔵の USB シリアル。下部 USB-C が直結）を主コンソールにする（公式は UART が主、USB は副）。文法は機体の `sf_command` にそろえる: `param get beacon.channel`、`param set beacon.channel N`、`param save`、`beacon status`（送信回数・失敗回数・現在チャンネル）、`reboot` |
| 保存先 | 公式の `config.json`（LittleFS）に `beacon_channel` を持たせ、単一の設定置き場を守る |
| 反映のしかた | 1〜13 の間の変更は再起動なしで即時反映（`esp_wifi_set_channel` を呼び直す）。0（公式動作）との切替は AP・BLE の起動有無が変わるため `reboot` を要する |
| sf CLI | `sf powerhub channel N`（設定 + 保存 + 読み返し確認）、`sf powerhub channel`（現在値表示）、`sf powerhub status`。実装は `sf cal` の `_send_calibration_command`（pyserial でポート自動検出、`\r\n` 終端で送信、応答行を読む）を流用 |
| 表示 | LED 1 個をチャンネル色で点灯（1 = 赤、6 = 緑、11 = 青、その他 = 白）。`sf monitor` でも 1Hz のログに現在チャンネルを出す |
| ラベル | 講習では 1/6/11 の 3 台を運用するので、設定後に「CH1 / CH6 / CH11」の付け替え式ラベルを貼る |

### タイミング: 独立タスク、優先度は既存最高より上

公式ファームの既存タスクは優先度 5〜15。ビーコン送信タスクは `esp_timer` の 20ms 周期通知を
受ける独立タスク（優先度 20 程度、I2C に依存しない）とし、メインループの I2C 処理に巻き込まれ
ないようにする。BLE はビーコン中は起動しないので、Wi-Fi と BLE の無線時間の取り合いは生じない。
ジッタ（周期のばらつき）は受け入れ基準で実測する。

### ビーコン定数の置き場: `firmware/common/protocol/` に集約

`0xBE 0xAC`・20ms・先行時間 500µs は現状コントローラの `espnow_tdma.h` にしかない。
`firmware/common/protocol/include/espnow_protocol.hpp`（3 ファームが既に共有）に
`kTdmaBeaconMarker` と `kTdmaFramePeriodUs` を追加し、PowerHub はこれを参照する。
コントローラ側を同定数に置き換える変更は **SCI チュートリアル（9/10）後**に行い、動作は変えない。

### 運用構成: 1 チャンネル 1 局、ID 0 は会場に置かない

```
   機体 wifi.channel = 1   ──ペアリング──▶  コントローラ群 A（ID 1〜9）  ◀── PowerHub #1（CH1）
   機体 wifi.channel = 6   ──ペアリング──▶  コントローラ群 B（ID 1〜9）  ◀── PowerHub #2（CH6）
   機体 wifi.channel = 11  ──ペアリング──▶  コントローラ群 C（ID 1〜9）  ◀── PowerHub #3（CH11）
```

スロット 0 は空くので 1 チャンネルあたり子機 9 台、3 チャンネルで 27 台まで。ID 0 の
コントローラが会場に残ると二重ビーコンになるため、事前に全台を ID 1〜9 にし、当日チェック
リストで確認する。ビーコン局が止まっても子機は自走タイマーで送信を続けるので、故障時は
「同期なし」に退化するだけで飛行は止まらない。

## 3. 受け入れ基準（講習前に満たすこと）

| # | 基準 | 確認方法 |
|---|------|----------|
| 1 | ID 1 と ID 2 のコントローラが、PowerHub 起動後 1 秒以内に画面に「SYNC」を表示する | ベンチ、目視 |
| 2 | PowerHub の電源を切ると 200ms 後に「LOST」、再投入で「SYNC」に戻る | ベンチ、目視 |
| 3 | ビーコン間隔が 20ms ± 0.2ms に収まる（99% 以上） | 子機側で `last_beacon_time_us` の差分を 1 分間ログし分布を出す |
| 4 | 3 台の PowerHub を 1/6/11 で同時に動かし、各チャンネルのコントローラが自チャンネルでのみ SYNC する | ベンチ |
| 5 | 公式機能が生きている: ボタンで前面 USB 電源を ON/OFF できる、電池 LED 表示が動く | ベンチ、目視 |
| 6 | 10m 離れて SYNC を維持する | 会場相当の空間 |
| 7 | 同一チャンネルで 3 機以上を同時にホバリングさせ、コントローラの送信成功率（`send_success_count`）が ID 0 運用時と同等以上 | 実飛行、ログ比較 |
| 8 | 電池のみで 4 時間以上ビーコンを送り続ける | 連続稼働、開始時と終了時の電池電圧を記録 |
| 9 | `sf build powerhub` → `sf flash powerhub` → `sf powerhub channel N` の 3 手順で書き込みとチャンネル設定が完了し、電源を切っても設定が残る | Windows と macOS |
| 10 | `sf powerhub channel 6` を実行すると再起動なしに子機の SYNC 表示がチャンネル 6 側へ移る | ベンチ |

## 4. 実装計画

### Phase 0: ビルド可否の確認と機材調達（1 日、9/11〜）

| 作業 | 内容 |
|------|------|
| 機材発注 | PowerHub 本体 3 台 + 予備 1 台、NP-F550 電池、USB-C ケーブル。納期 1 週間を見込む |
| ビルド試験 | 公式ファームを無改変で本リポジトリの ESP-IDF v5.5 でビルドする。arduino-esp32 3.2.0 は IDF 5.4 系向けなので失敗する可能性が高い。失敗なら arduino-esp32 3.3 系への更新で通るか確認する |
| 合否判定 | ビルドが通らず、更新でも 1 日で解決しない場合は Phase 1 に進まず、§5 の代替案を発動する |

ハードウェア到着前でも、ビーコン部品の送信試験は手元の AtomS3（同じ ESP32-S3）で行える。

### Phase 1: 共有定数と仕様書の整備（0.5 日）

| 作業 | 対象 | 内容 |
|------|------|------|
| 定数追加 | `firmware/common/protocol/include/espnow_protocol.hpp` | `kTdmaBeaconMarker[2]`、`kTdmaFramePeriodUs`、`kTdmaBeaconAdvanceUs` を追加。コントローラの値と一致することを `static_assert` で固定 |
| 仕様書修正 | `protocol/spec/espnow_tdma.yaml` | 子機の役割を「他のコントローラ」に修正。ビーコン局（controller 以外の送信元）を許容する旨を追記 |
| 文書更新 | `docs/architecture/tdma-usage.md` | 現行 ESP-IDF 版に合わない行番号参照を直し、「ビーコン局」節を追加 |

### Phase 2: 取り込みとビーコン部品（2 日）

| 作業 | 対象 | 内容 |
|------|------|------|
| 複製 | `firmware/powerhub/`、`third_party/mooncake*/` | 公式 `powerhub/` を平坦に複製。`UPSTREAM.md` に元コミットを記録。未使用の `arduinoWebSockets` はビルドから除外 |
| 依存の固定 | `firmware/powerhub/main/idf_component.yml` | asynctcp / espasyncwebserver を `==` で固定し `dependencies.lock` をコミット（m5unified で 2026-07-19 に起きた版ずれ障害の再発防止） |
| ビーコン部品 | `main/hal/utils/beacon/beacon.{h,cpp}` | Wi-Fi STA 起動 → チャンネル固定 → ESP-NOW 初期化 → ブロードキャスト宛先登録 → 20ms タイマーと送信タスク。送信回数・失敗回数を 1Hz でログ |
| 差し込み | `hal_esp32.cpp`、`app_ezdata.cpp`、`hal.h` | §2 の 3 か所 |
| コマンド口 | `main/hal/utils/console/console.{h,cpp}`、`sdkconfig.defaults` | USB Serial/JTAG を主コンソールに変更。行単位で読み `param get/set/save`、`beacon status`、`reboot` を解釈（機体の `sf_command` と同じ文法、150 行程度） |
| チャンネル表示 | `main/hal/utils/beacon/` | LED 1 個をチャンネル色で点灯（既存の `setLedColor` を使用） |
| sf CLI | `lib/sfcli/commands/powerhub.py`（新規） | `sf powerhub channel [N]`、`sf powerhub status`。`sf cal` のシリアル送受信を流用。`sf app` の予約名に `powerhub` を追加 |

### Phase 3: ベンチ検証（1 日、機材到着後）

受け入れ基準 1〜5 と 9〜10。基準 3 のジッタ分布は `analysis/` にスクリプトを置いて図にする。
二重ビーコン（ID 0 コントローラを故意に併存）で子機の表示がどう乱れるかも記録し、当日の
トラブル対応表に載せる。

### Phase 4: 飛行と会場相当の検証（1 日）

受け入れ基準 6〜8。基準 7 は同じ機体・同じ場所で「ID 0 コントローラ運用」と「PowerHub 運用」を
交互に 2 回ずつ行い、セッションをまたいで再現することを確認する。

### Phase 5: 運用文書と当日手順（0.5 日）

| 成果物 | 内容 |
|--------|------|
| `docs/guides/controller.md` | 「ビーコン局を使う運用」節。ID 0 を置かない理由、チャンネル対応表 |
| `docs/events/programming_challenge_2026/` | 機材チェックリスト（PowerHub 3 台・電池・アンテナ・ラベル）、設置手順（電源投入 → 各コントローラの SYNC 確認）、トラブル対応表（LOST が出た、二重ビーコン、チャンネル違い）。運営側が事前に済ませる作業（全コントローラの ID 1〜9 設定、機体の `wifi.channel` 設定、ペアリング）を子どもの手順と分けて書く |
| 筐体ラベル | 「CH1 / CH6 / CH11」 |

### 日程

| 期間 | 内容 |
|------|------|
| 9/10 | SCI チュートリアル（本計画の作業はしない） |
| 9/11〜9/12 | Phase 0（発注、ビルド試験） |
| 9/14〜9/18 | Phase 1〜2（ハードウェア無しで進められる範囲。AtomS3 で送信試験） |
| 9/22 | **代替案への切替判断日** |
| 9/21〜9/25 | Phase 3（機材到着後） |
| 9/28〜10/2 | Phase 4〜5、リハーサル |
| 10/3 | 最終確認、電池満充電 |
| 10/4 | 講習 |

## 5. リスクと未確認事項

| 項目 | 内容 | 対策 |
|------|------|------|
| ESP-IDF 版の不一致 | 公式は v5.4.1、本リポジトリは v5.5。arduino-esp32 3.2.0 が v5.5 で解決・ビルドできるかは未確認 | Phase 0 で最初に確認。arduino-esp32 の更新で通らなければ代替案 |
| 代替案 | 公式機能を諦め、ビーコンだけを送る最小ファーム（Wi-Fi 初期化 + 20ms 送信、100 行程度）を `firmware/powerhub/` に置く。PowerHub の電源ボタン・充電・ESP32 電源は STM32 が自律処理するので、この構成でも本体は動く。失うのは前面 USB 等の ON/OFF、電池監視、LED 表示 | 9/22 に判断。10/4 に間に合わせることを優先する |
| 二重ビーコン | 参加者のコントローラが ID 0 のまま会場に入る | 事前に全台 ID 変更、当日チェックリスト。ビーコン局が他の送信元からの `0xBE 0xAC` を受けたら LED で警告（Phase 2 で余裕があれば） |
| チャンネル不一致 | 機体の `wifi.channel` と PowerHub の `beacon_channel` の対応が崩れる | 対応表を掲示。コントローラの SYNC 表示で当日確認 |
| ESP-NOW の実績 | PowerHub 上での ESP-NOW 送信は M5Stack 公式に実績の記載がない。チップはコントローラと同じ ESP32-S3 なので技術的障害は無いと見る | Phase 2 で最初に送信を確認 |
| 書き込み手順 | ESP32-S3 の GPIO0 は STM32 が駆動している。`sf flash` の自動リセットで書き込めるか未確認。手動なら側面ボタン 3 秒長押し | Phase 3 で確認し、手順書に明記 |
| USB コンソールの切替 | 公式は UART が主コンソール。USB Serial/JTAG を主にすると公式のログ出力先が変わる。公式コードが UART 前提の処理を持つかは未確認 | Phase 2 で `sf monitor` による表示を確認 |
| 消費電力 | ビーコン送信時の平均電流は未実測。NP-F550（7.4V 2000mAh）で講習時間を持つ見込みだが根拠は無い | 受け入れ基準 8 で実測 |
| 講習の規模 | 参加人数・機体台数・使用チャンネル数が未確定 | ユーザーが決める。PowerHub の台数 = 使用チャンネル数 + 予備 1 |
| I2C の 50 秒タイムアウト | 公式の I2C ミューテックス待ちは 50 秒。I2C 異常時にメインループが長時間止まる | ビーコンタスクを I2C に依存させない設計で影響を遮断 |
| 仕様書の不整合 | `espnow_tdma.yaml` の子機 = vehicle | Phase 1 で修正 |

## 6. ユーザーに決めてもらう事項

| 項目 | 選択肢 | 推奨 |
|------|--------|------|
| 参加人数と機体台数 | 未定 | 決まり次第、使用チャンネル数と PowerHub 台数を確定 |
| 講習の形式 | 複数の参加者が同時にコントローラで操縦する場面がある（2026-09-09 ユーザー確定） | ビーコン局は必要。本計画を進める。PC からのプログラミング場面が併存する場合は、その場面の Wi-Fi チャンネル設計を別途扱う |
| 10 月 4 日のカレンダー | 「月例会」の終日予定が残っている | 行事名の書き換えは別途 |
| チャンネル指定方式 | PC + sf CLI（決定済み） | `sf powerhub channel N`。ボタン切替は作らない |
| 代替案の発動条件 | 9/22 時点で公式ファームの取り込みが受け入れ基準 1〜2 を満たさないとき | 発動する（公式機能は後日追加） |
| クラウド連携（EzData） | ビーコン中は無効 | 無効のまま。必要なら `beacon_channel = 0` で公式動作に戻せる |

## 7. 関連文書

| 文書 | 内容 |
|------|------|
| `docs/architecture/tdma-usage.md` | TDMA の解説（PlatformIO 時代の行番号参照あり、Phase 1 で更新） |
| `protocol/spec/espnow_tdma.yaml`、`protocol/spec/messages.yaml` | ビーコンのワイヤフォーマット |
| `docs/guides/controller.md` | Device ID・チャンネルの運用 |
| `docs/events/dxh2026/setup-loaner-pc.md`、`equipment-checklist.md` | 10 台を 1/6/11 に分散した実績 |
| [M5PowerHub-UserDemo](https://github.com/m5stack/M5PowerHub-UserDemo) | 公式 ESP32-S3 ファーム（取り込み元） |
| [M5PowerHub-Internal-FW](https://github.com/m5stack/M5PowerHub-Internal-FW) | STM32 側ファームとレジスタ表 |
| [PowerHub 製品ドキュメント](https://docs.m5stack.com/en/core/PowerHub) | ピン配置、ダウンロードモード手順 |
