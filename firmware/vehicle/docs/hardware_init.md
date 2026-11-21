# vehicle Hardware Initialization Design
# vehicle ハードウェア初期化設計

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 本文書の位置づけ

### このドキュメントについて

vehicle における **共有ハードウェア資源の所有権・初期化フロー・失敗時の挙動・HAL との接続規約** を定義する。具体的には次の問いに答える。

- I2C バスや SPI バスのような共有 HW 資源を誰が所有するか
- センサ・アクチュエータ・通信スタックの初期化はどの順序で行うか
- センサ init が失敗したらどう振る舞うか（飛行を止めるか、機能無効で続けるか）
- 学習者・ファーム実装者が触る範囲（namespace 境界）はどこか

設計文書 [`architecture.md`](architecture.md) §7 の要約に対する詳細版である。

### 対象読者

- vehicle ファームの実装者・拡張者
- 「StampFly のハードウェア層を学びたい」HW 学習者（L2/L3 アクセスを使う層）
- Workshop API（L0）や Topic API（L1）の利用者は本文書を読む必要はない

### 関連文書

| 文書 | 役割 |
|------|------|
| [`requirements.md`](requirements.md) | 要件定義 |
| [`architecture.md`](architecture.md) | 階層構造・コンポーネント分割・横断ルール |
| [`detailed_design.md`](detailed_design.md) | Topic 定義・インターフェース・状態遷移 |
| [`coding_and_education.md`](coding_and_education.md) | コーディング規約・教育計画 |
| **本文書** | **共有 HW 資源の所有・初期化・namespace 規約** |

---

## 2. 設計の背景

### 旧 vehicle/ で発生した問題

| # | 問題 | 場所 | 影響 |
|---|------|-----|-----|
| 1 | extern グローバル乱立 | `globals.hpp` の `g_imu`, `g_mag`, `g_baro`, `g_tof_*`, `g_optflow`, `g_power` | 「センサオブジェクトは誰が所有するか」が消失 |
| 2 | 責務逸脱 | `controller_comm.cpp` が `esp_netif_create_default_wifi_sta()` を呼ぶ | 通信モジュールが WiFi インフラを隠蔽、起動順序が追えない |
| 3 | 共有 I2C bus の所有が中途半端 | `init.cpp:64` の file-scope `s_i2c_bus` + `getI2CBus()` | 整理した部分とセンサ extern global が並存 |
| 4 | センサ外 extern | `globals.hpp:97-98` `g_baro_reference_altitude` | センサオブジェクトと状態が分離、ライフサイクル不明 |
| 5 | `volatile bool` の同期 | `globals.hpp:158-159` `g_eskf_ready`, `g_boot_complete` | atomic 保証なし、スレッド間 hand-off が脆い |
| 6 | Phase 1 が main.cpp 直書き | `main.cpp:524-550` | 順序変更が複数ファイルに波及 |
| 7 | fail-through が暗黙 | `init.cpp:124,140,159` | センサ init 失敗を WARN だけで続行、後続で null check 必須 |

### 業界標準パターンからの示唆

| フレームワーク | 借用する思想 | 借用しない理由 |
|--------------|------------|------------|
| **Zephyr RTOS** | parent-child バス所有モデル、init level による順序明示 | devicetree は ESP-IDF と二重メンテになる |
| **Mbed OS** | （特になし） | コンストラクタで失敗を返せない設計は教育的に NG |
| **Arduino** | 「ユーザーコードの簡潔さ」（L0 Workshop API のヒント） | グローバル `Wire` はマルチタスク/ISR で破綻 |

**結論**: Zephyr 思想（所有関係 + init 順序）を採用しつつ、ESP-IDF / FreeRTOS のネイティブ機構（`i2c_master_bus_handle_t`, `esp_err_t`, FreeRTOS task）はそのまま使うハイブリッド。

---

## 3. sf_board の責務

`sf_board` は vehicle における Board Support Package（BSP）であり、**共有ハードウェア資源の唯一の所有者**。

### 所有する資源

| 資源 | ESP-IDF 型 | 借用する HAL / Service |
|------|-----------|---------------------|
| I2C master bus | `i2c_master_bus_handle_t` | sf_hal_bmp280, sf_hal_bmm150, sf_hal_vl53l3cx, sf_hal_power |
| SPI host (IMU) | `spi_host_device_t` | sf_hal_bmi270 |
| SPI host (Flow) | `spi_host_device_t` | sf_hal_pmw3901 |
| LEDC timer | `ledc_timer_t` | sf_hal_motor, sf_hal_led, sf_hal_buzzer |
| esp_netif (STA) | `esp_netif_t*` | sf_comm（WiFi/ESP-NOW）, sf_telemetry（UDP socket） |
| Default event loop | （esp_event_loop） | sf_comm（WiFi/IP イベント） |
| NVS default partition | （nvs handle） | sf_calibration, params |

### 公開 API（namespace 規約）

```cpp
namespace sf::internal::board {
  // === Initialization ===
  esp_err_t init();   // 起動時 1 回、main.cpp Phase 1 で呼ぶ。冪等ではない

  // === Bus / handle accessors ===
  i2c_master_bus_handle_t i2c_bus();
  spi_host_device_t       imu_spi();
  spi_host_device_t       flow_spi();
  ledc_timer_t            motor_timer();

  // === Sensor presence query (Optional 分類用) ===
  enum class SensorId { Mag, FrontToF, /* … */ };
  bool sensor_present(SensorId id);
}
```

**namespace 規約（R8）:**
- `sf::internal::board::*` は **L3 ユーザー専用**（ファーム実装者・拡張者）
- L0/L1/L2 ユーザーは `sf_board.hpp` を include しない
- 学習者向けには `sf::api` namespace のみ公開し、`sf::internal` を意識しない設計

---

## 4. 起動シーケンス

### main.cpp の宣言的構造

```cpp
extern "C" void app_main() {
  ESP_LOGI(TAG, "=== vehicle boot ===");

  // ===== Phase 0: pre-kernel resources =====
  // NVS 単独で先に初期化（WiFi 設定読み込み等で必要）
  ESP_ERROR_CHECK(nvs_flash_init_with_fallback());

  // ===== Phase 1: BSP init — all shared HW =====
  // I2C / SPI / LEDC / netif / event_loop / sensor HAL / actuator HAL を順次
  ESP_ERROR_CHECK(sf::internal::board::init());

  // ===== Phase 2: Pub-Sub topics =====
  // 全 12+ Topic のバッファ・mutex を準備（タスク作成前に必須）
  sf::topics_init();

  // ===== Phase 3: parameter loading from NVS =====
  // params.cpp の table[] 全パラメータを NVS から復元、未保存値はデフォルト
  sf::params::init();

  // ===== Phase 4: tasks =====
  // 16 タスクを優先度付きで生成、ImuTask が pipeline を駆動
  sf::tasks::start_all();

  // ===== Phase 5: application (L1 app hook) =====
  // 全標準タスク起動後にアプリの追加タスクを起動（既定は何もしない）
  sf::app::start();

  ESP_LOGI(TAG, "=== boot complete ===");
}
```

### sf::internal::board::init() 内部の段階構造（Zephyr 流 init level）

```
Level 0: Pre-kernel resources
  - NVS は app_main 側で先に呼ばれる前提
  - default event loop の生成（`esp_event_loop_create_default()`）
  - esp_netif_init()

Level 1: Bus peripherals
  - I2C master bus（GPIO3=SDA, GPIO4=SCL, glitch filter, internal pullup）
  - I2C バス回復: バス生成の直後、どのセンサにも触る前に `i2c_master_bus_reset()` を 1 回呼ぶ（SCL を最大 9 回振って STOP を出し、通信の途中で止まったスレーブを解放する。マイコンだけが再起動してセンサが通電したままの場合に備える）。失敗しても起動は続け、結果を 1 行ログに出す
  - SPI host (IMU)
  - SPI host (Flow)
  - LEDC timer (PWM 周波数 150kHz, 8-bit 解像度)

Level 2: Critical sensors / actuators
  - BMI270 IMU 初期化 → 失敗時は abort()
  - Motor HAL 初期化 → 失敗時は abort()
  - VL53L3CX Bottom ToF 初期化 → **ARM の条件（モードを問わない）**（要件: ToF の出力停止中は ACRO を含め次の ARM を拒否する。実行時の出力停止の扱いは `detailed_design.md` §3 注12）。失敗しても起動は止めない（TofTask が `sensor_present(BottomToF) = false` にして続行し、ARM されていない間、XSHUT でリセットして初期化をやり直し続ける。ARM は ToF の出力停止として拒否される）

Level 3: Optional sensors
  - BMM150 Mag → 失敗時は sensor_present(Mag) = false で続行
  - VL53L3CX Front ToF → 失敗時は sensor_present(FrontToF) = false
  - PMW3901 Flow → 失敗時は sensor_present(Flow) = false
  - BMP280 Baro → 床基準の気圧高度（観測専用）、失敗時は flag-only

Level 4: Communication infrastructure
  - WiFi STA netif の生成（sf_board が所有・生成。R1: esp_netif の唯一の所有者は
    sf_board。sf_comm は board::sta_netif() を借用し esp_wifi_init/start のみ行う）

Level 5: Health publish
  - sensor_health Topic に初期 snapshot を publish
  - sensor_present() の結果と各 HAL の last_init_status を集約
```

### 起動順序の依存関係

```
nvs_flash_init  ──→  esp_event_loop  ──→  esp_netif_init
                                                │
                                                ▼
                                    esp_netif_create_default_wifi_sta
                                    (sf_board が生成・所有; sf_comm は
                                     後で esp_wifi_init/start のみ)

i2c_master_bus  ─┬─→  BMP280  ─┐
                 ├─→  BMM150   │
                 ├─→  VL53L3CX │
                 └─→  Power    └─→ sensor_present 集約 ─→ sensor_health publish

spi_imu        ─→  BMI270    ─→ Critical 確定
spi_flow       ─→  PMW3901   ─→ Optional 確定
ledc_timer     ─→  Motor / LED / Buzzer
```

各 HAL は `sf::internal::board::i2c_bus()` 等の getter から bus handle を取得し、自身の Config に渡して `init()` を呼ぶ。

---

## 5. 失敗の 3 段階分類

### 分類リスト

| 分類 | HW 要素 | 失敗時の挙動 | 理由 |
|------|--------|-----------|------|
| **Critical** | BMI270 IMU | `abort()` + LED 赤点滅 | 姿勢推定不可で飛行不能 |
| Critical | Motor HAL | `abort()` + LED 赤点滅 | 推力を出せない |
| Critical | NVS | `abort()` | パラメータ・キャリブを読めない |
| Critical | I2C bus / SPI host / LEDC timer | `abort()` | 全 HAL が依存する基盤 |
| **Optional** | BMM150 Mag | 初期化を最大 3 回（試行の間 20 ms。`config::OPTIONAL_SENSOR_INIT_MAX_ATTEMPTS`・`OPTIONAL_SENSOR_INIT_RETRY_DELAY_MS`、共通関数 `sf::initOptionalI2cSensor()`）試み、全回失敗して初めて `sensor_present(Mag) = false` で続行 | ヨー推定がドリフトしやすくなるが ACRO/STAB は可能 |
| Optional | VL53L3CX Front ToF | `sensor_present(FrontToF) = false` | 障害物検知不能、フライトは可能 |
| Optional | PMW3901 Flow | `sensor_present(Flow) = false` | 位置推定の補助（ホバーは ToF が鉛直を担う。気圧計は観測専用） |
| Optional | BMP280 Baro | Mag と同じ有限回の初期化再試行の後、全回失敗なら `sensor_present(Baro) = false` | 観測専用（制御・推定は ToF のみ）。無くても飛行は変わらない |
| Optional | Power monitor（INA3221） | Mag と同じ有限回の初期化再試行の後、全回失敗なら `sensor_present(Power) = false` | 電圧監視なしでも飛行は可能（バッテリ警告は失われる） |
| **Recoverable** | VL53L3CX Bottom ToF（起動時の初期化失敗、または出力停止） | ARM されていない間（INIT・IDLE_GROUND・IDLE_HELD）だけ、TofTask が XSHUT（GPIO 7）でリセットして初期化・測距開始をやり直す。最初の試行は出力停止が 2 s 続いた後、以後出力停止が続く間 10 s ごと（`kTofReinitSilenceUs`・`kTofReinitRetryIntervalUs`）。判断は純関数 `sf::mayReinitializeToF()`。ARM 中は決して試行しない。出力停止の間、ARM は `ArmBlock::TofSilent` で拒否される（モードを問わない、`detailed_design.md` §3 注12） | 高度推定の主観測。ACRO / STABILIZE の制御則は高度を使わないが、高度センサが保証できない機体は離陸させない。センサの電源・バスの一時的な喪失はリセットで復帰しうるので、電源の入れ直しを要求しない。飛行中は、ToF なしの着陸が働く |
| Recoverable | I2C / SPI 一時的 I/O エラー（起動時の初期化を含む） | task ループ内で retry、初期化時は上記の有限回の再試行（Optional な I2C センサ）、`READ_FAIL_LOG_INTERVAL = 400` cycles で警告抑制 | 振動や電源ノイズで稀に発生、再試行で復帰可能 |
| Recoverable | ESP-NOW パケットロス | failsafe 側で `command_setpoint` の age を判定 | 通信ロスは飛行中に発生しうる、タイムアウト Failsafe で対応 |

#### Optional な I2C センサの初期化再試行

BMM150・BMP280・INA3221 の初期化は、起動時の一時的な I2C エラーで「センサなし」に落ちないよう、有限回だけ再試行する。再試行は各タスク（MagTask・BaroTask・PowerTask）の先頭で、共通関数 `sf::initOptionalI2cSensor()`（`tasks/optional_sensor_init.hpp`）が行う。

| 項目 | 内容 |
|------|------|
| 試行回数・待ち | 最大 3 回（初回を含む）、試行の間 20 ms（`vTaskDelay`） |
| 失敗した試行 | WARN 1 行: センサ名・試行番号／最大回数・エラー名 |
| 2 回目以降で成功 | INFO 1 行: センサ名・何回目で成功したか（一時的な失敗だったことが起動ログから分かる） |
| 全回失敗 | Optional 無効（`sensor_present = false`）で起動を続ける。最後の試行のエラー名を WARN に出す |
| ドライバの条件 | 各 HAL の `init()` は失敗時に I2C デバイスハンドルを外して初期状態へ戻すので、再度呼んでも二重登録・ハンドル漏れが起きない |

400 Hz のタスク（ImuTask・ControlTask）と ToF（下向き・前方）の初期化・再初期化の経路は、この再試行の対象外である。

#### 前方 ToF と電池の接続（未解決の課題）

状態: **未対応**（2026-11-21 に記録。対応は後日）。

前方 ToF の基板は電池アダプタを兼ねており、電池を付けた状態でも外した状態でも機体に取り付けられる。電池が接続されていないと動作が不安定になり、デバイス ID などは読めることがあるが、測距はほぼできない（ハードウェアについての開発者の説明、2026-11-21）。

回路図で確かめたこと（主基板の回路図 `docs/hardware/schematics/Stamp_Fly_v1.0.pdf`、2024-06-05 作成。2026-11-21 に読み取り）:

| 項目 | 回路図の内容 |
|------|-------------|
| 前方 ToF の基板が入るコネクタ P3（2×4 ピン） | ピン 1・3 = `VBAT`、ピン 5・7 = GND、ピン 2 = SCL（G4）、ピン 4 = SDA（G3）、ピン 6 = `EXT_G1`（G8）、ピン 8 = `EXT_XSHUT`（G9）。**`+3.3V` は出ていない** |
| 電池の入口 | P3 の `VBAT` だけ。主基板にほかの電池コネクタは無い（電池は前方 ToF の基板を通って入る） |
| `VBAT` から先 | 逆接続保護の P チャネル MOSFET（Q1）→ 電流検出抵抗 R12（0.01 Ω、INA3221 のチャネル 2）→ `VBAT_IN`（モータ、および D3 を通って 5 V） |
| `+3.3V` | M5StampS3 の 3V3 ピンと同じネット。主基板に 3.3 V のレギュレータは無い。下向き ToF・地磁気・気圧・IMU・電流計・I2C のプルアップ（各線 4.7 kΩ × 2）はこの `+3.3V` につながる |
| USB の 5 V から電池側へ | D3（ショットキー）は `VBAT_IN` から 5 V の向きで、USB の 5 V から `VBAT_IN` へ順方向に流れる経路は無い |

したがって、前方 ToF の基板が受け取る電源は電池の電圧だけで、電池が無ければ正規の電源が無い。主基板上のセンサは `+3.3V`（USB でも供給される）で動くので、USB 給電だけでも動く。これは下の観測と合う。前方 ToF の基板の回路図（`docs/hardware/schematics/StampS3_Fly_Hat.pdf`、2023-09-22 作成。2026-11-21 に読み取り）:

| 項目 | 回路図の内容 |
|------|-------------|
| 載っている部品 | VL53L3CX（U1）、3.3 V のレギュレータ HT7533（U2、イネーブル端子なし）、電池コネクタ J1（2 ピン）、主基板の P3 と対になるコネクタ P1（2×4 ピン）、赤色 LED（D1、10 kΩ 経由で基板の 3.3 V へ） |
| 電池の経路 | J1 の電池の正極は `VBAT` に直結で、そのまま P1 から主基板へ渡る。途中にスイッチ・ヒューズ・ダイオードは無い |
| センサの電源 | `VBAT` → HT7533 → この基板の 3.3 V → VL53L3CX の電源ピン。この 3.3 V は P1 に出ておらず、主基板の `+3.3V` とは別のネット。**源は電池だけ** |
| プルアップ | SCL・SDA・GPIO1・XSHUT の 4 本が、各 4.7 kΩ でこの基板の 3.3 V へつながる。レベル変換は無い |

電池が無いときに起きること（回路図からの推論）:

- 前方 ToF には正規の電源が無い（根拠は強い）。
- ただし信号線から電源が回り込む。主基板の `+3.3V` → I2C のプルアップ（1 線あたり 2.35 kΩ）→ SCL・SDA → 前方 ToF の基板のプルアップ（4.7 kΩ）→ その基板の 3.3 V、およびマイコンの XSHUT 出力（G9）→ 4.7 kΩ → 同じ 3.3 V。配線上は確実につながっており、合成の抵抗は約 2 kΩ、流せる電流は 1 mA に満たない。
- この回り込みでセンサが起動して ID やアドレスの付け替えには応答し、測距を始めて電流が増えると電圧が落ちて止まる、と読める。「初期化は成功と表示され、読み出しは全て失敗する」という観測と合う（センサ側の消費電流はデータシートの一般的な値からの推定で、実測していない）。
- 前方 ToF の基板のプルアップは電源の落ちたレールにつながるので、共有の I2C バス（下向き ToF・電流計・地磁気・気圧と同じ SCL・SDA）の H レベルを引き下げる向きに働く（影響の大きさは未確認）。

USB 給電だけのとき電流計が 1.7〜1.9 V を読む原因は、回路図では決まらない。候補は、前方 ToF の基板の 3.3 V から HT7533 を逆向きに通って `VBAT` へ出る経路と、主基板の D3 の逆方向の漏れ電流の 2 つで、前方 ToF の基板を外して USB だけで同じ値を読めば判別できる（**未確認**）。P1 と P3 のピン番号は奇数と偶数が入れ替わって描かれており、嵌合の向きによるものかは未確認。

実機で確かめたこと（2026-11-20〜21、コミット 53553678）:

| 条件 | 観測 |
|------|------|
| USB 給電のみで起動（電池なし） | 前方 ToF の初期化は成功と表示される（`VL53L3CX (FRONT) initialized at address 0x31`、`front ready`）。その後の読み出しは全て `I2C read failed at 0x0031: ESP_ERR_INVALID_STATE` で失敗し、30 回連続（約 1 s、`kFrontDisableFailCycles`）で無効になる（`front failed 30 reads in a row — disabled`、`sensors : present=0x3F`） |
| その後に電池を接続 | 機体は再起動せず、前方 ToF は無効のまま残る（2 回中 2 回）。`prearm  : ready`、LED は緑の常灯で、正常なときと区別できない |
| 動作中に電池を外す（USB は接続のまま） | 同じ失敗の後に無効になり、電池を戻しても戻らない |
| 電池を先に接続して起動 | 前方 ToF は正常（`present=0x7F`、2 回中 2 回） |
| マイコンのリセット（PC がシリアルポートを開く）の後、電池あり | 前方 ToF は正常に戻る |

課題:

| # | 課題 | 現状 | 検討の出発点 |
|---|------|------|------------|
| 1 | USB で通電した後に電池を接続したとき、前方 ToF が再び正常に読めるようにする | 無効にした後は次のリセットまで再試行しない（`tof_task.cpp` の `trackFrontFailure`・`holdFrontInReset`） | 下向き ToF の非 ARM 中の再初期化（Recoverable、`reinitBottom`）と同じ考え方を前方にも使えるか。きっかけの候補は電池電圧の立ち上がり（PowerTask の値）。ARM 中は行わない |
| 2 | 電池が無い状態で前方 ToF が取り付けられているとき、前方 ToF を正しく無効と判断する | 初期化が成功と表示された後、30 回の読み出し失敗を待って無効にしている。その間コンソールにエラーが約 30 行出る | 初期化の時点で電池電圧を見て判断するか、最初の測距が成立してから ready とするか。前方 ToF の電源は電池の電圧だけ（回路図）なので、電流計が読む電池電圧は「前方 ToF に電源があるか」の判断に使える。デバイス ID が読めることは判断の根拠にならない |

関連: 前方 ToF が無効であることを ARM の前に知らせる手段が無い（`prearm` も LED も変わらない）。課題 1・2 と合わせて扱う。

### LED エラーパターン（Critical 失敗時）

`sf_board::init()` が abort 経路に入ったとき、LED が以下のパターンを表示してユーザーに伝える。

| エラー | パターン | 周波数 |
|------|---------|------|
| IMU init 失敗 | 赤色高速点滅 | 5 Hz |
| Motor HAL init 失敗 | 赤+紫交互 | 2 Hz |
| その他 Critical | 赤色低速点滅 | 1 Hz |

abort 後は `esp_restart()` を呼ばず、LED 表示のまま停止する（学習者が原因を読み取れる時間を確保）。watchdog 設定は `coding_and_education.md` 側で議論する。

**実装状況:** Critical 失敗時の **halt（停止）は実装済み** — `sf_board` の Critical バス/タイマ失敗は `board::init()` 内の `fatal()` が `vTaskDelay(portMAX_DELAY)` ループで停止し（`esp_restart` を呼ばない＝上記方針どおり）、IMU/モータの Critical 失敗は所有タスク（`imu_task` / `sf_actuator`）が同様に halt する。**LED エラーパターンの表示は Phase 6 に繰延**（LED/notify の所有確立後に配線）。それまで原因はシリアルログの `CRITICAL: …` 行で報告する。理由: LED ドライバ（`sf_hal_led`, WS2812/RMT）は現状どのタスクも初期化・所有しておらず（`notify_task` は代替実装）、`sf_board` に LED 依存を先行追加すると実機ブリングアップ直前のビルドリスクになるため。

---

## 6. HAL との接続規約

### 2 段階初期化（construct → init）

Mbed OS スタイルの「コンストラクタで失敗を返せない」を避けるため、全 HAL は **construct → init() の 2 段階** とする。

```cpp
// HAL 側の典型的な API (sf_hal_vl53l3cx の例)
namespace stampfly {
  class VL53L3CXWrapper {
   public:
    VL53L3CXWrapper() = default;          // Construct: 失敗できない、リソース取得なし
    esp_err_t init(const Config& cfg);   // Init: bus handle を含む Config を受けて初期化
    ~VL53L3CXWrapper();                   // Destruct: 取得済みリソース解放
  };
}
```

### Config による依存性注入

各 HAL は `Config` 構造体経由で bus handle や GPIO を受け取る（extern グローバル禁止、R2）。

```cpp
// tof_task.cpp での使用例
static stampfly::VL53L3CXWrapper tof_bottom;

void TofTask(void*) {
  auto cfg = stampfly::VL53L3CXWrapper::Config::defaultBottom(
      sf::internal::board::i2c_bus()  // BSP から bus を借りる
  );
  if (tof_bottom.init(cfg) != ESP_OK) {
    // sf_board::init() の Level 2/3 で既に判定されているはず
    // ここで失敗するのは task 起動後の異常
  }
  // … main loop
}
```

### task ローカル所有

センサ wrapper のインスタンスは **task のファイルスコープ static** で持つ（task の外から見えない）。これにより：

- 旧 vehicle/ の `g_imu` / `g_baro` 等の extern グローバル乱立を排除
- センサオブジェクトとその状態（last_value, fail_count など）が同じスコープで完結
- Topic Pub-Sub 経由でデータが流れるため、他 task から wrapper に直接アクセスする必要がない

---

## 7. namespace 規約

### 4 階層アクセスと namespace の対応

| 層 | namespace | 公開ヘッダ | 学習者からの見え方 |
|----|---------|---------|------------------|
| L0 | `ws::` | `ws_api.hpp` | Workshop API、`ws::motor_set_duty()` 等 |
| L1 | `sf::api::` | `sf_api.hpp` | Topic / params / state、`sf::api::sensor_imu` 等 |
| L2 | `stampfly::` | 各 HAL の `*_wrapper.hpp` | HAL クラス、`stampfly::BMI270Wrapper` 等 |
| L3 | `sf::internal::` | `sf_board.hpp` 等 | BSP getter、bus handle 取得 |

### include の規約

学習者向け Examples / Workshop コードは **次の include だけで完結する** ように設計する。

```cpp
// L0 学習者
#include "ws_api.hpp"          // ws::* のみ

// L1 学習者
#include "sf_api.hpp"           // sf::api::* のみ（Topic, params, state）

// L2 学習者
#include "sf_api.hpp"
#include "bmi270_wrapper.hpp"  // 個別 HAL を直接

// L3 ファーム実装者
#include "sf_api.hpp"
#include "sf_board.hpp"        // BSP getter
#include "bmi270_wrapper.hpp"
```

`sf::internal` を `sf::api` ヘッダから transitive に exposure しないことを徹底する（forward declaration、PIMPL を活用）。

---

## 8. 既存 Phase 2a 実装の手直し計画（M2 で実施）

vehicle の Phase 2a で既に結合済みの IMU / Motor / ESP-NOW / UDP は、本文書の方針に合わせて以下を手直しする。M2 ブランチで実施。

| ファイル | 現状 | M2 で実施する変更 |
|---------|-----|----------------|
| `main/main.cpp` | Phase 1 が TODO コメント | `sf::internal::board::init()` 1 行に集約 |
| `tasks/imu_task.cpp` | `Config::defaultStampFly()` (引数なし) | `Config::defaultStampFly(board::imu_spi())` に変更 |
| `components/sf_actuator/actuator.cpp` | LEDC 初期化を内部で実行 | LEDC timer は board が用意、HAL は channel のみ生成 |
| `components/sf_comm/comm.cpp` | `esp_netif_init` / `esp_event_loop_create_default` を内部で呼ぶ | board 側で済んだ前提に変更、WiFi STA / ESP-NOW 実体のみ所有 |
| `components/sf_telemetry/telemetry.cpp` | `waitForWifi()` polling 待ち | `sf_comm` のイベント通知で起動 |

これらの変更が完了した時点で M2 を main にマージ、その後 M3（Phase 2.2 ToF）で新規センサを board の方式で結合する。

---

<a id="english"></a>

## 1. About This Document

> **Status:** This document defines hardware initialization design for vehicle. The Japanese section above is the authoritative version. Full English translation is pending and will be completed in M1c (educational documentation milestone).

This document specifies:
- Ownership of shared HW resources (I2C/SPI buses, esp_netif, event loop, NVS)
- Initialization sequence (Phase 0 → Phase 5 in `app_main()`; Phase 5 = `sf::app::start()`, the L1 application hook)
- Failure classification (Critical / Optional / Recoverable)
- HAL connection contract (Config-based DI, 2-stage init)
- Namespace boundaries for the 4-tier learner access (L0 / L1 / L2 / L3)

**Bottom ToF classification (Recoverable while disarmed).** A bottom ToF whose bring-up fails at boot, or that delivers no sample for 2 s, is reset through XSHUT (GPIO 7) and initialised again by TofTask, as long as the craft is not armed (INIT / IDLE_GROUND / IDLE_HELD): the first attempt after 2 s of silence, then every 10 s while the sensor stays silent (`kTofReinitSilenceUs`, `kTofReinitRetryIntervalUs`). The decision is the pure function `sf::mayReinitializeToF()` (`tof_silence_policy.hpp`); in every armed state nothing about the sensor changes. While the sensor is silent ARM is refused in every mode (`ArmBlock::TofSilent`; `detailed_design.md` §3 note 12). The task never deletes itself, and a failed bring-up does not stop the boot.

**I2C bus recovery at boot.** Right after the I2C master bus is created and before any sensor is touched, sf_board calls `i2c_master_bus_reset()` once (it clocks SCL up to nine times and issues a STOP, releasing a slave that was stopped in the middle of a transaction while only the MCU restarted and the sensors stayed powered). A failure does not stop the boot; the result is logged on one line.

**Optional I2C sensors retry their initialisation.** BMM150 (Mag), BMP280 (Baro) and INA3221 (Power) are initialised up to 3 times (first try included, 20 ms between tries; `config::OPTIONAL_SENSOR_INIT_MAX_ATTEMPTS`, `OPTIONAL_SENSOR_INIT_RETRY_DELAY_MS`) by the shared function `sf::initOptionalI2cSensor()` (`tasks/optional_sensor_init.hpp`) at the top of MagTask, BaroTask and PowerTask. Each failed try logs one WARN line (sensor, try number / maximum, error name); a success on the second try or later logs one INFO line saying which try succeeded. Only when every try fails is the sensor treated as absent (Optional, `sensor_present = false`). Each HAL `init()` releases its I2C device handle on failure, so calling it again neither registers the device twice nor leaks a handle. ImuTask, ControlTask and the ToF bring-up / re-initialisation path are not affected.

**Front ToF and the battery (open issue, not yet addressed; recorded 2026-11-21).** The front ToF board doubles as the battery adapter and can be mounted with or without a battery. It is unstable without the battery: the device ID may still be read, but ranging is almost impossible (the developer's description of the hardware, 2026-11-21). Checked in the schematic of the main board (`docs/hardware/schematics/Stamp_Fly_v1.0.pdf`, created 2024-06-05; read on 2026-11-21): the connector P3 that takes the front ToF board (2x4 pins) carries `VBAT` on pins 1 and 3, GND on pins 5 and 7, SCL (G4), SDA (G3), `EXT_G1` (G8) and `EXT_XSHUT` (G9), and **no `+3.3V`**; P3 is the only battery input of the main board (the battery enters through the front ToF board); from `VBAT` the path is the reverse-polarity P-channel MOSFET Q1, the shunt R12 (0.01 ohm, INA3221 channel 2) and `VBAT_IN` (motors, and 5 V through D3); `+3.3V` is the same net as the 3V3 pin of the M5StampS3 and the main board has no 3.3 V regulator, so the bottom ToF, magnetometer, barometer, IMU, power monitor and the I2C pull-ups (two 4.7 kohm per line) run from it also on USB power; D3 points from `VBAT_IN` to 5 V, so USB cannot feed `VBAT_IN` in the forward direction. The front ToF board therefore receives only the battery voltage and has no proper supply without a battery, which agrees with the observations below; the battery voltage read by the power monitor can serve to judge whether the front ToF has power. The schematic of the front ToF board (`docs/hardware/schematics/StampS3_Fly_Hat.pdf`, created 2023-09-22; read on 2026-11-21) shows a VL53L3CX (U1), a 3.3 V regulator HT7533 (U2, no enable pin), the battery connector J1 (2 pins), the connector P1 (2x4) that mates with P3, and a red LED; the battery positive goes straight to `VBAT` and on to the main board with no switch, fuse or diode; `VBAT` feeds the HT7533, whose 3.3 V output powers the sensor, is not brought out on P1 and is a different net from the `+3.3V` of the main board, so **its only source is the battery**; SCL, SDA, GPIO1 and XSHUT are each pulled up with 4.7 kohm to that local 3.3 V, with no level shifter. Inferred from the drawings for the case without a battery: the sensor has no proper supply, but the signal lines back-feed its rail (main-board `+3.3V` through the I2C pull-ups of 2.35 kohm per line, SCL / SDA and the 4.7 kohm pull-ups of the front ToF board, and the XSHUT output G9 through 4.7 kohm), about 2 kohm in total and less than 1 mA; that is enough for the sensor to answer the ID and the address change and not enough once ranging starts, which agrees with 'initialisation reports success, every read fails' (the sensor currents are typical datasheet values, not measured); the pull-ups of the unpowered board also pull the shared I2C lines down (size not confirmed). The cause of the 1.7-1.9 V that the power monitor reads on USB power alone is not decided by the drawings: either a path from the local 3.3 V backwards through the HT7533 to `VBAT`, or reverse leakage of D3 on the main board; reading the same value with the front ToF board removed would tell (**not confirmed**). The pin numbers of P1 and P3 are drawn with odd and even swapped; whether that is the mating orientation is not confirmed. Measured on hardware (2026-11-20 to 21, commit 53553678): booting on USB power alone, the front ToF initialisation reports success (`VL53L3CX (FRONT) initialized at address 0x31`, `front ready`), then every read fails with `I2C read failed at 0x0031: ESP_ERR_INVALID_STATE` and after 30 failures in a row (about 1 s, `kFrontDisableFailCycles`) it is disabled (`sensors : present=0x3F`); connecting the battery afterwards does not restart the vehicle and the front ToF stays disabled (2 of 2), while `prearm  : ready` and the solid green LED look the same as in the healthy case; removing the battery while running (USB still connected) ends the same way; booting with the battery connected first gives a healthy front ToF (`present=0x7F`, 2 of 2); after an MCU reset with the battery connected it is healthy again. Two issues are open. (1) Make the front ToF readable again when the battery is connected after power-up on USB: today nothing retries until the next reset (`trackFrontFailure`, `holdFrontInReset` in `tof_task.cpp`); a starting point is the disarmed-only re-initialisation used for the bottom ToF (`reinitBottom`), triggered for example by the battery voltage rising, never while armed. (2) Judge the front ToF as disabled correctly when it is mounted without a battery: today the initialisation reports success and 30 failed reads (about 30 error lines on the console) are needed; a starting point is to look at the battery voltage at initialisation, or to report ready only after a first valid measurement; a readable device ID is not evidence. Related: nothing shows a disabled front ToF before arming (neither `prearm` nor the LED changes); handle it together with (1) and (2).

See the Japanese section for complete content.
