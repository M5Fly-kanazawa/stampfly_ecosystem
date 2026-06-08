# vehicle_new Hardware Initialization Design
# vehicle_new ハードウェア初期化設計

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 本文書の位置づけ

### このドキュメントについて

vehicle_new における **共有ハードウェア資源の所有権・初期化フロー・失敗時の挙動・HAL との接続規約** を定義する。具体的には次の問いに答える。

- I2C バスや SPI バスのような共有 HW 資源を誰が所有するか
- センサ・アクチュエータ・通信スタックの初期化はどの順序で行うか
- センサ init が失敗したらどう振る舞うか（飛行を止めるか、機能無効で続けるか）
- 学習者・ファーム実装者が触る範囲（namespace 境界）はどこか

設計文書 [`architecture.md`](architecture.md) §7 の要約に対する詳細版である。

### 対象読者

- vehicle_new ファームの実装者・拡張者
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
| **Arduino** | 「ユーザーコードの簡潔さ」（L0 Sketch API のヒント） | グローバル `Wire` はマルチタスク/ISR で破綻 |

**結論**: Zephyr 思想（所有関係 + init 順序）を採用しつつ、ESP-IDF / FreeRTOS のネイティブ機構（`i2c_master_bus_handle_t`, `esp_err_t`, FreeRTOS task）はそのまま使うハイブリッド。

---

## 3. sf_board の責務

`sf_board` は vehicle_new における Board Support Package（BSP）であり、**共有ハードウェア資源の唯一の所有者**。

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
  ESP_LOGI(TAG, "=== vehicle_new boot ===");

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
  // 14 タスクを優先度付きで生成、ImuTask が pipeline を駆動
  sf::tasks::start_all();

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
  - SPI host (IMU)
  - SPI host (Flow)
  - LEDC timer (PWM 周波数 150kHz, 8-bit 解像度)

Level 2: Critical sensors / actuators
  - BMI270 IMU 初期化 → 失敗時は abort()
  - Motor HAL 初期化 → 失敗時は abort()
  - VL53L3CX Bottom ToF 初期化 → ALT/POS で必須、失敗時は abort()

Level 3: Optional sensors
  - BMM150 Mag → 失敗時は sensor_present(Mag) = false で続行
  - VL53L3CX Front ToF → 失敗時は sensor_present(FrontToF) = false
  - PMW3901 Flow → 失敗時は sensor_present(Flow) = false
  - BMP280 Baro → 高度推定の補助、失敗時は flag-only

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
| Critical | VL53L3CX Bottom ToF | `abort()`（ALT/POS モード使用時のみ） | 高度推定の主観測 |
| **Optional** | BMM150 Mag | `sensor_present(Mag) = false` で続行 | ヨー推定がドリフトしやすくなるが ACRO/STAB は可能 |
| Optional | VL53L3CX Front ToF | `sensor_present(FrontToF) = false` | 障害物検知不能、フライトは可能 |
| Optional | PMW3901 Flow | `sensor_present(Flow) = false` | 位置推定の補助、ホバーは ToF + Baro で可能 |
| Optional | BMP280 Baro | `sensor_present(Baro) = false` | ToF と冗長、片方あれば高度推定可能 |
| Optional | Power monitor | `sensor_present(Power) = false` | 電圧監視なしでも飛行は可能（バッテリ警告は失われる） |
| **Recoverable** | I2C / SPI 一時的 I/O エラー | task ループ内で retry、`READ_FAIL_LOG_INTERVAL = 400` cycles で警告抑制 | 振動や電源ノイズで稀に発生、再試行で復帰可能 |
| Recoverable | ESP-NOW パケットロス | failsafe 側で `command_setpoint` の age を判定 | 通信ロスは飛行中に発生しうる、タイムアウト Failsafe で対応 |

### LED エラーパターン（Critical 失敗時）

`sf_board::init()` が abort 経路に入ったとき、LED が以下のパターンを表示してユーザーに伝える。

| エラー | パターン | 周波数 |
|------|---------|------|
| IMU init 失敗 | 赤色高速点滅 | 5 Hz |
| Motor HAL init 失敗 | 赤+紫交互 | 2 Hz |
| Critical ToF init 失敗 | 赤+橙交互 | 2 Hz |
| その他 Critical | 赤色低速点滅 | 1 Hz |

abort 後は `esp_restart()` を呼ばず、LED 表示のまま停止する（学習者が原因を読み取れる時間を確保）。watchdog 設定は `coding_and_education.md` 側で議論する。

**実装状況（Phase 4 時点）:** Critical 失敗時の **halt（停止）は実装済み** — `sf_board` の Critical バス/タイマ失敗は `board::init()` 内の `fatal()` が `vTaskDelay(portMAX_DELAY)` ループで停止し（`esp_restart` を呼ばない＝上記方針どおり）、IMU/モータの Critical 失敗は所有タスク（`imu_task` / `sf_actuator`）が同様に halt する。**LED エラーパターンの表示は Phase 6 に繰延**（LED/notify の所有確立後に配線）。それまで原因はシリアルログの `CRITICAL: …` 行で報告する。理由: LED ドライバ（`sf_hal_led`, WS2812/RMT）は現状どのタスクも初期化・所有しておらず（`notify_task` はスタブ）、`sf_board` に LED 依存を先行追加すると実機ブリングアップ直前のビルドリスクになるため。

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
| L0 | `ws::` | `ws_api.hpp` | Sketch API、`ws::motor_set_duty()` 等 |
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

vehicle_new の Phase 2a で既に結合済みの IMU / Motor / ESP-NOW / UDP は、本文書の方針に合わせて以下を手直しする。M2 ブランチで実施。

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

> **Status:** This document defines hardware initialization design for vehicle_new. The Japanese section above is the authoritative version. Full English translation is pending and will be completed in M1c (educational documentation milestone).

This document specifies:
- Ownership of shared HW resources (I2C/SPI buses, esp_netif, event loop, NVS)
- Initialization sequence (Phase 0 → Phase 4 in `app_main()`)
- Failure classification (Critical / Optional / Recoverable)
- HAL connection contract (Config-based DI, 2-stage init)
- Namespace boundaries for the 4-tier learner access (L0 / L1 / L2 / L3)

See the Japanese section for complete content.
