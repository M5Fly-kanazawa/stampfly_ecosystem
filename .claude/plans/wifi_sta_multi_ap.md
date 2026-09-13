# StampFly WiFi STA+APモード共存実装計画

## 実装ステータス

**✅ 実装完了 (2026-01-25)**

| Phase | ステータス | コミット |
|-------|----------|---------|
| Phase 1: STA基盤 | ✅ 完了 | 12eceea, 89d88ab |
| Phase 2: CLIコマンド | ✅ 完了 | 0928969 |
| Phase 3: 警告とエラーハンドリング | ✅ 完了 | 8c15f92 |
| ドキュメント整備 | ✅ 完了 | 3e861e2, 5b69155 |
| 実機テスト | ✅ 完了 | - |

**成果物:**
- `controller_comm.hpp/cpp` - Multi-AP対応WiFi STA実装
- `cmd_comm.cpp` - `wifi sta` CLIコマンド（6つのサブコマンド）
- `docs/wifi-sta-setup.md` - ユーザー向けセットアップガイド
- `docs/plans/WIFI_COMM_PLAN.md` - Phase 3追加

## 概要

StampFlyドローンにWiFi STAモードを追加し、既存のAPモードと共存させる。これにより、WiFiルーター経由でのROS2開発、インターネットアクセス、複数ドローン群制御、デバッグの利便性向上を実現する。

## 現状分析

**既存実装:**
- WiFiモード: `WIFI_MODE_APSTA` で設定済み（controller_comm.cpp:103）
- APモード: SSID="StampFly", IP=192.168.4.1, OPEN認証、動作中
- STAモード: netif未作成、設定未実施のため未アクティブ
- ネットワークサービス: Telnet CLI (port 23), WebSocket (port 80), UDP (8888/8889)

**ESP32 APSTA制約:**
- APとSTAは同じチャンネルを使用（STAが接続したAPのチャンネルに自動追従）
- チャンネル変更時、APも追従して変更される
- ESP-NOW通信は現在のチャンネルで実行

**ユーザー要件:**
- AP設定: 複数保存（研究室と自宅の少なくとも2つ）
- 優先順位: 保存順に接続試行（先に追加したAPが優先）
- 自動接続: 必須（起動時に優先順位順でWiFiルーターへ自動接続）
- 主な用途: ROS2開発、インターネット接続、群制御準備、デバッグ

---

## 実装方針

### 設計原則

1. **複数AP対応**: 最大5個のAP設定を保存（研究室、自宅など複数環境対応）
2. **優先順位自動試行**: 保存順に接続試行、最初に成功したAPに接続
3. **既存機能維持**: APモード、ESP-NOW、UDPサーバーに影響なし
4. **自動接続優先**: 起動時に自動的にルーターへ接続
5. **明示的なチャンネル管理**: STA接続時のチャンネル変更をログで明示

### Phase分け

- **Phase 1**: STA基盤実装（データ構造、イベントハンドラ、接続/切断）
- **Phase 2**: NVS永続化とCLIコマンド
- **Phase 3**: 自動接続とエラーハンドリング

---

## Phase 1: STA基盤実装

### 1.1 データ構造追加

**ファイル**: `firmware/vehicle/components/sf_svc_comm/include/controller_comm.hpp`

```cpp
class ControllerComm {
    // ... existing members ...

private:
    // STA設定構造体
    struct STAConfig {
        char ssid[32];
        char password[64];
        bool is_valid;
    };

    static constexpr int MAX_STA_CONFIGS = 5;  // 最大5個のAP設定

    // STA設定リスト（優先順位順）
    STAConfig sta_configs_[MAX_STA_CONFIGS] = {};
    int sta_config_count_ = 0;
    int current_sta_index_ = -1;  // 現在接続中のAP index (-1: 未接続)

    bool sta_auto_connect_ = true;  // デフォルトON（ユーザー要件）
    bool sta_connected_ = false;
    char sta_ip_addr_[16] = {0};

    // 接続試行管理
    int connection_attempt_index_ = 0;  // 次に試すAP index
    bool is_connecting_ = false;

    // イベントハンドラ
    esp_netif_t* sta_netif_ = nullptr;
    esp_event_handler_instance_t wifi_event_handler_ = nullptr;
    esp_event_handler_instance_t ip_event_handler_ = nullptr;
};
```

**公開インターフェース追加:**

```cpp
public:
    // STA設定管理（複数AP対応）
    esp_err_t addSTAConfig(const char* ssid, const char* password);
    esp_err_t removeSTAConfig(int index);
    esp_err_t removeSTAConfig(const char* ssid);
    int getSTAConfigCount() const { return sta_config_count_; }
    const STAConfig* getSTAConfig(int index) const;

    // STA接続管理
    esp_err_t connectSTA();  // 優先順位順に自動接続
    esp_err_t connectSTA(int index);  // 指定indexのAPに接続
    esp_err_t disconnectSTA();

    // STA状態取得
    bool isSTAConnected() const { return sta_connected_; }
    const char* getSTAIPAddress() const { return sta_ip_addr_; }
    int getCurrentSTAIndex() const { return current_sta_index_; }
    const char* getCurrentSTASSID() const;

    // 自動接続設定
    void setSTAAutoConnect(bool enable) { sta_auto_connect_ = enable; }
    bool isSTAAutoConnect() const { return sta_auto_connect_; }
```

### 1.2 イベントハンドラ実装

**ファイル**: `firmware/vehicle/components/sf_svc_comm/controller_comm.cpp`

**WiFiイベントハンドラ:**

```cpp
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    ControllerComm* self = static_cast<ControllerComm*>(arg);

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA started");
            break;

        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t* event = (wifi_event_sta_connected_t*)event_data;
            ESP_LOGI(TAG, "STA connected to SSID:%s, Channel:%d",
                     event->ssid, event->channel);

            // チャンネル変更を検出・ログ
            if (event->channel != self->config_.wifi_channel) {
                ESP_LOGW(TAG, "Channel changed: %d -> %d (AP follows STA)",
                         self->config_.wifi_channel, event->channel);
                self->config_.wifi_channel = event->channel;
            }
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
            ESP_LOGW(TAG, "STA disconnected from '%s' (reason:%d)",
                     self->current_sta_index_ >= 0 ?
                     self->sta_configs_[self->current_sta_index_].ssid : "unknown",
                     event->reason);
            self->sta_connected_ = false;
            self->is_connecting_ = false;

            // 自動再接続: 次のAPを試行
            if (self->sta_auto_connect_ && self->sta_config_count_ > 0) {
                self->connection_attempt_index_++;
                if (self->connection_attempt_index_ >= self->sta_config_count_) {
                    self->connection_attempt_index_ = 0;  // 最初に戻る
                    ESP_LOGI(TAG, "All APs tried, retrying from first AP...");
                    vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒待機
                }
                ESP_LOGI(TAG, "Trying next AP (index %d)...", self->connection_attempt_index_);
                self->connectSTA(self->connection_attempt_index_);
            }
            break;
        }
    }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    ControllerComm* self = static_cast<ControllerComm*>(arg);

    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "STA got IP: " IPSTR " (connected to '%s')",
                 IP2STR(&event->ip_info.ip),
                 self->current_sta_index_ >= 0 ?
                 self->sta_configs_[self->current_sta_index_].ssid : "unknown");

        snprintf(self->sta_ip_addr_, sizeof(self->sta_ip_addr_),
                 IPSTR, IP2STR(&event->ip_info.ip));
        self->sta_connected_ = true;
        self->is_connecting_ = false;

        // 接続成功したら、次回の試行は最初のAPから
        self->connection_attempt_index_ = 0;
    }
}
```

### 1.3 初期化処理の修正

**`init()` 関数の変更:**

```cpp
esp_err_t ControllerComm::init(const Config& config)
{
    ESP_LOGI(TAG, "Initializing ESP-NOW communication");
    ESP_LOGI(TAG, "  WiFi channel: %d", config.wifi_channel);

    config_ = config;
    s_instance = this;

    // STAネットワークインターフェース作成（追加）
    sta_netif_ = esp_netif_create_default_wifi_sta();

    // APネットワークインターフェース作成（既存）
    esp_netif_create_default_wifi_ap();

    // WiFi初期化（既存）
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&wifi_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // イベントハンドラ登録（追加）
    esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, this, &wifi_event_handler_);

    esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &ip_event_handler, this, &ip_event_handler_);

    // WiFiモード設定（既存、すでにAPSTA）
    ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // AP設定（既存）
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "StampFly");
    ap_config.ap.ssid_len = 8;
    ap_config.ap.channel = config.wifi_channel;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
    ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
    }

    // 電力節約無効化（既存）
    esp_wifi_set_ps(WIFI_PS_NONE);

    // WiFi起動（既存）
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // チャンネル設定（既存）
    ret = esp_wifi_set_channel(config.wifi_channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // STA設定リスト読み込み（追加）
    loadSTAConfigsFromNVS();

    // 自動接続フラグ読み込み（追加、デフォルトtrue）
    sta_auto_connect_ = loadSTAAutoConnectFromNVS();

    // 自動接続有効かつSTA設定済みなら接続試行（追加）
    if (sta_auto_connect_ && sta_config_count_ > 0) {
        ESP_LOGI(TAG, "Auto-connecting to STA (%d APs configured)...", sta_config_count_);
        connectSTA();  // 優先順位順に自動接続
    }

    // ESP-NOW初期化（既存、以下同じ）
    // ...

    initialized_ = true;
    ESP_LOGI(TAG, "ESP-NOW communication initialized");
    ESP_LOGI(TAG, "WiFi AP 'StampFly' started on channel %d (http://192.168.4.1)",
             config.wifi_channel);

    return ESP_OK;
}
```

### 1.4 STA設定管理関数

```cpp
esp_err_t ControllerComm::addSTAConfig(const char* ssid, const char* password)
{
    if (ssid == nullptr || password == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sta_config_count_ >= MAX_STA_CONFIGS) {
        ESP_LOGE(TAG, "STA config list full (max %d)", MAX_STA_CONFIGS);
        return ESP_ERR_NO_MEM;
    }

    // 重複チェック
    for (int i = 0; i < sta_config_count_; i++) {
        if (strcmp(sta_configs_[i].ssid, ssid) == 0) {
            ESP_LOGW(TAG, "SSID '%s' already exists at index %d", ssid, i);
            return ESP_ERR_INVALID_STATE;
        }
    }

    // 新しいAPを追加
    STAConfig& cfg = sta_configs_[sta_config_count_];
    strncpy(cfg.ssid, ssid, sizeof(cfg.ssid) - 1);
    strncpy(cfg.password, password, sizeof(cfg.password) - 1);
    cfg.is_valid = true;

    sta_config_count_++;
    ESP_LOGI(TAG, "Added STA config #%d: SSID=%s", sta_config_count_, ssid);
    return ESP_OK;
}

esp_err_t ControllerComm::removeSTAConfig(int index)
{
    if (index < 0 || index >= sta_config_count_) {
        return ESP_ERR_INVALID_ARG;
    }

    // 配列を詰める
    for (int i = index; i < sta_config_count_ - 1; i++) {
        sta_configs_[i] = sta_configs_[i + 1];
    }
    sta_config_count_--;

    // 現在接続中のindexを調整
    if (current_sta_index_ == index) {
        current_sta_index_ = -1;
        sta_connected_ = false;
    } else if (current_sta_index_ > index) {
        current_sta_index_--;
    }

    ESP_LOGI(TAG, "Removed STA config #%d", index);
    return ESP_OK;
}

esp_err_t ControllerComm::removeSTAConfig(const char* ssid)
{
    for (int i = 0; i < sta_config_count_; i++) {
        if (strcmp(sta_configs_[i].ssid, ssid) == 0) {
            return removeSTAConfig(i);
        }
    }
    ESP_LOGW(TAG, "SSID '%s' not found", ssid);
    return ESP_ERR_NOT_FOUND;
}

const ControllerComm::STAConfig* ControllerComm::getSTAConfig(int index) const
{
    if (index < 0 || index >= sta_config_count_) {
        return nullptr;
    }
    return &sta_configs_[index];
}

const char* ControllerComm::getCurrentSTASSID() const
{
    if (current_sta_index_ >= 0 && current_sta_index_ < sta_config_count_) {
        return sta_configs_[current_sta_index_].ssid;
    }
    return nullptr;
}
```

### 1.5 STA接続/切断関数

```cpp
esp_err_t ControllerComm::connectSTA()
{
    // 優先順位順（index 0から）に接続試行
    if (sta_config_count_ == 0) {
        ESP_LOGE(TAG, "No STA configs available");
        return ESP_ERR_INVALID_STATE;
    }

    connection_attempt_index_ = 0;
    return connectSTA(0);
}

esp_err_t ControllerComm::connectSTA(int index)
{
    if (index < 0 || index >= sta_config_count_) {
        ESP_LOGE(TAG, "Invalid STA config index: %d", index);
        return ESP_ERR_INVALID_ARG;
    }

    if (is_connecting_) {
        ESP_LOGW(TAG, "Already connecting, skipping...");
        return ESP_ERR_INVALID_STATE;
    }

    const STAConfig& cfg = sta_configs_[index];
    if (!cfg.is_valid) {
        ESP_LOGE(TAG, "STA config #%d is invalid", index);
        return ESP_ERR_INVALID_STATE;
    }

    wifi_config_t sta_config = {};
    strncpy((char*)sta_config.sta.ssid, cfg.ssid, 32);
    strncpy((char*)sta_config.sta.password, cfg.password, 64);
    sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Connecting to SSID '%s' (priority %d/%d)...",
             cfg.ssid, index + 1, sta_config_count_);

    current_sta_index_ = index;
    connection_attempt_index_ = index;
    is_connecting_ = true;

    return esp_wifi_connect();
}

esp_err_t ControllerComm::disconnectSTA()
{
    sta_connected_ = false;
    current_sta_index_ = -1;
    is_connecting_ = false;
    return esp_wifi_disconnect();
}
```

### 1.6 デストラクタでのクリーンアップ

```cpp
ControllerComm::~ControllerComm()
{
    if (wifi_event_handler_) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler_);
    }
    if (ip_event_handler_) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               ip_event_handler_);
    }
}
```

---

## Phase 2: NVS永続化とCLIコマンド

### 2.1 NVSスキーマ

**既存NVSキー（`NVS_NAMESPACE="stampfly"`）:**
- `ctrl_mac` (blob, 6bytes) - コントローラMACアドレス
- `wifi_ch` (u8) - WiFiチャンネル

**追加NVSキー（複数AP対応）:**
```cpp
static constexpr const char* NVS_KEY_STA_COUNT = "sta_count";      // u8
static constexpr const char* NVS_KEY_STA_AUTO  = "sta_auto";       // u8

// 各AP設定（index 0-4）
// 例: "sta_0_ssid", "sta_0_pass", "sta_1_ssid", "sta_1_pass", ...
static constexpr const char* NVS_KEY_STA_SSID_FMT = "sta_%d_ssid"; // string
static constexpr const char* NVS_KEY_STA_PASS_FMT = "sta_%d_pass"; // string
```

**NVS容量見積もり:**
- 1つのAP設定: SSID(32) + Password(64) = 96 bytes
- 5つのAP設定: 96 × 5 = 480 bytes
- メタデータ(count, auto): 2 bytes
- 合計: 約500 bytes（NVS十分に余裕あり）

### 2.2 NVS保存/復元関数

**ファイル**: `firmware/vehicle/components/sf_svc_comm/controller_comm.cpp`

```cpp
esp_err_t ControllerComm::saveSTAConfigsToNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    // AP設定数を保存
    nvs_set_u8(handle, NVS_KEY_STA_COUNT, sta_config_count_);

    // 自動接続フラグを保存
    nvs_set_u8(handle, NVS_KEY_STA_AUTO, sta_auto_connect_ ? 1 : 0);

    // 各AP設定を保存
    char key[32];
    for (int i = 0; i < sta_config_count_; i++) {
        snprintf(key, sizeof(key), NVS_KEY_STA_SSID_FMT, i);
        nvs_set_str(handle, key, sta_configs_[i].ssid);

        snprintf(key, sizeof(key), NVS_KEY_STA_PASS_FMT, i);
        nvs_set_str(handle, key, sta_configs_[i].password);
    }

    // 削除されたAP設定のキーをクリア（index >= sta_config_count_）
    for (int i = sta_config_count_; i < MAX_STA_CONFIGS; i++) {
        snprintf(key, sizeof(key), NVS_KEY_STA_SSID_FMT, i);
        nvs_erase_key(handle, key);

        snprintf(key, sizeof(key), NVS_KEY_STA_PASS_FMT, i);
        nvs_erase_key(handle, key);
    }

    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Saved %d STA configs to NVS", sta_config_count_);
    return ESP_OK;
}

esp_err_t ControllerComm::loadSTAConfigsFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "No STA configs in NVS (first boot)");
        return ESP_ERR_NOT_FOUND;
    }

    // AP設定数を読み込み
    uint8_t count = 0;
    ret = nvs_get_u8(handle, NVS_KEY_STA_COUNT, &count);
    if (ret != ESP_OK || count > MAX_STA_CONFIGS) {
        nvs_close(handle);
        return ESP_ERR_INVALID_STATE;
    }

    sta_config_count_ = count;

    // 各AP設定を読み込み
    char key[32];
    for (int i = 0; i < sta_config_count_; i++) {
        size_t len;

        snprintf(key, sizeof(key), NVS_KEY_STA_SSID_FMT, i);
        len = sizeof(sta_configs_[i].ssid);
        ret = nvs_get_str(handle, key, sta_configs_[i].ssid, &len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load SSID #%d", i);
            sta_configs_[i].is_valid = false;
            continue;
        }

        snprintf(key, sizeof(key), NVS_KEY_STA_PASS_FMT, i);
        len = sizeof(sta_configs_[i].password);
        ret = nvs_get_str(handle, key, sta_configs_[i].password, &len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load password #%d", i);
            sta_configs_[i].is_valid = false;
            continue;
        }

        sta_configs_[i].is_valid = true;
        ESP_LOGI(TAG, "Loaded STA config #%d: SSID=%s", i, sta_configs_[i].ssid);
    }

    nvs_close(handle);
    return ESP_OK;
}

bool ControllerComm::loadSTAAutoConnectFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return true;  // デフォルトON（ユーザー要件）
    }

    uint8_t auto_conn = 1;  // デフォルトON
    nvs_get_u8(handle, NVS_KEY_STA_AUTO, &auto_conn);
    nvs_close(handle);

    return auto_conn == 1;
}
```

### 2.3 CLIコマンド実装

**ファイル**: `firmware/vehicle/components/sf_svc_console/commands/cmd_comm.cpp`

**`wifi` コマンド拡張（複数AP対応）:**

```cpp
static int cmd_wifi(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& comm = ControllerComm::getInstance();

    if (argc < 2) {
        // ヘルプ表示
        console.print("WiFi Configuration:\r\n");
        console.print("  Mode: AP+STA\r\n");
        console.print("  Channel: %d\r\n", comm.getChannel());
        console.print("  AP SSID: StampFly\r\n");
        console.print("  AP IP: 192.168.4.1\r\n");
        console.print("  STA Status: %s\r\n",
                     comm.isSTAConnected() ? "Connected" : "Disconnected");
        if (comm.isSTAConnected()) {
            console.print("  STA IP: %s\r\n", comm.getSTAIPAddress());
            console.print("  Connected to: %s\r\n", comm.getCurrentSTASSID());
        }
        console.print("  Saved APs: %d\r\n", comm.getSTAConfigCount());
        console.print("\r\nUsage:\r\n");
        console.print("  wifi status              - Show WiFi status\r\n");
        console.print("  wifi channel [1-13]      - Get/set WiFi channel\r\n");
        console.print("  wifi sta list            - List saved APs\r\n");
        console.print("  wifi sta add <ssid> <password> - Add AP config\r\n");
        console.print("  wifi sta remove <index>  - Remove AP config\r\n");
        console.print("  wifi sta connect [index] - Connect to AP (auto or specific)\r\n");
        console.print("  wifi sta disconnect      - Disconnect from AP\r\n");
        console.print("  wifi sta auto [on|off]   - Auto-connect on boot\r\n");
        return 0;
    }

    const char* cmd = argv[1];

    // ... existing "channel" and "status" handling ...

    if (strcmp(cmd, "sta") == 0) {
        if (argc < 3) {
            console.print("Usage: wifi sta <list|add|remove|connect|disconnect|auto>\r\n");
            return 1;
        }

        const char* subcmd = argv[2];

        if (strcmp(subcmd, "list") == 0) {
            // AP設定一覧表示
            int count = comm.getSTAConfigCount();
            if (count == 0) {
                console.print("No APs configured\r\n");
                return 0;
            }

            console.print("Saved APs (priority order):\r\n");
            for (int i = 0; i < count; i++) {
                const auto* cfg = comm.getSTAConfig(i);
                if (cfg) {
                    console.print("  [%d] %s", i, cfg->ssid);
                    if (comm.isSTAConnected() && comm.getCurrentSTAIndex() == i) {
                        console.print(" (connected)");
                    }
                    console.print("\r\n");
                }
            }

        } else if (strcmp(subcmd, "add") == 0) {
            // AP設定追加
            if (argc < 5) {
                console.print("Usage: wifi sta add <ssid> <password>\r\n");
                return 1;
            }

            const char* ssid = argv[3];
            const char* password = argv[4];

            esp_err_t ret = comm.addSTAConfig(ssid, password);
            if (ret == ESP_OK) {
                ret = comm.saveSTAConfigsToNVS();
                console.print("Added AP: SSID=%s (priority %d)\r\n",
                             ssid, comm.getSTAConfigCount());
            } else if (ret == ESP_ERR_NO_MEM) {
                console.print("AP list full (max 5)\r\n");
                return 1;
            } else if (ret == ESP_ERR_INVALID_STATE) {
                console.print("AP already exists\r\n");
                return 1;
            } else {
                console.print("Failed to add AP: %s\r\n", esp_err_to_name(ret));
                return 1;
            }

        } else if (strcmp(subcmd, "remove") == 0) {
            // AP設定削除
            if (argc < 4) {
                console.print("Usage: wifi sta remove <index>\r\n");
                return 1;
            }

            int index = atoi(argv[3]);
            esp_err_t ret = comm.removeSTAConfig(index);
            if (ret == ESP_OK) {
                ret = comm.saveSTAConfigsToNVS();
                console.print("Removed AP #%d\r\n", index);
            } else {
                console.print("Failed to remove AP: %s\r\n", esp_err_to_name(ret));
                return 1;
            }

        } else if (strcmp(subcmd, "connect") == 0) {
            // AP接続
            esp_err_t ret;
            if (argc >= 4) {
                // 指定indexに接続
                int index = atoi(argv[3]);
                ret = comm.connectSTA(index);
                console.print("Connecting to AP #%d...\r\n", index);
            } else {
                // 優先順位順に自動接続
                ret = comm.connectSTA();
                console.print("Auto-connecting to saved APs...\r\n");
            }

            if (ret != ESP_OK) {
                console.print("Failed to connect: %s\r\n", esp_err_to_name(ret));
                return 1;
            }

        } else if (strcmp(subcmd, "disconnect") == 0) {
            comm.disconnectSTA();
            console.print("Disconnected from AP\r\n");

        } else if (strcmp(subcmd, "auto") == 0) {
            if (argc < 4) {
                console.print("Auto-connect: %s\r\n",
                             comm.isSTAAutoConnect() ? "ON" : "OFF");
            } else {
                bool enable = (strcmp(argv[3], "on") == 0);
                comm.setSTAAutoConnect(enable);
                comm.saveSTAConfigsToNVS();
                console.print("Auto-connect: %s\r\n", enable ? "ON" : "OFF");
            }

        } else {
            console.print("Unknown subcommand: %s\r\n", subcmd);
            return 1;
        }
    }

    return 0;
}
```

---

## Phase 3: エラーハンドリングと統合

### 3.1 チャンネル変更の警告

**既存の `setChannel()` 関数に警告追加:**

```cpp
esp_err_t ControllerComm::setChannel(int channel, bool save_to_nvs)
{
    // ... existing validation ...

    // STA接続中の警告
    if (sta_connected_) {
        ESP_LOGW(TAG, "STA is connected - channel change may disconnect AP");
        ESP_LOGW(TAG, "STA will override this channel when reconnected");
    }

    // ... existing implementation ...
}
```

### 3.2 再接続ロジック

**イベントハンドラ内で実装済み:**
- `WIFI_EVENT_STA_DISCONNECTED` で自動再接続
- `sta_auto_connect_` フラグで制御

### 3.3 統合テスト項目

| テストケース | 期待動作 | 確認方法 |
|-------------|---------|---------|
| STA設定なしで起動 | AP専用モードで起動 | WiFi CLI接続確認 |
| STA設定あり、auto=on | 起動時に優先順位順で自動接続 | ログでIP取得確認、接続先SSID確認 |
| STA設定あり、auto=off | 起動時は未接続 | 手動接続コマンド実行 |
| 複数AP設定、最初が不在 | 2番目のAPに自動接続 | ログで試行順序確認 |
| 複数AP設定、全て不在 | 全てタイムアウト後リトライ | ログで全試行確認 |
| 存在しないSSIDに接続 | 次のAPを試行 | ログで切り替え確認 |
| パスワード誤り | AUTH_FAIL、次のAPを試行 | エラーログ確認、次AP接続 |
| ルーター再起動（接続中） | 自動再接続成功（次APも試行） | STA IP再取得確認 |
| STA接続中にチャンネル変更 | 警告ログ、STAが優先 | チャンネル追従ログ |
| AP追加/削除/一覧 | CLI経由で管理可能 | `wifi sta list` 確認 |

---

## 検証方法

### ステップ1: 複数AP設定と接続

```bash
# シリアルコンソール or Telnet (telnet 192.168.4.1)
> wifi status
# Mode: AP+STA, Channel: 1, STA: Disconnected, Saved APs: 0

> wifi sta add Lab-WiFi lab123
# Added AP: SSID=Lab-WiFi (priority 1)

> wifi sta add Home-WiFi home456
# Added AP: SSID=Home-WiFi (priority 2)

> wifi sta list
# Saved APs (priority order):
#   [0] Lab-WiFi
#   [1] Home-WiFi

> wifi sta connect
# Auto-connecting to saved APs...
# Connecting to SSID 'Lab-WiFi' (priority 1/2)...
# (数秒後)
# STA got IP: 192.168.1.123 (connected to 'Lab-WiFi')

> wifi status
# STA Status: Connected
# STA IP: 192.168.1.123
# Connected to: Lab-WiFi
# Saved APs: 2
```

### ステップ2: フェイルオーバーテスト

```bash
# Lab-WiFiルーターを停止

# 自動的に次のAPを試行
# ログで確認:
# "STA disconnected from 'Lab-WiFi' (reason:201)"
# "Trying next AP (index 1)..."
# "Connecting to SSID 'Home-WiFi' (priority 2/2)..."
# "STA got IP: 192.168.0.50 (connected to 'Home-WiFi')"

> wifi status
# STA Status: Connected
# STA IP: 192.168.0.50
# Connected to: Home-WiFi
```

### ステップ3: 自動接続テスト

```bash
> wifi sta auto on
# Auto-connect: ON

# リブート
> reboot

# 起動ログで確認:
# "Loaded STA config #0: SSID=Lab-WiFi"
# "Loaded STA config #1: SSID=Home-WiFi"
# "Auto-connecting to STA (2 APs configured)..."
# "Connecting to SSID 'Lab-WiFi' (priority 1/2)..."
# (Lab-WiFiが不在の場合)
# "STA disconnected from 'Lab-WiFi' (reason:201)"
# "Trying next AP (index 1)..."
# "Connecting to SSID 'Home-WiFi' (priority 2/2)..."
# "STA got IP: 192.168.0.50 (connected to 'Home-WiFi')"
```

### ステップ4: APとSTAの共存確認

**PC側の操作:**

```bash
# 1. StampFly APに接続（SSID="StampFly"）
# PC IP: 192.168.4.x

# WebSocket telemetry確認
firefox http://192.168.4.1

# 2. 同時に、StampFlyと同じWiFiルーターに接続
# PC IP: 192.168.1.y
# StampFly STA IP: 192.168.1.123

# ROS2ブリッジから接続確認
ping 192.168.1.123
nc 192.168.1.123 23  # Telnet CLI
```

### ステップ5: チャンネル追従確認

```bash
# ルーターのチャンネルを変更（例: ch1 -> ch6）
# ESP32がSTAとして再接続時、自動的にch6に追従
# ログで確認:
# "Channel changed: 1 -> 6 (AP follows STA)"
```

---

## 実装ファイル一覧

### 修正ファイル

| ファイル | 変更内容 | 優先度 |
|---------|---------|--------|
| `firmware/vehicle/components/sf_svc_comm/include/controller_comm.hpp` | データ構造追加、公開インターフェース追加 | 高 |
| `firmware/vehicle/components/sf_svc_comm/controller_comm.cpp` | イベントハンドラ、init()修正、STA接続/NVS関数実装 | 高 |
| `firmware/vehicle/components/sf_svc_console/commands/cmd_comm.cpp` | `wifi sta` コマンド追加 | 中 |

### 参考ファイル

| ファイル | 参照箇所 |
|---------|---------|
| `firmware/controller/components/sf_udp_client/udp_client.cpp` | STAモードのイベントハンドラパターン（L54-L106） |

---

## マイルストーン

### Milestone 1: STA基盤（Phase 1）
- [ ] `controller_comm.hpp` にデータ構造・インターフェース追加
- [ ] `controller_comm.cpp` にイベントハンドラ実装
- [ ] `init()` 関数修正（netif作成、イベント登録）
- [ ] `setSTAConfig()`, `connectSTA()`, `disconnectSTA()` 実装
- [ ] **検証**: 手動でSTA接続成功（`wifi sta config` + `wifi sta connect`）

### Milestone 2: 永続化とCLI（Phase 2）
- [ ] NVS保存/復元関数実装
- [ ] `cmd_comm.cpp` に `wifi sta` コマンド追加
- [ ] **検証**: 設定保存→リブート→手動接続

### Milestone 3: 自動接続（Phase 3）
- [ ] `init()` に自動接続ロジック追加
- [ ] `wifi sta auto on/off` コマンド実装
- [ ] チャンネル追従ログ追加
- [ ] **検証**: 設定保存→リブート→自動接続成功

### Milestone 4: 極端な条件（Phase 3）
- [ ] エラーハンドリング（接続失敗、タイムアウト）
- [ ] 全シナリオでの統合テスト
- [ ] **検証**: 存在しないSSID、パスワード誤り、ルーター再起動

---

## 設計判断の根拠

### 1. なぜAP設定を最大5個にするのか？
**理由**: ユーザー要件「研究室と自宅の少なくとも2つ」を満たすため。5個あれば研究室、自宅、カフェ、会社など複数環境でも十分。それ以上はNVS容量とUI複雑性のトレードオフで不要と判断。

### 2. なぜ優先順位は保存順（固定）にするのか？
**理由**: シンプルで予測可能。ユーザーは最もよく使うAPを最初に追加すればよい。動的な優先順位変更（RSSI順など）は予期しない動作を招く可能性がある。

### 3. なぜ自動接続をデフォルトONにするのか？
**理由**: ユーザーが「必須機能」と回答。ROS2開発では電源ONするだけでルーター接続してほしいため。明示的にOFFにしない限り自動接続する。

### 4. なぜパスワードを平文保存するのか？
**理由**: 教育用プラットフォームとして実装の単純さを優先。本格運用ではNVS Flash Encryptionを有効化することを推奨（ドキュメントに記載）。

### 5. なぜ接続失敗時に次のAPを試すのか？
**理由**: ユーザーが複数環境を移動する際の利便性。研究室で起動したら研究室のAPに接続、自宅で起動したら自宅のAPに接続、全て自動で行われる。

### 6. なぜチャンネル変更に警告を出すのか？
**理由**: ESP32の制約（STA優先）を明示し、ユーザーの混乱を防ぐため。ESP-NOWもチャンネルに依存するため、予期しない動作を防ぐ。

---

## リスクと制約事項

| リスク | 影響 | 軽減策 |
|--------|------|--------|
| STA接続失敗時のAP動作停止 | 高 | イベントハンドラでAP継続を保証、独立動作確認 |
| チャンネル変更によるESP-NOW切断 | 中 | ログで警告、ドキュメント化、再ペアリング手順提供 |
| パスワード漏洩 | 中（教育用途） | NVS暗号化推奨をドキュメント化 |
| NVS容量不足 | 低 | 5個のAP設定で約500 bytes、十分な余裕あり |
| 複数AP試行でブート時間増加 | 低 | タイムアウトを適切に設定、全試行で最大30秒程度 |
| フェイルオーバー時の一時的切断 | 低 | アプリケーション層で再接続処理を実装推奨 |

---

## 使用シナリオ別の動作

### シナリオA: ラボと自宅を行き来する開発者
1. **初回設定**: `wifi sta add Lab-WiFi labpass` + `wifi sta add Home-WiFi homepass`
2. **ラボで起動**: 自動的にLab-WiFi（priority 1）に接続
3. **自宅で起動**: Lab-WiFiが不在のため、自動的にHome-WiFi（priority 2）に接続
4. **両環境でROS2開発**: STA IP経由で同じコードベースで動作
5. **AP維持**: どちらの環境でもローカルデバッグ用にAPモード併用可能

### シナリオB: フィールドでの飛行
1. 自動接続OFF設定でリブート（`wifi sta auto off`）
2. AP専用モード（STA接続試行なし、起動高速化）
3. ESP-NOWコントローラとペアリング
4. 単機飛行、ログ記録

### シナリオC: インターネットアクセス
1. STA経由でルーター接続（複数APから自動選択）
2. ルーターがインターネットに接続
3. StampFlyからOTA更新、クラウドロギング、NTP時刻同期が可能
4. AP側は同時にローカルデバッグ用として使用

### シナリオD: 複数ドローン群制御（将来）
1. 全てのドローンが同じルーターに接続（STA mode）
2. 各ドローンは同じAP設定リストを持つ（Lab-WiFi, Home-WiFi, ...）
3. PCから各ドローンのSTA IPへ一括制御指令
4. APモードは各機でローカルデバッグ用として維持

### シナリオE: ルーター障害時の自動復旧
1. Lab-WiFiに接続中
2. Lab-WiFiルーターが再起動
3. 自動的にLab-WiFiへ再接続試行
4. 失敗したら次のAP（Home-WiFi）を試行
5. 全てのAPが復旧するまでリトライ継続

---

## 次のステップ（実装後）

1. **ドキュメント化**
   - CLIコマンドリファレンス更新
   - チャンネル管理の注意事項を `docs/plans/WIFI_COMM_PLAN.md` に追記
   - 使用例を `README.md` に追加

2. **ROS2統合テスト**
   - STA IP経由でROS2ブリッジノードと通信
   - 400Hzテレメトリ受信確認
   - UDP制御パケット送信確認

3. **群制御への準備**
   - 複数ドローンを同じルーターに接続
   - IP割り当て管理（DHCPまたは静的IP）
   - 一括制御プロトコルの検討

---

## 参考資料

- ESP-IDF WiFi Driver: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html
- APSTA Mode: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#station-ap-coexistence
- Controller側STA実装: `firmware/controller/components/sf_udp_client/udp_client.cpp`

---

**Critical Files:**
- `/Users/kouhei/Library/CloudStorage/Dropbox/01教育研究/20マルチコプタ/stampfly_ecosystem/firmware/vehicle/components/sf_svc_comm/controller_comm.cpp`
- `/Users/kouhei/Library/CloudStorage/Dropbox/01教育研究/20マルチコプタ/stampfly_ecosystem/firmware/vehicle/components/sf_svc_comm/include/controller_comm.hpp`
- `/Users/kouhei/Library/CloudStorage/Dropbox/01教育研究/20マルチコプタ/stampfly_ecosystem/firmware/vehicle/components/sf_svc_console/commands/cmd_comm.cpp`
