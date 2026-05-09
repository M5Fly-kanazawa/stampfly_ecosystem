idf_component_register(
    SRCS "telemetry.cpp"
    INCLUDE_DIRS "include"
    # sf_core: topics + data types / トピックとデータ型
    # sf_math: quaternion-to-Euler / クォータニオン→オイラー角変換
    # sf_comm: sf::waitForWifiReady — EventGroup-based readiness signal (R16)
    # lwip:    UDP socket API      / UDPソケットAPI
    # esp_wifi/esp_netif: read STA readiness, do NOT init  / STA準備状態の参照（初期化はsf_comm）
    # esp_timer: microsecond timestamps / マイクロ秒タイムスタンプ
    REQUIRES sf_core sf_math sf_comm lwip esp_wifi esp_netif esp_timer
)
