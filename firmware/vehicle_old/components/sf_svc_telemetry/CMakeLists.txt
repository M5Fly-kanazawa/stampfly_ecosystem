idf_component_register(
    SRCS "telemetry.cpp" "udp_telemetry.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_http_server esp_event freertos lwip sf_svc_state
    PRIV_REQUIRES esp_timer
    EMBED_FILES "www/index.html" "www/three.min.js"
)
