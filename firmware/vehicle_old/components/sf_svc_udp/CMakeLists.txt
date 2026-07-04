# StampFly UDP Server Component
# UDP通信サーバーコンポーネント

idf_component_register(
    SRCS
        "udp_server.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        protocol
        lwip
        freertos
        esp_common
        log
)
