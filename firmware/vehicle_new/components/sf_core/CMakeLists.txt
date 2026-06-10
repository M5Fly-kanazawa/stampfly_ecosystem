idf_component_register(
    SRCS
        "params.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        nvs_flash
        esp_timer    # reload-callback timestamps / 再読込コールバックの時刻印
)
