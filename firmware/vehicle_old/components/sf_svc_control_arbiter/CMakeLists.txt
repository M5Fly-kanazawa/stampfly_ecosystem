# StampFly Control Arbiter Component
# 制御入力アービターコンポーネント

idf_component_register(
    SRCS
        "control_arbiter.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        freertos
        esp_common
        log
    PRIV_REQUIRES
        sf_svc_state
)
