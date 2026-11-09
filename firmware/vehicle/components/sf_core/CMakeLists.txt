idf_component_register(
    SRCS
        "params.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        nvs_flash
        esp_timer    # reload-callback timestamps / 再読込コールバックの時刻印
        sf_math      # sf::math::kGravity for flight_limits.hpp (header-only) / flight_limits.hpp の重力定数（ヘッダのみ）
)
