# sf_estimator: estimator interface (header) + pure-logic estimator-side detection
# (tilt_consistency, host-testable, no ESP-IDF dependency)
# sf_estimator: 推定器インターフェース（ヘッダ）＋推定器側の純粋ロジック検出
# （tilt_consistency、ホストテスト可能、ESP-IDF 非依存）
idf_component_register(
    SRCS "tilt_consistency.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        sf_core
)
