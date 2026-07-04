# sf_controller is header-only (interface definition)
# sf_controllerはヘッダーのみ（インターフェース定義）
idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES
        sf_core
        sf_state
)
