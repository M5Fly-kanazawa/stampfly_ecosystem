# Collect all VL53LX driver source files
file(GLOB VL53LX_SRCS "src/vl53lx/*.c")

idf_component_register(
    SRCS "src/vl53lx_platform.c" "src/vl53lx_platform_ipp.c" "src/vl53lx_outlier_filter.c" "src/vl53l3cx_wrapper.cpp" ${VL53LX_SRCS}
    INCLUDE_DIRS "include/vl53lx" "include"
    REQUIRES driver esp_timer
)
