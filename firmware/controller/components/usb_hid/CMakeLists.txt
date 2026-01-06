idf_component_register(
    SRCS "src/usb_hid.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        esp_common
        espressif__esp_tinyusb
)
