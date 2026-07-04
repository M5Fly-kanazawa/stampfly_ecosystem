idf_component_register(
    SRCS "serial_cli.cpp"
    INCLUDE_DIRS "include"
    REQUIRES console freertos sf_lib_line_editor sf_svc_console
)
