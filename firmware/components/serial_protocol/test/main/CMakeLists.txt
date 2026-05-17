idf_component_register(
    SRCS "test_app_main.c" "../test_serial_protocol.c"
    INCLUDE_DIRS "."
    REQUIRES unity serial_protocol
    WHOLE_ARCHIVE
)
