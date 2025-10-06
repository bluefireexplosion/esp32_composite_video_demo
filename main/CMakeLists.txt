idf_component_register(
    SRCS "demo.c" "rhino.c" "pm5544.c"
    INCLUDE_DIRS ""
)
target_add_binary_data(${COMPONENT_TARGET} "./assets/portal.mp3" BINARY)