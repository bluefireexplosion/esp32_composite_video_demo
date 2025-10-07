idf_component_register(
    SRCS
        "demo.c"
        "rhino.c"
        "pm5544.c"
    INCLUDE_DIRS
        "."
)

# Embed the MP3 as raw binary data
target_add_binary_data(${COMPONENT_LIB} "portal.mp3" BINARY)