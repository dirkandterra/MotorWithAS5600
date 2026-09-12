idf_component_register(
    SRCS "main.c"
    "can.c"
    INCLUDE_DIRS "."
    REQUIRES as5600 l293d driver
)
