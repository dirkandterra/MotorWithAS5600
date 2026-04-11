idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES as5600 ssd1306 l293d driver
)
