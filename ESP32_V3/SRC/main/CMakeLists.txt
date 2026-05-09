file(GLOB SH2_SOURCES "sh2/*.c")

idf_component_register(
    SRCS "main.c" "bno085_hal.c" ${SH2_SOURCES}
    INCLUDE_DIRS "." "sh2"
    REQUIRES driver esp_timer freertos
)