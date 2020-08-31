# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES "esp32-camera" "esp_wifi" "driver" "nvs_flash" "wpa_supplicant" "fatfs" "sdmmc")

set(COMPONENT_SRCS "main.c" "led.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
