# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES "camera" "esp_wifi" "driver" "nvs_flash" "wpa_supplicant")

set(COMPONENT_SRCS "main.c" "led.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
