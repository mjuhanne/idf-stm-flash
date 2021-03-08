idf_component_register(SRCS "main/stm_flash.c" "main/stm_pro_mode.c" "main/stm_flash_http.c"
                       INCLUDE_DIRS "main"
                       REQUIRES esp_event esp_wifi esp_http_client esp-tls
                       			freertos spiffs nvs_flash)
