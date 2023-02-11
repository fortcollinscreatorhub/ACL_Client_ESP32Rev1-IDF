idf_component_register(SRCS "wificonfig.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_eth nvs_flash esp_http_server )
