idf_component_register(SRCS "Wifi_example.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver esp_wifi nvs_flash)
