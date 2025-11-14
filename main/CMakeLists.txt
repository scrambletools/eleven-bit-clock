idf_component_register(SRCS "clock.c"
                    PRIV_REQUIRES esp_wifi nvs_flash led_strip esp_adc esp_http_server dns_server esp_driver_i2s esp_driver_gpio lwip
                    INCLUDE_DIRS "."
                    EMBED_FILES root.html setup_root.html style.css response.html timezones.csv)
