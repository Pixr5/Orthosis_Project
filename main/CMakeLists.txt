idf_component_register(SRCS "The_Holy_Code.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_ledc driver esp_adc esp_timer bt nvs_flash)
