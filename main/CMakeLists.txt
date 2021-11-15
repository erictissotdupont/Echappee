idf_component_register(SRCS "Echappe_main.c" "WebServer.c" "PowerManager.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_server nvs_flash driver )