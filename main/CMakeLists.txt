idf_component_register(SRCS "stanford_kinematics.c" "stanford_gait.c" "main.c" "driver_board.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "wizard.html"
                       PRIV_REQUIRES mqtt esp_wifi esp_event esp_netif nvs_flash esp_http_server esp_driver_gpio esp_driver_spi esp_timer lwip)