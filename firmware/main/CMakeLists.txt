idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES led_indicator audio_pipeline_mgr wake_word ws_transport
             nvs_flash esp_wifi esp_event esp_netif
)
