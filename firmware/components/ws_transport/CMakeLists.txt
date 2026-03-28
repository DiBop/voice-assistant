idf_component_register(
    SRCS "ws_transport.c"
    INCLUDE_DIRS "include"
    REQUIRES va_common espressif__esp_websocket_client esp_wifi nvs_flash
             esp_event esp_netif log freertos
)
