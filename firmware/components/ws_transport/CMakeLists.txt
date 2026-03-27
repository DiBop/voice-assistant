idf_component_register(
    SRCS "ws_transport.c"
    INCLUDE_DIRS "include"
    # esp_websocket_client is added when implementing
    # REQUIRES esp_websocket_client log
)
