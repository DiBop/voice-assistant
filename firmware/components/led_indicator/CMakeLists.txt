idf_component_register(
    SRCS "led_indicator.c"
    INCLUDE_DIRS "include"
    # driver added when implementing WS2812 RMT
    # REQUIRES driver log
)
