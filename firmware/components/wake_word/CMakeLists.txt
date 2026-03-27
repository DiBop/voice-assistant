idf_component_register(
    SRCS "wake_word.c"
    INCLUDE_DIRS "include"
    # audio_recorder and esp-sr are ADF components added when implementing
    # REQUIRES audio_recorder esp-sr log
)
