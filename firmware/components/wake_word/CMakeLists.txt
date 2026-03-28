idf_component_register(
    SRCS "wake_word.c"
    INCLUDE_DIRS "include"
    REQUIRES va_common audio_pipeline_mgr audio_recorder esp-sr log
)
