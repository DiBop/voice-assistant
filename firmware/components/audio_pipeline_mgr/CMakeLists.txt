idf_component_register(
    SRCS "audio_pipeline_mgr.c"
    INCLUDE_DIRS "include"
    REQUIRES va_common audio_pipeline audio_stream esp-adf-libs log
)
