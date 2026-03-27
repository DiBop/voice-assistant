idf_component_register(
    SRCS "audio_pipeline_mgr.c"
    INCLUDE_DIRS "include"
    # audio_pipeline and audio_stream are ADF components added when implementing
    # REQUIRES audio_pipeline audio_stream log driver
)
