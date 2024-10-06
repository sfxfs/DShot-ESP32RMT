idf_component_register(SRCS "DShotRMT.cpp" "dshot_rmt_encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer)