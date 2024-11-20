set(COMPONENT_REQUIRES "driver")
idf_component_register(SRCS "data_encoder.c" "pulse_encoder.c" "wss_main.c"
                       INCLUDE_DIRS ".")
