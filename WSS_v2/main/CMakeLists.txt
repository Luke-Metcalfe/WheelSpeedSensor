#idf_component_register(SRCS "ir_nec_transceiver_main.c" "ir_nec_encoder.c"
#                       INCLUDE_DIRS ".")

# Application requires ACAN2517FD
set(COMPONENT_REQUIRES "ACAN25177FD")
idf_component_register(SRCS "main.cpp" "ir_nec_encoder.c"
                    INCLUDE_DIRS ".")