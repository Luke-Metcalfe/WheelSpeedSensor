# CMakeFiles.txt inside "SparkFun_u-blox_GNSS" folder
cmake_minimum_required(VERSION 3.5)
idf_component_register(SRCS "src/ACAN2517FD.cpp" "src/ACAN2517FDSettings.cpp"
                    INCLUDE_DIRS "src/."
                    REQUIRES "arduino" # Library requires Arduino
                    )

project(WSS)