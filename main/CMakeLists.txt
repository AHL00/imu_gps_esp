idf_component_register(SRCS "mag_enc.c" "i2c_bus.c" "main.c" "minmea/minmea.c" "gps.c" "imu.c" "bno055/bno055.c"
                    INCLUDE_DIRS ".")

# Define timegm as mktime for compatibility
add_compile_definitions(-Dtimegm=mktime)