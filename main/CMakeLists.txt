idf_component_register(SRCS "main.c" "minmea/minmea.c" "gps.c"
                    INCLUDE_DIRS ".")

# Define timegm as mktime for compatibility
add_compile_definitions(-Dtimegm=mktime)