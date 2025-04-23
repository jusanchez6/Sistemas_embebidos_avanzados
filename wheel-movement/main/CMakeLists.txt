idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include"
                    "../sensors/TM151-driver/include" "../sensors/VL53L1X-driver/include"
                    "../platform/include")

file(GLOB SRC_FILES "../sensors/*/*.c" "../platform/*/*.c")