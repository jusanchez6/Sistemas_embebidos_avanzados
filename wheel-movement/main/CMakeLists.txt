idf_component_register(SRCS "s-curve_profile.c" "../sensors/AS5600_HAL_Library/as5600_lib.c"
                            "../platform/platform_esp32s3.c" "../drivers/bldc_pwm.c"
                    INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include"
                    "../sensors/TM151-driver/include" "../sensors/VL53L1X-driver/include"
                    "../platform/include" "../drivers/include")