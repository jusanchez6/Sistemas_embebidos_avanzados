file(GLOB SRCS_AS5600  "../sensors/AS5600_HAL_Library/*.c")
file(GLOB SRCS_TM151  "../sensors/TM151-driver/*.c")
file(GLOB SRCS_VL53L1X  "../sensors/VL53L1X-driver/*.c")

file(GLOB SRCS_PLATFORM  "../platform/*.c")
file(GLOB SRCS_DRIVERS  "../drivers/*.c")




idf_component_register(
        SRCS "sensor_fusion.c" "main.c" ${SRCS_AS5600} ${SRCS_TM151} ${SRCS_VL53L1X} ${SRCS_PLATFORM} ${SRCS_DRIVERS}

        INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include" "../sensors/TM151-driver/include" "../sensors/VL53L1X-driver/include"
                    "../platform/include" "../drivers/include"
)

