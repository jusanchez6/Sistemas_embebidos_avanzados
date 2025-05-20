file(GLOB SRCS_AS5600  "../sensors/AS5600_HAL_Library/*.c")
file(GLOB SRCS_TM151  "../sensors/TM151-driver/*.c")
file(GLOB SRCS_VL53L1X  "../sensors/VL53L1X-driver/*.c")

file(GLOB SRCS_PLATFORM  "../platform/*.c")
file(GLOB SRCS_BLDC_PWM  "../drivers/bldc_pwm/*.c")
file(GLOB SRCS_PID_LIB  "../drivers/pid_lib/*.c")
file(GLOB SRCS_SENSOR_FUSION "../drivers/sensor_fusion/*.c")



idf_component_register(
        SRCS "main.c" "main_pid.c" ${SRCS_AS5600} ${SRCS_TM151} ${SRCS_VL53L1X} ${SRCS_PLATFORM} ${SRCS_BLDC_PWM} ${SRCS_PID_LIB} ${SRCS_SENSOR_FUSION}

        INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include" "../sensors/TM151-driver/include" "../sensors/VL53L1X-driver/include"
                    "../platform/include" "../drivers/bldc_pwm/include" "../drivers/pid_lib/include" "../drivers/sensor_fusion/include"
)

