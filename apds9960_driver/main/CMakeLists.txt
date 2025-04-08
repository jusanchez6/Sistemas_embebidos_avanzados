
file(GLOB SRC_FILES "../apds9960_driver/*.c")

idf_component_register(
	SRCS "sensorRGB.c" ${SRC_FILES} 
        INCLUDE_DIRS "." "../apds9960_driver/include"
)
