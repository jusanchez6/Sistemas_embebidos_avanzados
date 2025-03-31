file (GLOB SRC_FILES "../as5600/*.c")

idf_component_register(
        SRCS "main.c" ${SRC_FILES}
        INCLUDE_DIRS "." "../as5600/include" 
)
