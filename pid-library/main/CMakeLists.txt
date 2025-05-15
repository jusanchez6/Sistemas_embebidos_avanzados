file(GLOB SRC_FILES "../pid_lib/*.c")

idf_component_register(
	SRCS "PID-LIBRARY.c" ${SRC_FILES}
                    INCLUDE_DIRS "." "../pid_lib/include"

)
