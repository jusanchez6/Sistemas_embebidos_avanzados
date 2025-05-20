file (GLOB SRCS_FILES "../AS5600-driver/*.c")
file (GLOB SRCS_PLATFORM "../platform/*.c")

idf_component_register(SRCS "./main.c"
                    ${SRCS_FILES} ${SRCS_PLATFORM}
                    INCLUDE_DIRS "." "../AS5600-driver/include" "../platform/include"
)
