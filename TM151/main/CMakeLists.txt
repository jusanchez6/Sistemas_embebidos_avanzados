file(GLOB SRC_FILES "../EasyProfile/*.cpp")

idf_component_register(
    SRCS "TM151.cpp" ${SRC_FILES}
    INCLUDE_DIRS "." "../EasyProfile/include"
)

