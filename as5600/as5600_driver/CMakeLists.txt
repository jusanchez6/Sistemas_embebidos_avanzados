idf_component_register(SRCS "as5600_driver.c"
                    INCLUDE_DIRS "include")


set_source_files_properties(AS5600.cpp PROPERTIES COMPILE_FLAGS "-std=c++17")