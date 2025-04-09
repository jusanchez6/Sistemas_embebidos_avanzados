# Advanced Embedded Systems

The best course in Electronic Engineering has its own repository. Here you'll find a well-organized collection of sensor drivers and some programs developed throughout the course.

## Drivers
- [x] T-AS5600 Magnetic Encoder  
- [x] VL6180 Laser Range Sensor  
- [x] TM151 IMU  
- [x] APDS-9960 RGB Sensor  

## Driver Structure

```
my_project/

├── CMakeLists.txt                      # main CMake file of the project 

├── sdkconfig                           # Configuration generated by `menuconfig`

├── main/

│   ├── CMakeLists.txt                  # project Cmake file.

│   ├── main.c                          # main file of the project

└── sensor_driver/                      # driver folder

    ├── sensor_driver.c                 # Iimplementation of the folder

    └── include/                        # header files folder
        
        ├── sensor_driver.h             # header file
        
        └── driver_config.h             # optional configuration of the driver
    

```

## Basic configuration of the `main/CMakeLists.txt`

The following code shows how to corretly configure the CMakeLists file for the proper use of the structure of the drivers presented previously


```c
file (GLOB SRC_FILES "../sensor_driver/*.c")

idf_component_register(
        SRCS "main.c" ${SRC_FILES}
        INCLUDE_DIRS "." "../sensor_driver/include"
)

``` 
