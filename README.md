# Sistemas Embebidos Avanzados

La mejor materia de Ingenieria Electrónica tiene su repositorio. Aquí se encuentra de manera organizada los drivers de los sensores y algunos programas desarrollados a lo largo del curso

## Drivers
- [x] T-AS5600 Magnetic encoder

- [ ] VL6180 Laser Range Sensor

- [ ] TM151 IMU

- [x] APDS-9960 RGB Sensor

### Estructura de los drivers
```
my_project/

├── CMakeLists.txt                      # CMake principal del proyecto

├── sdkconfig                           # Configuración generada por `menuconfig`

├── main/

│   ├── CMakeLists.txt                  # CMake del proyecto principal

│   ├── main.c                          # Archivo principal de la aplicación

└── sensor_driver/                      # Carpeta de componentes personalizados

    ├── sensor_driver.c                 # Implementación del driver

    └── include/                        # Carpeta de archivos de cabecera
        
        ├── sensor_driver.h             # Archivo de cabecera del driver
        
        └── driver_config.h             # Configuración opcional del driver
    

```

### Configuración del main/CMakeLists.txt

```c
file (GLOB SRC_FILES "../sensor_driver/*.c")

idf_component_register(
        SRCS "main.c" ${SRC_FILES}
        INCLUDE_DIRS "." "../sensor_driver/include"
)

``` 
