# Sistemas Embebidos Avanzados

La mejor materia de Ingenieria Electrónica tiene su repositorio. Aquí se encuentra de manera organizada los drivers de los sensores y algunos programas desarrollados a lo largo del curso

## Drivers
- [ ] T-AS5600 Magnetic encoder

- [ ] VL6180 Laser Range Sensor

- [ ] TM151 IMU

- [ ] APDS-9960 RGB Sensor

### Estructura de los drivers

my_project/

├── CMakeLists.txt                # CMake principal del proyecto

├── sdkconfig                     # Configuración generada por `menuconfig`

├── main/

│   ├── CMakeLists.txt            # CMake del proyecto principal

│   ├── main.c                    # Archivo principal de la aplicación

│   └── README.md                 # Documentación del proyecto

└── components/                   # Carpeta de componentes personalizados

    └── sensor_driver/            # Driver del sensor como componente
    
        ├── CMakeLists.txt        # CMake del componente del driver
        
        ├── sensor_driver.c       # Implementación del driver
        
        ├── sensor_driver.h       # Archivo de cabecera del driver
        
        └── driver_config.h       # Configuración opcional del driver

### Configuración del main/CMakeLists.txt

```c 
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ".")

# Incluir el componente del driver
target_link_libraries(${COMPONENT_LIB} PRIVATE sensor_driver) 
``` 
