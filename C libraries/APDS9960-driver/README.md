| Supported Targets | ESP32-S3 |

# APDS9960 (LIGHT AND COLOR SENSOR) DRIVER.

This library is designed to work with the RGB sensor APDS9960 using the ESP32-S3 It includes features for the initialization, configuration and RGB components reading 

## Features
* I2C communications and HAL functions
* Fully document hardware library and Macro definitions for registers and bitfields.
* Support for RGB light components sensor reading

 ## 1. How to use.

 ### 1. **Include the library**
 Ensure you include the necessary header files in your project, and configure the `CMakeLists.txt` file properly for the compiling process.

 ### 2. **Inicialize the sensor**
Before use the sensor be sure you initialize the sensor library by the following lines of code:
```c
APDS9960_t apds9960 = {};
APDS9960_init(&apds9960, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);    
```
- `I2C_MASTER_NUM`: I2C port number
- `I2C_MASTER_SCL_IO`: SCL pin
- `I2C_MASTER_SDA_IO`: SDA pin
 
 ### 3. **Configure the gain and mode of the sensor**
 You can configure the gain of the sensor and the mode whit the following lines:
 ```c
APDS9960_set_mode(&apds9960, APDS9960_AEN_ENABLE);
APDS9960_set_gain(&apds9960, APDS9960_AGAIN_64X);
```

### 4. **Read the rgb components**
Finally read the RGB components
```c
uint16_t r, g, b;
APDS9960_get_RGB(&apds9960, &r, &g, &b, true);
printf("R: %d, G: %d, B: %d\n", r, g, b);
```

## Documentation
here it includes too the documentation with doxygen.Run the following command for futher information

```bash
doxygen Doxyfile
```

