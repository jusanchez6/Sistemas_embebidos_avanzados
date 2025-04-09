| Supported Targets | ESP32-S3 |

# AS5600 ESP32-S3 driver

This library is designed to work with the AS5600 magnetic position sensor using an ESP32-S3. It includes features for initialization, configuration, angle reading, and an automated calibration process.

## Features

- I2C communication with the AS5600 sensor
- Support for angle reading via ADC
- Full configuration of the CONF register
- Automatic calibration using a timer


## Overview
The AS5600 is a 12-bit contactless rotary position sensor that uses magnetic field sensing to determine the angle of a rotating magnet. This library provides an interface for communicating with the AS5600 sensor using I2C and reading angle values either through digital communication or via the ADC output.

### Features
- I2C communication support
- ADC-based angle reading
- Register access for sensor configuration
- Functions for setting and retrieving configuration parameters
- Support for permanent programming (burn mode)

## How to Use

### 1. **Include the Library**
Ensure you include the necessary header files in your project:

```c
#include "as5600_lib.h"
```

### 2. **Initialize the Sensor**
Before using the sensor, initialize the I2C communication and ADC pin (if using analog output):

```c
AS5600_t sensor;
AS5600_Init(&sensor, I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_34);
```

- `I2C_NUM_0`: I2C port number
- `GPIO_NUM_21`: SCL pin
- `GPIO_NUM_22`: SDA pin
- `GPIO_NUM_34`: ADC pin (OUT pin of AS5600)

### 3. **Read the Angle Value**
You can read the angle from the AS5600 sensor using I2C:

```c
uint16_t angle;
AS5600_GetAngle(&sensor, &angle);
printf("Angle: %d degrees\n", angle);
```

The value can also be obtained by adc.

```c
float angle = AS5600_ADC_GetAngle(&sensor);
printf("Analog Angle (ADC): %.2f degrees\n", angle);
```


## 4. AS5600 Sensor Calibration

This code includes a basic calibration routine for the AS5600 sensor using the ESP32-S3. The calibration runs automatically at program startup. Here's the step-by-step process:

1. **Program start**  
   When the ESP32-S3 is powered on, the sensor is configured and a timer starts the calibration process.

2. **Position the magnet at the maximum angle**  
   In the first step, you'll be prompted to move the magnet (or rotating shaft) to its **maximum** rotation position.  
   The console will show a message like:
   ```
   AS5600 calibration step 1. As step 4 in page 23 of the datasheet, move the magnet (or wheel) to the MAX position.
   ```

3. **Set OUT signal to ground (GND)**  
   The code sets the sensor's OUT pin to low to mark this position as part of the calibration.

4. **Angle reading**  
   The ADC angle reading is enabled.  
   The ESP32 will print the angle value once per second for 100 cycles.  
   This helps verify if the signal is stable and the maximum position was correctly registered:
   ```
   angle-> 265.32
   angle-> 265.37
   ...
   ```

5. **Calibration complete**  
   The timer stops and the calibration routine finishes.  
   The sensor is now ready to operate with the calibrated range.

## Documentation
here it includes too the documentation with doxygen.Run the following command for futher information

```bash
doxygen Doxyfile
```

