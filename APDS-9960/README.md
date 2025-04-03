# AS5600 Library Guide

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

Or if using the ADC output:

```c
float angle_adc = AS5600_ADC_GetAngle(&sensor);
printf("Angle from ADC: %.2f degrees\n", angle_adc);
```

### 4. **Configure Sensor Settings**
Set a custom start position:

```c
AS5600_SetStartPosition(&sensor, 1024);
```

Read the start position:

```c
uint16_t start_position;
AS5600_GetStartPosition(&sensor, &start_position);
printf("Start Position: %d\n", start_position);
```

### 5. **Burn Configuration (Permanent Programming)**
If you need to permanently store the zero and max positions:

```c
AS5600_BurnAngleCommand(&sensor);
```

### 6. **Deinitialize the Sensor**
Before shutting down or resetting, release the I2C and ADC resources:

```c
AS5600_Deinit(&sensor);
```

## Notes
- The sensor requires a stable power supply (3.3V) to ensure accurate readings.
- The ADC output range is limited to 10%-90% of VCC, so consider calibration if using the ADC method.
- Burn commands (`BurnAngleCommand` and `BurnSettingCommand`) are **irreversible** and should be used with caution.

---
This guide provides a basic overview of how to use the AS5600 library. For advanced configurations, refer to the AS5600 datasheet and source code documentation.

