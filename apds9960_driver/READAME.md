# APDS9960 (LIGHT AND COLOR SENSOR) DRIVER.
This folder contains the driver of the APDS9960 sensor, here you can find:
- HAL code for the Esp32s3 I2C protocol comunication.
- APDS9960 definitions for registers, values, bitfields and configuration structures.
- Functions to configure the sensor, the gain, and read the values of red, green and blue components in the light.
- A simple example of the usage of the library written in C and available for the Esp32s3
to use the driver and run the example, run the commando:
```bash
idf.py -p [PORT] flash monitor
```
It should shows a simple message on terminal or serial monitor with the three values of red, green and blue detected
printing in a value of 0 to 255

Also includes the doxygen documentation, running the command:

```bash
doxygen Doxyfile
```


 

