# Advanced Embedded Systems

The best course in Electronic Engineering has its own repository. Here you'll find a well-organized collection of sensor drivers and some programs developed throughout the course.

# LAB 1 - [C libraries](./C%20libraries/)

The purpose of this laboratory was to develop the drives for the sensors that will be used in the robot. These are the AS5600 magnetic encoder, the VL53L1X long range LiDAR, VL6180X short range LiDAR, APD9960 RGB sensor and the TM151 9-axis IMU.

## Features

* Hardware Abstaction Layer: These libraries integrate a hardware abstaction layer (HAL) that can be configured to whatever microcontroller used. The drivers use these HAL functions and, if using other microcontroller different to ESP32-S3, feel free to change these hardware imlpementations to get the drivers work on your device.


# LAB2 - [One-Wheel Movement](./wheel-movement/)

This laboratory intended to develop a firmware capable of moving a wheel in a certain velocity and distance. This project implements the AS5600 magnetic encoder, the VL53L1X long range LiDAR and the TM151 9-axis IMU. Nevertheless, at the time, only the encoder is being used for something useful.

## Features

* Velocity controller through incremental PID.
* Distance controller through encoder arc estimation.
* Commands through UART interface: One can send commands to the ESP32-S3 to indicate certain things:
  * ```P %f %f %f %f```: To change PID parameters online (```P <proportional> <integral> <derivative> <set_point>```)
  * ```X %hd```: Sets a duty cycle (speed PWM) to the ESC (```X <new_pwm>```). *Note this is PWM, not velocity!*
  * ```S %f```: Changes the set point of the PID (```S <new_set_point>```).
  * ```D%hu_%f```: This is one of the command for this specific lab. Sets right displacement with a goal distance at a certaing goal velocity (```D<distance>_<velocity>```).
  * ```I%hu_%f```: This is one of the command for this specific lab. Sets left displacement with a goal distance at a certaing goal velocity (```I<distance>_<velocity>```).

# LAB 3 - [Robot Movement](./robot-mov/) (**in progress**)

This practice consisted in integrating more actuators and sensors in order to move a 3-wheeled robot.

## Features

* Independent velocity controller through incremental PID.
* System model to control general robot position and velocity.
* Commands through WiFi interface to move the robot:
  * Linear displacement (forward or backwards, at a specified angle, velocity and distance).
  * In-place rotation (clockwise, counter clockwise, at a specified rotation degrees and velocity).
  * Circular displacement: (clockwise, counter clockwise, at a specified radius, degrees and velocity).