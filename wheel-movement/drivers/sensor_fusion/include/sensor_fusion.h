/**
 * @file sensor_fusion.h
 * @author Striker 1
 * @brief Sensor fusion for motor control
 * @details This file contains the definition of sensor fusion algorithms for motor control applications. The
 * sensor fusion algorithms are used to combine data from multiple sensors to improve the accuracy and reliability
 * of the system.
 * @version 0.1
 * @date 06/05/2025
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct
{
    float velocity; // Velocity in cm/s
    float prev_acc; // Previous acceleration values
} imu_data_t;


/**
 * @brief Initialize the sensor fusion module
 * @param imu_data Pointer to the IMU data structure
 */
void sf_init(imu_data_t *imu_data);

/**
 * @brief Estimate the velocity using IMU data
 * @param imu_data Pointer to the IMU data structure
 * @param acceleration Current acceleration value (m/s^2)
 * @param time_interval Time interval between measurements (s)
 */
void estimate_velocity_imu(imu_data_t *imu_data, float acceleration, float time_interval);

/**
 * @brief Estimate the velocity using Lidar and Encoder data
 * @param velocity_imu Velocity from IMU (cm/s)
 * @param velocity_lidar Velocity from Lidar (cm/s)
 * @param velocity_encoder Velocity from Encoder (cm/s)
 * @return Estimated velocity (cm/s)
 */
float estimate_velocity(float velocity_imu, float velocity_lidar, float velocity_encoder);

#endif // SENSOR_FUSION_H