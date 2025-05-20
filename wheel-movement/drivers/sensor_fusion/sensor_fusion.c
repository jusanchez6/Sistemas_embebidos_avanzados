/**
 * @file sensor_fusion.c
 * @author Striker 1
 * @brief Sensor fusion for motor control
 * @details This file contains the implementation of sensor fusion algorithms for motor control applications. The
 * sensor fusion algorithms are used to combine data from multiple sensors to improve the accuracy and reliability
 * of the system.
 * @version 0.1
 * @date 06/05/2025
 */

#include "sensor_fusion.h"

void estimate_velocity_imu(imu_data_t *imu_data, float acceleration, float time_interval){
    // Placeholder for velocity estimation logic
    // v(t) = v(t-1) + [a(t-1) + a(t)] * dt/2
    // where dt is the time interval between measurements
    imu_data->velocity += (100) * (imu_data->prev_acc + acceleration) * time_interval / 2;
    imu_data->prev_acc = acceleration; ///< Update the previous acceleration value
}

float estimate_velocity(float velocity_imu, float velocity_lidar, float velocity_encoder){
    // Placeholder for position estimation logic

    float d_il = fabsf(velocity_imu - velocity_lidar);      ///< Calculate the difference between IMU and Lidar velocities
    float d_ie = fabsf(velocity_imu - velocity_encoder);    ///< Calculate the difference between IMU and Encoder velocities
    float d_le = fabsf(velocity_lidar - velocity_encoder);  ///< Calculate the difference between Lidar and Encoder velocities

    if (d_il <= d_ie && d_il <= d_le) {  ///< If the difference between IMU and Lidar velocities is the smallest
        return (velocity_imu + velocity_lidar) * 0.5f; ///< Return the average of IMU and Lidar velocities
    } else if (d_ie <= d_il && d_ie <= d_le) { ///< If the difference between IMU and Encoder velocities is the smallest
        return (velocity_imu + velocity_encoder) * 0.5f; ///< Return the average of IMU and Encoder velocities
    } else { ///< If the difference between Lidar and Encoder velocities is the smallest
        return (velocity_lidar + velocity_encoder) * 0.5f; ///< Return the average of Lidar and Encoder velocities
    }
}