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

void estimate_velocity_encoder(encoder_data_t * encoder_data, float angle, float time_interval){
    // Placeholder for velocity estimation logic
    // v(t) = (angle(t) - angle(t-1)) * radio / dt
    // where dt is the time interval between measurements
    angle = angle * 3.1415 / 180; ///< Convert angle to radians
    encoder_data->velocity = (100)* (angle - encoder_data->angle_prev) * encoder_data->radio / time_interval; ///< Calculate the velocity
    encoder_data->angle_prev = angle; ///< Update the previous angle value
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