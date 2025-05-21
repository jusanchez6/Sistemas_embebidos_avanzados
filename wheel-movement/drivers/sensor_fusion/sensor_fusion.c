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

void sf_init(imu_data_t *imu_data, encoder_data_t *encoder_data, lidar_data_t *lidar_data){
    // Initialize the IMU data structure
    imu_data->velocity = 0.0f; ///< Initialize velocity to 0
    imu_data->prev_acc = 0.0f; ///< Initialize previous acceleration to 0

    // Initialize the Encoder data structure
    encoder_data->velocity = 0.0f; ///< Initialize velocity to 0
    encoder_data->angle_prev = 0.0f; ///< Initialize previous angle to 0
    encoder_data->radio = 3.0f; ///< Initialize radio to 3 cm

    // Initialize the Lidar data structure
    lidar_data->velocity = 0.0f; ///< Initialize velocity to 0
    lidar_data->prev_distance = 0; ///< Initialize previous distance to 0
    lidar_data->start_distance = 0; ///< Initialize start distance to 0
}

void estimate_velocity_imu(imu_data_t *imu_data, float acceleration, float time_interval){
    // Placeholder for velocity estimation logic
    // v(t) = v(t-1) + [a(t-1) + a(t)] * dt/2
    // where dt is the time interval between measurements

    // Get size of the window
    int win_size = sizeof(imu_data->window) / sizeof(float);
    float vel = (100) * (imu_data->prev_acc + acceleration) * time_interval * 0.5f;

    if(win_size < 100){
        imu_data->velocity += vel; ///< Calculate the velocity
        imu_data->window[win_size] = vel; ///< Store the velocity in the window
    } else {
        imu_data->velocity += vel - imu_data->window[0]; ///< Calculate the velocity
        for(int i = 0; i < win_size - 1; i++){
            imu_data->window[i] = imu_data->window[i + 1]; ///< Shift the window
        }
        imu_data->window[win_size - 1] = vel; ///< Store the velocity in the window
    }
    
    imu_data->prev_acc = acceleration; ///< Update the previous acceleration value
}

void estimate_velocity_encoder(encoder_data_t * encoder_data, float angle, float time_interval){
    // Placeholder for velocity estimation logic
    // v(t) = (angle(t) - angle(t-1)) * radio / dt
    // where dt is the time interval between measurements
    // Get size of the window
    int win_size = sizeof(encoder_data->window) / sizeof(float);
    angle = angle * 3.1415 / 180; ///< Convert angle to radians
    float dist = abs(angle - encoder_data->angle_prev) * encoder_data->radio; ///< Calculate the distance in cm

    if(dist < 15){ ///< If angle difference is not too big
        encoder_data->distance += dist; ///< Store the distance
        float vel =  dist / time_interval;

        if(win_size < 100){
            encoder_data->velocity += vel; ///< Calculate the velocity
            encoder_data->window[win_size] = vel; ///< Store the velocity in the window
        } else {
            encoder_data->velocity += vel - encoder_data->window[0]; ///< Calculate the velocity
            for(int i = 0; i < win_size - 1; i++){
                encoder_data->window[i] = encoder_data->window[i + 1]; ///< Shift the window
            }
            encoder_data->window[win_size - 1] = vel; ///< Store the velocity in the window
        }
        encoder_data->angle_prev = angle; ///< Update the previous angle value
    }
}

void estimate_velocity_lidar(lidar_data_t * lidar_data, uint16_t distance, float time_interval){
    // Placeholder for velocity estimation logic
    // v(t) = (distance(t) - distance(t-1)) / dt
    // where dt is the time interval between measurements
    float dist = abs(lidar_data->prev_distance - distance); ///< Calculate the distance in cm
    lidar_data->velocity = (lidar_data->velocity + (dist / time_interval)) / 2; ///< Calculate the velocity
    lidar_data->prev_distance = distance; ///< Store the previous distance
}

float estimate_position(float position_lidar, float position_encoder){
    // Placeholder for position estimation logic
    // v(t) = (position(t) - position(t-1)) / dt
    // where dt is the time interval between measurements
    float d_le = fabsf(position_lidar - position_encoder);  ///< Calculate the difference between Lidar and Encoder positions

    return (position_lidar + position_encoder) * 0.5f; ///< Return the average of Lidar and Encoder positions
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