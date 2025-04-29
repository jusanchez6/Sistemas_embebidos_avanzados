/**
 * @file s-curve_profile.c
 * @author Striker 1
 * @brief S-curve profile generation for motor control
 * @details This file contains the implementation of an S-curve profile generator for motor control applications. The S-curve profile is used to smoothly accelerate and decelerate a motor, reducing mechanical stress and improving performance.
 * @version 0.1
 * @date 29/04/2025
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SAMPLE_TIME 10 ///< Sample time in milliseconds

typedef struct {
    // S-curve profile parameters filled
    float max_vel; ///< Maximum velocity in units per second
    float max_acc; ///< Maximum acceleration in units per second squared
    float max_decel; ///< Maximum deceleration in units per second squared
    float target_pos; ///< Target position in units
    float initial_pos; ///< Initial position in units
    float time_to_target; ///< Time to reach target position in miliseconds
    // S-curve profile parameters to be filled
    float *time;        ///< Time points
    float *position;    ///< Position points
    float *velocity;    ///< Velocity points (optional)
    int num_points;     ///< Number of points in the profile
} S_Curve_Profile;

/**
 * @brief Generate an S-curve profile for motor control
 * @param prof Pointer to the S_Curve_Profile structure
 */
void generate_s_curve_profile(S_Curve_Profile *prof);

/**
 * @brief Generate the S-curve position at a given time
 * @param prof Pointer to the S_Curve_Profile structure
 * @param time Time point in milliseconds
 * @return Position at the given time
 */
float calculate_s(S_Curve_Profile *prof, float time);

/**
 * @brief Generate the S-curve velocity at a given time
 * @param prof Pointer to the S_Curve_Profile structure
 * @param time Time point in milliseconds
 * @return Velocity at the given time
 */
//float generate_s_curve_velocity(S_Curve_Profile *prof, uint16_t time);

/**
 * @brief Generate the S-curve acceleration at a given time
 * @param prof Pointer to the S_Curve_Profile structure
 * @param time Time point in milliseconds
 * @return Acceleration at the given time
 */
//float generate_s_curve_acceleration(S_Curve_Profile *prof, uint16_t time);

void app_main(void)
{
    // Initialize the S-curve profile parameters
    S_Curve_Profile profile;
    profile.max_vel = 1.0; // Maximum velocity (units/s)
    profile.max_acc = 0.5; // Maximum acceleration (units/s^2)
    profile.max_decel = 0.5; // Maximum deceleration (units/s^2)
    profile.target_pos = 10.0; // Target position (units)
    profile.initial_pos = 0.0; // Current position (units)
    profile.time_to_target = 5000.0; // Time to reach target position (seconds)

    // Allocate memory for the profile points
    profile.time = malloc((profile.time_to_target / SAMPLE_TIME) * sizeof(float));
    profile.position = malloc((profile.time_to_target / SAMPLE_TIME) * sizeof(float));
    profile.velocity = malloc((profile.time_to_target / SAMPLE_TIME) * sizeof(float));
    if (profile.time == NULL || profile.position == NULL || profile.velocity == NULL) {
        printf("Memory allocation failed\n");
        return;
    }

    // Generate the S-curve profile
    generate_s_curve_profile(&profile);

    // Print the generated profile
    for (int i = 0; i < profile.num_points; i++) {
        // Send to PID controller
        printf("Sending to PID controller...");
        printf("Time: %.2f, Position: %.2f, Velocity: %.7f\n", profile.time[i], profile.position[i], profile.velocity[i]);
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); // Delay for 10 ms
    }
}

void generate_s_curve_profile(S_Curve_Profile *prof){
    assert((uint16_t)prof->time_to_target % SAMPLE_TIME == 0.0); ///< Ensure that the time_to_target is a multiple of SAMPLE_TIME
    prof->num_points = prof->time_to_target/SAMPLE_TIME; ///< Number of points in the profile
    // Calculate the time required to reach the target position
    for(float t = 0.0; t < prof->time_to_target; t+=SAMPLE_TIME){
        // Calculate the position, velocity, and acceleration at each time point
        prof->time[(uint16_t)t/SAMPLE_TIME] = t;
        prof->position[(uint16_t)t/SAMPLE_TIME] = prof->initial_pos + prof->target_pos * calculate_s(prof, t); ///< Calculate the position x_d(t) from s(t)
        prof->velocity[(uint16_t)t/SAMPLE_TIME] = (prof->position[(uint16_t)t/SAMPLE_TIME] - prof->position[((uint16_t)t/SAMPLE_TIME) - 1]) / (float)SAMPLE_TIME; ///< Calculate the velocity as finite difference of positions (discrete derivative)
    }
}

float calculate_s(S_Curve_Profile *prof, float time){
    // Calculate the position at a given time using the S-curve profile
    // Equation: s(t) = 3*(t)^2 - 2*(t)^3
    //                    (T)^2     (T)^3
    return (3 * pow(time/prof->time_to_target, 2) - 2 * pow(time/prof->time_to_target, 3));
}

/* void generate_s_curve_profile(S_Curve_Profile *prof, float max_vel, float max_acc, float max_decel, float target_pos){
    // Calculate the time required to reach the target position
    float t_acc = max_vel / max_acc; // Time to accelerate to max velocity
    float t_decel = max_vel / max_decel; // Time to decelerate from max velocity
    float d_acc = 0.5 * max_acc * t_acc * t_acc; // Distance covered during acceleration
    float d_decel = 0.5 * max_decel * t_decel * t_decel; // Distance covered during deceleration

    // Check if the target position can be reached with the given parameters
    if (d_acc + d_decel > target_pos) {
        // Adjust the profile for a shorter distance
        t_acc = sqrt(target_pos / (max_acc + max_decel));
        d_acc = 0.5 * max_acc * t_acc * t_acc;
        d_decel = target_pos - d_acc;
        t_decel = sqrt(d_decel / (0.5 * max_decel));
    }

    // Calculate the total time for the profile
    float total_time = 2 * t_acc + (target_pos - d_acc - d_decel) / max_vel;

    // Allocate memory for the profile points
    prof->num_points = (int)(total_time / 0.1) + 1; // Sample every 0.1 seconds
    prof->time = malloc(prof->num_points * sizeof(float));
    prof->position = malloc(prof->num_points * sizeof(float));
    prof->velocity = malloc(prof->num_points * sizeof(float));

    // Generate the profile points
    for (int i = 0; i < prof->num_points; i++) {
        float t = i * 0.1; // Current time point

        if (t < t_acc) {
            // Acceleration phase
            prof->velocity[i] = max_acc * t;
            prof->position[i] = 0.5 * max_acc * t * t;
        } else if (t < total_time - t_decel) {
            // Constant velocity phase
            prof->velocity[i] = max_vel;
            prof->position[i] = d_acc + max_vel * (t - t_acc);
        } else {
            // Deceleration phase
            float dt = total_time - t;
            prof->velocity[i] = max_vel
        }
    }
} */