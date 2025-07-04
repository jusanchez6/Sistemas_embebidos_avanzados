#ifndef MOV_CALCULATION_H
#define MOV_CALCULATION_H

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define PI 3.14159265358979323846
#define DELTA PI/6.0  ///< Angle in degrees for the transformation (orientation angle of the body)
#define N     16      ///< Reduction factor for the transformation (planetary gear ratio)
#define R     0.0325f ///< Radio of the wheel in cm

void linear_movement(bool forward, float linear_velocity, float angle, float *x_velocity, float *y_velocity);

void cal_lin_to_ang_velocity(float x_velocity, float y_velocity, uint8_t vel_selection, float *wheel_velocity);

#endif // MOV_CALCULATION_H