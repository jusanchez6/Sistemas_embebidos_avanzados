#include "mov_calculation.h"

void linear_movement(bool forward, float linear_velocity, float angle, float *x_velocity, float *y_velocity) {
    if (forward) {
        *x_velocity = linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = linear_velocity * cosf(angle * PI / 180.0f);
    } else {
        *x_velocity = -linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = -linear_velocity * cosf(angle * PI / 180.0f);
    }
}

void cal_lin_to_ang_velocity(float x_velocity, float y_velocity, uint8_t vel_selection, float *wheel_velocity) {

    float scale = N / R;

    float cos_d = cosf(DELTA);
    float sin_d = sinf(DELTA);

    // wb = 0, so ignore third column
    switch (vel_selection)  // vel_selection is a constant expression
    {
    case 0: // right wheel
        *wheel_velocity = scale * ( -y_velocity );
        break;
    case 1: // left wheel
        *wheel_velocity = scale * ( cos_d * x_velocity + sin_d * y_velocity );
        break;
    case 2: // back wheel
        *wheel_velocity = scale * ( -cos_d * x_velocity + sin_d * y_velocity );
        break;
    
    default:
        break;
    }
}