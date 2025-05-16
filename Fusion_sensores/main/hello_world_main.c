#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SensorFusion";
float aceleration = 0.0f;
float aceleration_prev = 0.0f;
float old_angle = 0.0f; 
float new_angle = 0.0f;
float old_distance = 0.0f;
float new_distance = 0.0f;
float wheel_radious = 0.12; 
/*float time1 = 0.0f;
float time2 = 0.0f;*/
float delta_time = 0.01;
float vel_LiDAR = 0.0f;
float vel_TM151 = 0.0f;
float vel_encoder = 0.0f;
// Placeholder: implement con tu hardware
float getEncoderV(void) {
    //obtener angulo, angulo sin overflow en 360
    float velocidad_radial = (new_angle - old_angle) / delta_time;
    vel_encoder = velocidad_radial * wheel_radious;
    old_angle = new_angle;
    return 0.0f;
}

float getTM151V(void) {
        //aceleracion
        vel_TM151 += 0.5f * (aceleration + aceleration_prev) * delta_time;      
        // 4) Actualizar para siguiente paso
        aceleration_prev = aceleration;


    return 0.0f;
}

float getLiDARV(void) {
    //new_distance = getLiDARDistance();
    vel_LiDAR = (new_distance - old_distance) / delta_time;
    old_distance = new_distance;
    return 0.0f;
}

/**
 * Fusión: promedia las dos lecturas más cercanas
 */
float fuseVelocity(float ve, float vt, float vl) {
    float d_et = fabsf(ve - vt);
    float d_el = fabsf(ve - vl);
    float d_tl = fabsf(vt - vl);
    if (d_et <= d_el && d_et <= d_tl) {
        return (ve + vt) * 0.5f;
    } else if (d_el <= d_et && d_el <= d_tl) {
        return (ve + vl) * 0.5f;
    } else {
        return (vt + vl) * 0.5f;
    }
}

void fusion_task(void *arg) {
    while (1) {
        float ve = getEncoderV();
        float vt = getTM151V();
        float vl = getLiDARV();
        float vf = fuseVelocity(ve, vt, vl);
        ESP_LOGI(TAG, "v_enc=%.2f, v_tm=%.2f, v_lidar=%.2f -> v_fus=%.2f", ve, vt, vl, vf);
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}

void app_main(void) {

    ESP_LOGI(TAG, "Iniciando tarea de fusión de sensores...");
    xTaskCreate(fusion_task, "fusion_task", 2048, NULL, 5, NULL);
}