#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <freertos/FreeRTOS.h>
#include <stdint.h>

// Structure to hold visual odometry data
typedef struct {
    float dx;      // Translation in x
    float dy;      // Translation in y
    float dz;      // Translation in z
    float roll;    // Rotation around x-axis
    float pitch;   // Rotation around y-axis
    float yaw;     // Rotation around z-axis
    uint32_t timestamp_ms;
} vo_data_t;

esp_err_t visual_odometry_init();
void visual_odometry_task(void *pvParameters);

#endif // VISUAL_ODOMETRY_H
