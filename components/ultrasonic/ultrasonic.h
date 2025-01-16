// components/ultrasonic/ultrasonic.h
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <freertos/FreeRTOS.h>
#include <stdint.h>

typedef enum {
    SENSOR_FORWARD,
    SENSOR_BACKWARD,
    SENSOR_LEFT,
    SENSOR_RIGHT,
    SENSOR_UPWARD,
    SENSOR_DOWNWARD,
    SENSOR_DOWNWARD_FORWARD
} sensor_id_t;

typedef struct {
    int trigger_pin;
    int echo_pin;
    sensor_id_t id;
} ultrasonic_sensor_config_t;

typedef struct {
    float distance_cm;
    uint32_t timestamp_ms;
    sensor_id_t id;
} ultrasonic_data_t;

esp_err_t ultrasonic_init();
void ultrasonic_task(void *pvParameters);

#endif // ULTRASONIC_H

