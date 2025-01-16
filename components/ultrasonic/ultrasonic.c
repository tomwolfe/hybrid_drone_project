// components/ultrasonic/ultrasonic.c
#include "ultrasonic.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <sys/time.h>

static const char *TAG = "ULTRASONIC";

#define NUM_SENSORS 7

static const ultrasonic_sensor_config_t sensors[NUM_SENSORS] = {
    {GPIO_NUM_16, GPIO_NUM_17, SENSOR_FORWARD},
    {GPIO_NUM_18, GPIO_NUM_19, SENSOR_BACKWARD},
    {GPIO_NUM_26, GPIO_NUM_27, SENSOR_LEFT},
    {GPIO_NUM_14, GPIO_NUM_12, SENSOR_RIGHT},
    {GPIO_NUM_25, GPIO_NUM_33, SENSOR_UPWARD},
    {GPIO_NUM_32, GPIO_NUM_35, SENSOR_DOWNWARD},
    {GPIO_NUM_4,  GPIO_NUM_5,  SENSOR_DOWNWARD_FORWARD}
};

esp_err_t ultrasonic_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 0,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    for (int i = 0; i < NUM_SENSORS; i++) {
        io_conf.pin_bit_mask |= (1ULL << sensors[i].trigger_pin);
    }
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.mode = GPIO_MODE_INPUT;
    for (int i = 0; i < NUM_SENSORS; i++) {
        io_conf.pin_bit_mask = (1ULL << sensors[i].echo_pin);
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
    return ESP_OK;
}

static bool get_sensor_reading(const ultrasonic_sensor_config_t *sensor, float *distance) {
    gpio_set_level(sensor->trigger_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(sensor->trigger_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(sensor->trigger_pin, 0);

    int64_t start_time = esp_timer_get_time();
    int64_t timeout_time = start_time + 10000;

    while (gpio_get_level(sensor->echo_pin) == 0) {
        if (esp_timer_get_time() > timeout_time) return false;
    }

    int64_t echo_start_time = esp_timer_get_time();
    while (gpio_get_level(sensor->echo_pin) == 1) {
        if (esp_timer_get_time() > timeout_time * 2) return false;
    }
    int64_t echo_end_time = esp_timer_get_time();

    int64_t echo_duration_us = echo_end_time - echo_start_time;
    *distance = echo_duration_us * 0.0343 / 2;
    return true;
}

void ultrasonic_task(void *pvParameters) {
    QueueHandle_t ultrasonic_queue = (QueueHandle_t)pvParameters;
    while (1) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            float distance;
            if (get_sensor_reading(&sensors[i], &distance)) {
                ultrasonic_data_t data = {
                    .distance_cm = distance,
                    .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
                    .id = sensors[i].id
                };
                if (xQueueSend(ultrasonic_queue, &data, pdMS_TO_TICKS(10)) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send ultrasonic data to queue");
                }
            } else {
                ESP_LOGW(TAG, "Ultrasonic sensor %d reading timed out", sensors[i].id);
                ultrasonic_data_t error_data = {
                    .distance_cm = -1.0f, // Indicate error
                    .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
                    .id = sensors[i].id
                };
                xQueueSend(ultrasonic_queue, &error_data, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

