#include "navigation.h"
#include "esp_log.h"
#include "ultrasonic.h"
#include "visual_odometry.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

static const char *TAG = "NAVIGATION";
static QueueHandle_t ultrasonic_queue_handle;
static QueueHandle_t visual_odometry_queue_handle;

esp_err_t navigation_init(QueueHandle_t ultrasonic_queue, QueueHandle_t vo_queue) {
    ultrasonic_queue_handle = ultrasonic_queue;
    visual_odometry_queue_handle = vo_queue;
    return ESP_OK;
}

void navigation_task(void *pvParameters) {
    ultrasonic_data_t ultrasonic_data;
    vo_data_t vo_data;

    while (1) {
        // Process ultrasonic data
        if (xQueueReceive(ultrasonic_queue_handle, &ultrasonic_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "Received ultrasonic data: ID=%d, Distance=%.2f cm", ultrasonic_data.id, ultrasonic_data.distance_cm);
            // Implement obstacle avoidance logic here using ultrasonic data
        }

        // Process visual odometry data
        if (xQueueReceive(visual_odometry_queue_handle, &vo_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "Received VO data: dx=%.2f, dy=%.2f, yaw=%.2f", vo_data.dx, vo_data.dy, vo_data.yaw);
            // Integrate VO data into navigation logic (e.g., update position estimate)
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}
