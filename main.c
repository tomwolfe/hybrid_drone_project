// main.c - Entry point and core FreeRTOS setup
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "qr_code.h"
#include "ultrasonic.h"
#include "communication.h"
#include "navigation.h"
#include "power_management.h"
#include "magnet_control.h"
#include "logging_task.h"
#include "security.h"
#include "resource_monitor.h"
#include "ota_update.h"
#include "camera.h"
#include "mavlink_handler.h"
#include "visual_odometry.h"

static const char *TAG = "MAIN";

// Task handles
TaskHandle_t qr_code_task_handle;
TaskHandle_t ultrasonic_task_handle;
TaskHandle_t communication_task_handle;
TaskHandle_t navigation_task_handle;
TaskHandle_t power_management_task_handle;
TaskHandle_t magnet_control_task_handle;
TaskHandle_t logging_task_handle;
TaskHandle_t resource_monitor_task_handle;
TaskHandle_t visual_odometry_task_handle;

// Queue handles for inter-task communication
QueueHandle_t ultrasonic_data_queue;
QueueHandle_t qr_code_data_queue;
QueueHandle_t command_queue;
QueueHandle_t telemetry_queue;
QueueHandle_t logging_queue;
QueueHandle_t visual_odometry_queue; // For passing VO data to navigation

// Semaphore for I2C bus access
SemaphoreHandle_t i2c_mutex;

void app_main() {
    ESP_LOGI(TAG, "Enhanced Hybrid Drone Architecture (ArduPilot Edition) - ESP32-S3 Startup");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C bus (Assuming default pins)
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21, // Example SDA pin
        .scl_io_num = GPIO_NUM_22, // Example SCL pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    i2c_mutex = xSemaphoreCreateMutex();

    // Initialize Queues
    ultrasonic_data_queue = xQueueCreate(10, sizeof(ultrasonic_data_t));
    qr_code_data_queue = xQueueCreate(5, sizeof(qr_code_result_t));
    command_queue = xQueueCreate(10, sizeof(command_t));
    telemetry_queue = xQueueCreate(10, sizeof(telemetry_data_t));
    logging_queue = xQueueCreate(20, sizeof(log_message_t));
    visual_odometry_queue = xQueueCreate(5, sizeof(vo_data_t)); // Queue for visual odometry data

    if (ultrasonic_data_queue == NULL || qr_code_data_queue == NULL || command_queue == NULL || telemetry_queue == NULL || logging_queue == NULL || visual_odometry_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queues");
        esp_restart(); // Critical failure, restart
    }

    // Initialize Subsystems
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed");
        // Proceed without camera, some features might be disabled
    }
    ultrasonic_init();
    communication_init(command_queue, telemetry_queue);
    navigation_init(ultrasonic_data_queue, visual_odometry_queue); // Pass VO queue to navigation
    power_management_init();
    magnet_control_init(i2c_mutex);
    logging_init(logging_queue);
    security_init();
    resource_monitor_init();
    ota_update_init();
    mavlink_init(); // Initialize MAVLink communication

    // Create Tasks
    BaseType_t result;

    result = xTaskCreatePinnedToCore(qr_code_task, "QR_Task", 4096, qr_code_data_queue, 5, &qr_code_task_handle, APP_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create QR Code Task");

    result = xTaskCreatePinnedToCore(ultrasonic_task, "Ultra_Task", 4096, ultrasonic_data_queue, 4, &ultrasonic_task_handle, APP_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create Ultrasonic Task");

    result = xTaskCreatePinnedToCore(communication_task, "Comm_Task", 4096, NULL, 3, &communication_task_handle, PRO_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create Communication Task");

    result = xTaskCreatePinnedToCore(navigation_task, "Nav_Task", 4096, NULL, 4, &navigation_task_handle, APP_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create Navigation Task");

    result = xTaskCreatePinnedToCore(power_management_task, "Power_Task", 2048, NULL, 2, &power_management_task_handle, PRO_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create Power Management Task");

    result = xTaskCreatePinnedToCore(magnet_control_task, "Magnet_Task", 2048, NULL, 3, &magnet_control_task_handle, APP_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create Magnet Control Task");

    result = xTaskCreatePinnedToCore(logging_task, "Log_Task", 4096, logging_queue, 1, &logging_task_handle, PRO_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create Logging Task");

    result = xTaskCreatePinnedToCore(resource_monitor_task, "ResMon_Task", 2048, NULL, 1, &resource_monitor_task_handle, PRO_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create Resource Monitor Task");

    result = xTaskCreatePinnedToCore(visual_odometry_task, "VO_Task", 8192, visual_odometry_queue, 4, &visual_odometry_task_handle, APP_CPU_NUM);
    if (result != pdPASS) ESP_LOGE(TAG, "Failed to create Visual Odometry Task");

    if (result == pdPASS) {
        ESP_LOGI(TAG, "All core tasks created successfully.");
    } else {
        ESP_LOGE(TAG, "Task creation failed, system might be unstable.");
    }
}

