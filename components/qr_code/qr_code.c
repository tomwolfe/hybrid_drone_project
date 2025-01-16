// components/qr_code/qr_code.c
#include "qr_code.h"
#include "esp_log.h"
#include "camera.h"
#include "esp_qrcode.h"
#include <esp_timer.h>
#include <stdlib.h> // For malloc, free

static const char *TAG = "QR_CODE";

esp_err_t qr_code_init() {
    return ESP_OK;
}

static void decode_qr_code(camera_fb_t *fb, QueueHandle_t qr_code_queue) {
    if (!fb) {
        ESP_LOGE(TAG, "Received null frame buffer");
        return;
    }

    esp_qrcode_handle_t qrcode_handle = esp_qrcode_create();
    if (!qrcode_handle) {
        ESP_LOGE(TAG, "Failed to create QR code handle");
        return;
    }

    esp_qrcode_config_t config = {
        .max_decode_steps = 8,
        .try_harder = true,
        .roi_x0 = 0, // Example: Scan the entire image
        .roi_y0 = 0,
        .roi_width = fb->width,
        .roi_height = fb->height,
        .enable_grayscale = true
    };
    esp_qrcode_configure(qrcode_handle, &config);

    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {
        .callback = NULL,
        .name = "qr_decode_timer",
        .dispatch_method = ESP_TIMER_TASK,
    };
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_once(timer, 100000);

    esp_qrcode_decode_image(qrcode_handle, fb->buf, fb->width, fb->height);

    esp_timer_stop(timer);
    uint64_t decode_time_us = esp_timer_get_time();

    esp_qrcode_result_t results[4];
    int num_found = esp_qrcode_get_results(qrcode_handle, results, 4);

    ESP_LOGI(TAG, "Found %d QR codes in %llu us", num_found, decode_time_us);

    for (int i = 0; i < num_found; i++) {
        qr_code_result_t result;
        result.data_len = results[i].payload_len;
        result.data = malloc(result.data_len + 1); // +1 for null terminator
        if (result.data) {
            memcpy(result.data, results[i].payload, result.data_len);
            result.data[result.data_len] = '\0'; // Null-terminate the string
            result.confidence = 90; // Example confidence
            ESP_LOGI(TAG, "Decoded QR Code: %s (Confidence: %d%%)", result.data, result.confidence);
            if (xQueueSend(qr_code_queue, &result, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Failed to send QR code result to queue");
            }
            free(result.data);
        } else {
            ESP_LOGE(TAG, "Failed to allocate memory for QR code data");
        }
    }
    esp_qrcode_destroy(qrcode_handle);
}

void qr_code_task(void *pvParameters) {
    QueueHandle_t qr_code_queue = (QueueHandle_t)pvParameters;
    camera_fb_t *fb = NULL;

    while (1) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        decode_qr_code(fb, qr_code_queue);

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

