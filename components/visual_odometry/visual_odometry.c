#include "visual_odometry.h"
#include "esp_log.h"
#include "camera.h"
#include <esp_timer.h>
#include <stdlib.h>
#include <math.h>
#include <esp_jpg_decode.h> // For JPEG decoding

static const char *TAG = "VISUAL_ODOMETRY";

#define VO_IMAGE_WIDTH  80
#define VO_IMAGE_HEIGHT 60
#define FEATURE_THRESHOLD 50
#define MIN_MATCH_DISTANCE_SQ 100

typedef struct {
    int x, y;
} feature_point_t;

static uint8_t prev_gray_frame[VO_IMAGE_WIDTH * VO_IMAGE_HEIGHT];

esp_err_t visual_odometry_init() {
    memset(prev_gray_frame, 0, sizeof(prev_gray_frame));
    return ESP_OK;
}

// Function to decode JPEG and convert to grayscale
static esp_err_t decode_jpeg_to_grayscale(camera_fb_t *fb, uint8_t *gray_frame) {
    if (!fb || fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "Error: Expected JPEG format");
        return ESP_FAIL;
    }

    jpeg_decode_header_info_t header_info;
    if (esp_jpg_parse_header(fb->buf, fb->len, &header_info) != ESP_OK) {
        ESP_LOGE(TAG, "JPEG header parsing failed");
        return ESP_FAIL;
    }

    uint8_t *decoded_buffer = (uint8_t *)malloc(header_info.width * header_info.height);
    if (!decoded_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for decoded JPEG");
        return ESP_FAIL;
    }

    if (esp_jpg_decode(fb->buf, fb->len, JPG_COLOR_GRAYSCALE, decoded_buffer, header_info.width * header_info.height, 0) != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decoding failed");
        free(decoded_buffer);
        return ESP_FAIL;
    }

    // Downsample and copy to the target grayscale frame
    for (int y = 0; y < VO_IMAGE_HEIGHT; y++) {
        for (int x = 0; x < VO_IMAGE_WIDTH; x++) {
            int src_x = x * header_info.width / VO_IMAGE_WIDTH;
            int src_y = y * header_info.height / VO_IMAGE_HEIGHT;
            if (src_x < header_info.width && src_y < header_info.height) {
                gray_frame[y * VO_IMAGE_WIDTH + x] = decoded_buffer[src_y * header_info.width + src_x];
            }
        }
    }
    free(decoded_buffer);
    return ESP_OK;
}

static int detect_features(const uint8_t *gray_frame, feature_point_t *features, int max_features) {
    int feature_count = 0;
    for (int y = 1; y < VO_IMAGE_HEIGHT - 1; y++) {
        for (int x = 1; x < VO_IMAGE_WIDTH - 1; x++) {
            int center_pixel = gray_frame[y * VO_IMAGE_WIDTH + x];
            if (abs(center_pixel - gray_frame[(y - 1) * VO_IMAGE_WIDTH + x]) > FEATURE_THRESHOLD ||
                abs(center_pixel - gray_frame[(y + 1) * VO_IMAGE_WIDTH + x]) > FEATURE_THRESHOLD ||
                abs(center_pixel - gray_frame[y * VO_IMAGE_WIDTH + (x - 1)]) > FEATURE_THRESHOLD ||
                abs(center_pixel - gray_frame[y * VO_IMAGE_WIDTH + (x + 1)]) > FEATURE_THRESHOLD) {
                if (feature_count < max_features) {
                    features[feature_count].x = x;
                    features[feature_count].y = y;
                    feature_count++;
                }
            }
        }
    }
    return feature_count;
}

static int match_features(const uint8_t *prev_frame, const uint8_t *curr_frame, const feature_point_t *prev_features, int prev_count, const feature_point_t *curr_features, int curr_count, int *matches) {
    int match_count = 0;
    for (int i = 0; i < curr_count; i++) {
        int best_match_index = -1;
        int min_distance_sq = MIN_MATCH_DISTANCE_SQ;
        for (int j = 0; j < prev_count; j++) {
            int dx = curr_features[i].x - prev_features[j].x;
            int dy = curr_features[i].y - prev_features[j].y;
            int distance_sq = dx * dx + dy * dy;
            if (distance_sq < min_distance_sq) {
                min_distance_sq = distance_sq;
                best_match_index = j;
            }
        }
        if (best_match_index != -1) {
            matches[match_count * 2] = best_match_index;
            matches[match_count * 2 + 1] = i;
            match_count++;
        }
    }
    return match_count;
}

// Basic motion estimation including rotation (simplified approach)
static void estimate_motion(const feature_point_t *prev_features, const feature_point_t *curr_features, const int *matches, int match_count, vo_data_t *vo_data) {
    if (match_count >= 5) { // Need a minimum number of matches
        float avg_dx = 0, avg_dy = 0;
        float avg_rotation = 0; // Simplified rotation estimation

        for (int i = 0; i < match_count; i++) {
            int prev_index = matches[i * 2];
            int curr_index = matches[i * 2 + 1];

            avg_dx += (float)(curr_features[curr_index].x - prev_features[prev_index].x);
            avg_dy += (float)(curr_features[curr_index].y - prev_features[prev_index].y);

            // Very simplified rotation estimation: change in angle
            float prev_angle = atan2(prev_features[prev_index].y - (VO_IMAGE_HEIGHT / 2.0f), prev_features[prev_index].x - (VO_IMAGE_WIDTH / 2.0f));
            float curr_angle = atan2(curr_features[curr_index].y - (VO_IMAGE_HEIGHT / 2.0f), curr_features[curr_index].x - (VO_IMAGE_WIDTH / 2.0f));
            avg_rotation += curr_angle - prev_angle;
        }

        vo_data->dx = avg_dx / match_count;
        vo_data->dy = avg_dy / match_count;
        vo_data->dz = 0;
        vo_data->yaw = avg_rotation / match_count; // Assign to yaw (rotation around Z)
        vo_data->roll = 0;
        vo_data->pitch = 0;

        ESP_LOGI(TAG, "Estimated motion: dx=%.2f, dy=%.2f, yaw=%.2f", vo_data->dx, vo_data->dy, vo_data->yaw);

    } else {
        vo_data->dx = 0;
        vo_data->dy = 0;
        vo_data->dz = 0;
        vo_data->yaw = 0;
        vo_data->roll = 0;
        vo_data->pitch = 0;
        ESP_LOGW(TAG, "Insufficient matches for reliable motion estimation");
    }
}

void visual_odometry_task(void *pvParameters) {
    QueueHandle_t vo_queue = (QueueHandle_t)pvParameters;
    camera_fb_t *fb = NULL;
    uint8_t *current_gray_frame = (uint8_t *)malloc(VO_IMAGE_WIDTH * VO_IMAGE_HEIGHT);
    feature_point_t *prev_features = (feature_point_t *)malloc(sizeof(feature_point_t) * 50);
    feature_point_t *curr_features = (feature_point_t *)malloc(sizeof(feature_point_t) * 50);
    int *matches = (int *)malloc(sizeof(int) * 50 * 2);
    int prev_feature_count = 0;

    if (!current_gray_frame || !prev_features || !curr_features || !matches) {
        ESP_LOGE(TAG, "Failed to allocate memory for VO");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed for VO");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (decode_jpeg_to_grayscale(fb, current_gray_frame) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to decode JPEG or convert to grayscale");
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int current_feature_count = detect_features(current_gray_frame, curr_features, 50);
        ESP_LOGI(TAG, "Detected %d features", current_feature_count);

        vo_data_t vo_data = {0};
        if (prev_feature_count > 0 && current_feature_count > 0) {
            int match_count = match_features(prev_gray_frame, current_gray_frame, prev_features, prev_feature_count, curr_features, current_feature_count, matches);
            ESP_LOGI(TAG, "Matched %d features", match_count);
            if (match_count >= 5) {
                estimate_motion(prev_features, curr_features, matches, match_count, &vo_data);
                vo_data.timestamp_ms = esp_timer_get_time() / 1000;
                if (xQueueSend(vo_queue, &vo_data, pdMS_TO_TICKS(10)) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send VO data to queue");
                }
            } else {
                ESP_LOGW(TAG, "Insufficient matches for motion estimation");
            }
        }

        // Prepare for the next iteration
        memcpy(prev_gray_frame, current_gray_frame, VO_IMAGE_WIDTH * VO_IMAGE_HEIGHT);
        memcpy(prev_features, curr_features, current_feature_count * sizeof(feature_point_t));
        prev_feature_count = current_feature_count;

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    free(current_gray_frame);
    free(prev_features);
    free(curr_features);
    free(matches);
    vTaskDelete(NULL);
}
