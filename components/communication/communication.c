// components/communication/communication.c
#include "communication.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_tls.h"
#include "esp_mac.h"
#include <stdio.h>

static const char *TAG = "COMMUNICATION";
static esp_mqtt_client_handle_t mqtt_client;
static QueueHandle_t command_queue_handle;
static QueueHandle_t telemetry_queue_handle;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, "/drone/command", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA, TOPIC=%.*s DATA=%.*s", event->topic_len, event->topic, event->data_len, event->data);
        cJSON *json = cJSON_Parse(event->data);
        if (json != NULL) {
            command_t command;
            cJSON *command_id_json = cJSON_GetObjectItem(json, "command_id");
            if (cJSON_IsNumber(command_id_json)) {
                command.command_id = command_id_json->valueint;
                if (xQueueSend(command_queue_handle, &command, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send command to queue");
                }
            } else {
                ESP_LOGW(TAG, "Invalid command format");
            }
            cJSON_Delete(json);
        } else {
            ESP_LOGW(TAG, "Failed to parse JSON command");
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err_num);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)",  errno, strerror(errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

esp_err_t communication_init(QueueHandle_t command_queue, QueueHandle_t telemetry_queue) {
    command_queue_handle = command_queue;
    telemetry_queue_handle = telemetry_queue;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.uri = "mqtts://your_mqtt_broker:8883", // Use secure MQTT
        .broker.username = "your_mqtt_username",
        .broker.password = "your_mqtt_password",
        .broker.transport = MQTT_TRANSPORT_OVER_SSL,
        .broker.skip_cert_common_name_check = true, // For testing, consider proper certs in production
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

    return ESP_OK;
}

esp_err_t send_telemetry(const telemetry_data_t *data) {
    if (!mqtt_client) return ESP_FAIL;

    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "battery_voltage", data->battery_voltage);
    cJSON_AddNumberToObject(json, "cpu_load", data->cpu_load);
    char *json_str = cJSON_PrintUnformatted(json);
    if (json_str != NULL) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, "/drone/telemetry", json_str, 1, 0, 0); // QoS 1 for reliability
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d, data=%s", msg_id, json_str);
        free(json_str);
    } else {
        ESP_LOGE(TAG, "Failed to serialize telemetry data to JSON");
    }
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t receive_command(command_t *command) {
    if (xQueueReceive(command_queue_handle, command, pdMS_TO_TICKS(100)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

void communication_task(void *pvParameters) {
    telemetry_data_t telemetry;
    while (1) {
        // Example telemetry data
        telemetry.battery_voltage = 12.34f;
        telemetry.cpu_load = 0.5f; // Example CPU load
        send_telemetry(&telemetry);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

