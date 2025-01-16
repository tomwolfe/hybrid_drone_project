// components/communication/communication.h
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <freertos/FreeRTOS.h>
#include <stdint.h>

typedef struct {
    int command_id;
    // Add specific command parameters
} command_t;

typedef struct {
    float battery_voltage;
    float cpu_load;
    // Add other relevant telemetry data
} telemetry_data_t;

esp_err_t communication_init(QueueHandle_t command_queue, QueueHandle_t telemetry_queue);
void communication_task(void *pvParameters);
esp_err_t send_telemetry(const telemetry_data_t *data);
esp_err_t receive_command(command_t *command);

#endif // COMMUNICATION_H

