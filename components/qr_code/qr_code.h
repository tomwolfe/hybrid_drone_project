// components/qr_code/qr_code.h
#ifndef QR_CODE_H
#define QR_CODE_H

#include <freertos/FreeRTOS.h>
#include <stdint.h>

typedef struct {
    uint8_t* data;
    size_t data_len;
    int confidence;
    // Add Region of Interest data if needed
} qr_code_result_t;

esp_err_t qr_code_init();
void qr_code_task(void *pvParameters);

#endif // QR_CODE_H

