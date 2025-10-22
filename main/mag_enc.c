#include "mag_enc.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "i2c_bus.h"
#include <math.h>

#define TAG "mag_enc"
#define ERR_CHECK ESP_ERROR_CHECK_WITHOUT_ABORT
#define ERR_CHECK_ABORT ESP_ERROR_CHECK

#define MAG_ENC_TASK_STACK_SIZE 2048
#define MAG_ENC_I2C_TIMEOUT_MS 1000
#define MAG_ENC_READ_INTERVAL_MS 10

// AS5600 I2C address
#define MAG_ENC_I2C_ADDR 0x36

// AS5600 Register addresses
#define AS5600_REG_ZMCO 0x00
#define AS5600_REG_ZPOS_H 0x01
#define AS5600_REG_ZPOS_L 0x02
#define AS5600_REG_MPOS_H 0x03
#define AS5600_REG_MPOS_L 0x04
#define AS5600_REG_MANG_H 0x05
#define AS5600_REG_MANG_L 0x06
#define AS5600_REG_CONF_H 0x07
#define AS5600_REG_CONF_L 0x08
#define AS5600_REG_RAW_ANGLE_H 0x0C
#define AS5600_REG_RAW_ANGLE_L 0x0D
#define AS5600_REG_ANGLE_H 0x0E
#define AS5600_REG_ANGLE_L 0x0F
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_AGC 0x1A
#define AS5600_REG_MAGNITUDE_H 0x1B
#define AS5600_REG_MAGNITUDE_L 0x1C

void debug_log_mag_enc_data(const mag_enc_data_t *mag_enc_data)
{
    if (!mag_enc_data->initialised)
    {
        if (mag_enc_data->error_msg)
        {
            ESP_LOGE(TAG, "Magnetic Encoder Data:\n  MAG_ENC not initialized\n  Error: %s", mag_enc_data->error_msg);
        }
        else
        {
            ESP_LOGE(TAG, "Magnetic Encoder Data:\n  MAG_ENC not initialized");
        }
        return;
    }

    char buffer[512];
    char magnet_str[64];

    if (mag_enc_data->magnet_detected)
    {
        if (mag_enc_data->magnet_too_strong)
        {
            snprintf(magnet_str, sizeof(magnet_str), "Detected (TOO STRONG)");
        }
        else if (mag_enc_data->magnet_too_weak)
        {
            snprintf(magnet_str, sizeof(magnet_str), "Detected (TOO WEAK)");
        }
        else
        {
            snprintf(magnet_str, sizeof(magnet_str), "Detected (OK)");
        }
    }
    else
    {
        snprintf(magnet_str, sizeof(magnet_str), "Not Detected");
    }

    snprintf(buffer, sizeof(buffer),
             "\n"
             "  Angle: Raw=%.2f°, Processed=%.2f°\n"
             "  Raw Values: Angle=%u, Raw=%u\n"
             "  Magnet Status: %s\n"
             "  AGC: %u\n"
             "  Magnitude: %u\n"
             "  Status Code: 0x%02X",
             mag_enc_data->raw_angle, mag_enc_data->angle,
             mag_enc_data->angle_raw, mag_enc_data->raw_angle_raw,
             magnet_str,
             mag_enc_data->agc,
             mag_enc_data->magnitude,
             mag_enc_data->status_code);

    ESP_LOGI(TAG, "%s", buffer);
}

mag_enc_data_t new_mag_enc_data_t()
{
    mag_enc_data_t data = {0};
    data.initialised = false;
    data.error_msg = NULL;
    data.raw_angle = 0.0f;
    data.angle = 0.0f;
    data.raw_angle_raw = 0;
    data.angle_raw = 0;
    data.magnet_detected = false;
    data.magnet_too_strong = false;
    data.magnet_too_weak = false;
    data.agc = 0;
    data.magnitude = 0;
    data.status_code = 0;
    data.valid_angle = false;
    data.valid_magnitude = false;

    return data;
}

static i2c_master_dev_handle_t mag_enc_dev_handle;

mag_enc_data_t mag_enc_data;
SemaphoreHandle_t mag_enc_mutex = NULL;

mag_enc_data_t mag_enc_get_data()
{
    if (mag_enc_mutex == NULL)
    {
        ESP_LOGE(TAG, "MAG_ENC mutex not initialized");
        return new_mag_enc_data_t(); // Return empty data
    }

    xSemaphoreTake(mag_enc_mutex, portMAX_DELAY);
    mag_enc_data_t data_copy = mag_enc_data;
    xSemaphoreGive(mag_enc_mutex);
    return data_copy;
}

esp_err_t mag_enc_i2c_read_register(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    return i2c_master_transmit_receive(mag_enc_dev_handle, &reg_addr, 1, data, len, MAG_ENC_I2C_TIMEOUT_MS);
}

esp_err_t mag_enc_i2c_write_register(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    uint8_t *buffer = (uint8_t *)malloc(len + 1);
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for I2C write buffer");
        return ESP_ERR_NO_MEM;
    }
    buffer[0] = reg_addr;
    memcpy(buffer + 1, data, len);

    esp_err_t ret = i2c_master_transmit(mag_enc_dev_handle, buffer, len + 1, MAG_ENC_I2C_TIMEOUT_MS);

    free(buffer);

    return ret;
}

uint16_t mag_enc_read_raw_angle()
{
    uint8_t data[2];
    esp_err_t ret = mag_enc_i2c_read_register(AS5600_REG_RAW_ANGLE_H, data, 2);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read raw angle: %s", esp_err_to_name(ret));
        return 0;
    }

    // Combine the two bytes into a 12-bit value (bits 11:0)
    uint16_t raw_angle = ((data[0] & 0x0F) << 8) | data[1];
    return raw_angle;
}

uint16_t mag_enc_read_processed_angle()
{
    uint8_t data[2];
    esp_err_t ret = mag_enc_i2c_read_register(AS5600_REG_ANGLE_H, data, 2);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read processed angle: %s", esp_err_to_name(ret));
        return 0;
    }

    // Combine the two bytes into a 12-bit value (bits 11:0)
    uint16_t angle = ((data[0] & 0x0F) << 8) | data[1];
    return angle;
}

uint8_t mag_enc_read_status()
{
    uint8_t status;
    esp_err_t ret = mag_enc_i2c_read_register(AS5600_REG_STATUS, &status, 1);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read status: %s", esp_err_to_name(ret));
        return 0;
    }

    return status;
}

uint8_t mag_enc_read_agc()
{
    uint8_t agc;
    esp_err_t ret = mag_enc_i2c_read_register(AS5600_REG_AGC, &agc, 1);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read AGC: %s", esp_err_to_name(ret));
        return 0;
    }

    return agc;
}

uint16_t mag_enc_read_magnitude()
{
    uint8_t data[2];
    esp_err_t ret = mag_enc_i2c_read_register(AS5600_REG_MAGNITUDE_H, data, 2);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read magnitude: %s", esp_err_to_name(ret));
        return 0;
    }

    // Combine the two bytes into a 12-bit value
    uint16_t magnitude = ((data[0] & 0x0F) << 8) | data[1];
    return magnitude;
}

void mag_enc_task(void *arg)
{
    while (1)
    {
        mag_enc_data_t new_data = new_mag_enc_data_t();
        new_data.initialised = true;

        // (12-bit ADC value)
        new_data.raw_angle_raw = mag_enc_read_raw_angle();
        new_data.raw_angle = (new_data.raw_angle_raw / 4095.0f) * 360.0f;
        new_data.valid_angle = true;

        // (12-bit ADC value)
        new_data.angle_raw = mag_enc_read_processed_angle();
        new_data.angle = (new_data.angle_raw / 4095.0f) * 360.0f;

        // Status codes: 0 0 MD ML MH 0 0 0
        new_data.status_code = mag_enc_read_status();
        new_data.magnet_detected = (new_data.status_code & 0b00100000) != 0;   // MD
        new_data.magnet_too_weak = (new_data.status_code & 0b00010000) != 0;   // ML
        new_data.magnet_too_strong = (new_data.status_code & 0b00001000) != 0; // MH

        new_data.agc = mag_enc_read_agc();

        new_data.magnitude = mag_enc_read_magnitude();
        new_data.valid_magnitude = true;

        xSemaphoreTake(mag_enc_mutex, portMAX_DELAY);
        mag_enc_data = new_data;
        xSemaphoreGive(mag_enc_mutex);

        vTaskDelay(pdMS_TO_TICKS(MAG_ENC_READ_INTERVAL_MS));
    }
}

void mag_enc_init()
{
    if (!is_i2c_initialized())
    {
        ESP_LOGE(TAG, "I2C must be initialized before initializing MAG_ENC");
        return;
    }

    int freq;
    get_i2c_config(NULL, NULL, &freq, NULL);

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAG_ENC_I2C_ADDR,
        .scl_speed_hz = freq,
        .scl_wait_us = 0,
        .flags.disable_ack_check = false,
    };

    i2c_master_bus_handle_t *const bus_handle_ptr;
    esp_err_t err = get_i2c_bus_handle(&bus_handle_ptr);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get I2C bus handle: %s", esp_err_to_name(err));
        return;
    }

    esp_err_t ret = i2c_master_bus_add_device(*bus_handle_ptr, &dev_conf, &mag_enc_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Initializing AS5600 magnetic encoder...");

    // Verify we can read from the device
    uint8_t zmco_reg;
    ret = mag_enc_i2c_read_register(AS5600_REG_ZMCO, &zmco_reg, 1);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "AS5600 initialization failed: Cannot communicate with device. Error: %s", esp_err_to_name(ret));
        mag_enc_data.initialised = false;
        mag_enc_data.error_msg = "AS5600 communication failed";
        return;
    }

    ESP_LOGI(TAG, "Successfully communicated with AS5600");
    // ESP_LOGI(TAG, "ZMCO Register: 0x%02X", zmco_reg);

    mag_enc_data = new_mag_enc_data_t();
    mag_enc_mutex = xSemaphoreCreateMutex();

    xTaskCreate(mag_enc_task, "mag_enc_task", MAG_ENC_TASK_STACK_SIZE, NULL, 5, NULL);

    mag_enc_data.initialised = true;
    mag_enc_data.error_msg = NULL;

    ESP_LOGI(TAG, "AS5600 initialization successful.");
}