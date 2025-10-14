#include "i2c_bus.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TAG "i2c"

static SemaphoreHandle_t i2c_mutex = NULL;
static i2c_master_bus_handle_t bus_handle = NULL;
static bool i2c_initialized = false;
static int i2c_sda_pin = 21;          // Default SDA pin
static int i2c_scl_pin = 22;          // Default SCL pin
static int i2c_frequency_hz = 400000; // Default frequency 400k
static int i2c_timeout_ms = 1000;     // Default timeout 1000ms

void set_i2c_pins(int sda_pin, int scl_pin)
{
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_sda_pin = sda_pin;
    i2c_scl_pin = scl_pin;
    xSemaphoreGive(i2c_mutex);
}

void set_i2c_frequency(int frequency_hz)
{
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_frequency_hz = frequency_hz;
    xSemaphoreGive(i2c_mutex);
}

void set_i2c_timeout(int timeout_ms)
{
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_timeout_ms = timeout_ms;
    xSemaphoreGive(i2c_mutex);
}

esp_err_t i2c_master_init()
{
    if (i2c_initialized)
    {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    i2c_master_bus_config_t conf = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = i2c_sda_pin,
        .scl_io_num = i2c_scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret;
    if (bus_handle != NULL)
    {
        ESP_LOGW(TAG, "I2C bus already created");
        return ESP_OK;
    }

    ret = i2c_new_master_bus(&conf, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_initialized = true;

    xSemaphoreGive(i2c_mutex);

    ESP_LOGI(TAG, "I2C initialized successfully on SDA pin %d, SCL pin %d", i2c_sda_pin, i2c_scl_pin);

    return ESP_OK;
}

void get_i2c_config(int *sda_pin, int *scl_pin, int *frequency_hz, int *timeout_ms)
{
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    if (sda_pin)
        *sda_pin = i2c_sda_pin;
    if (scl_pin)
        *scl_pin = i2c_scl_pin;
    if (frequency_hz)
        *frequency_hz = i2c_frequency_hz;
    if (timeout_ms)
        *timeout_ms = i2c_timeout_ms;
    xSemaphoreGive(i2c_mutex);
}

bool is_i2c_initialized()
{
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    bool initialized = i2c_initialized;
    xSemaphoreGive(i2c_mutex);
    return initialized;
}

esp_err_t get_i2c_bus_handle(i2c_master_bus_handle_t **handle)
{
    if (!i2c_initialized || bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C not initialized, bus handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    // We don't need a mutex for this as bus_handle is only
    // set once during initialization and we're borrowing
    // a const pointer
    *handle = &bus_handle;
    return ESP_OK;
}

esp_err_t scan_i2c_bus(uint8_t *found_addresses, size_t max_addresses, int timeout_ms)
{
    if (!i2c_initialized || bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C not initialized, cannot scan bus");
        return ESP_ERR_INVALID_STATE;
    }

    size_t found_count = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        if (found_count >= max_addresses)
            break;

        esp_err_t ret = i2c_master_probe(bus_handle, addr, timeout_ms);
        if (ret == ESP_OK)
        {
            found_addresses[found_count++] = addr;
            ESP_LOGI(TAG, "Found I2C device at address 0x%02X", addr);
        }
    }

    return ESP_OK;
}
