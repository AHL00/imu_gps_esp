#include <stdio.h>

#include <esp_system.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "gps.h"
#include "i2c_bus.h"
#include "imu.h"

#include "driver/gpio.h"

// Task to constantly check gpio 15 high or low. this is just for debug
#include "driver/adc.h"

#define BUILTIN_LED_GPIO GPIO_NUM_15

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    set_i2c_pins(4, 5);
    // i2c unstable af cause we're using internal pullups and crappy breadboard/jumpers.
    // reduce frequency to help stability
    set_i2c_frequency(100000);
    i2c_master_init();
    uint8_t found_addresses[128];

    scan_i2c_bus(found_addresses, 128, 1000);

    imu_init();
    gps_init(7, 6);

    // Initialise built-in LED GPIO
    gpio_set_direction(BUILTIN_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUILTIN_LED_GPIO, 0);
    bool led_state = false;

    // Countdown before starting monitoring
    for (int i = 5; i > 0; i--)
    {
        ESP_LOGI("MAIN", "Starting in %d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Go into alternate screen mode
    printf("\033[?1049h");
    // Hide cursor
    printf("\033[?25l");

    while (1)
    {
        // Move cursor to top-left
        printf("\033[H");
        // Clear screen from cursor down
        printf("\033[J");

        ESP_LOGI("MAIN", "Sensor Monitor");

        gps_data_t data = gps_get_data();
        debug_log_gps_data(&data);

        imu_data_t imu_data = imu_get_data();
        debug_log_imu_data(&imu_data);

        printf("-------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
