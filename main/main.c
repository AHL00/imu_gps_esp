#include <stdio.h>

#include <esp_system.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "gps.h"


static const char *TAG = "gps_uart";

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    gps_init();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        gps_data_t data = gps_get_data();
        debug_log_gps_data(&data);
    }
}
