
#include <stdint.h>
#include <stdbool.h>

#include "gps.h"
#include "minmea/minmea.h"
#include "esp_log.h"
#include "driver/uart.h"

gps_data_t new_gps_data_t()
{
    gps_data_t data = {0};
    data.latitude = 0.0;
    data.longitude = 0.0;
    data.lat_dir = 'N';
    data.lon_dir = 'E';
    data.altitude = 0.0;
    data.geoid_sep = 0.0;
    data.speed_kmh = 0.0;
    data.speed_knots = 0.0;
    data.speed_mps = 0.0;
    data.course = 0.0;
    data.fix_quality = 0;
    data.satellites = 0;
    data.hdop = 0.0;
    data.vdop = 0.0;
    data.pdop = 0.0;
    data.valid_time = false;
    data.valid_date = false;
    data.valid_position = false;
    data.valid_altitude = false;
    data.valid_speed = false;
    data.valid_satellites = false;
    return data;
}

static const char *TAG = "gps";

void parse_nmea(const char *sentence, gps_data_t *data)
{
    switch (minmea_sentence_id(sentence, false))
    {
    case MINMEA_SENTENCE_RMC:
    {
        struct minmea_sentence_rmc frame;
        if (minmea_parse_rmc(&frame, sentence))
        {
            ESP_LOGD(TAG, "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                     frame.latitude.value, frame.latitude.scale,
                     frame.longitude.value, frame.longitude.scale,
                     frame.speed.value, frame.speed.scale);
            ESP_LOGD(TAG, "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                     minmea_rescale(&frame.latitude, 1000),
                     minmea_rescale(&frame.longitude, 1000),
                     minmea_rescale(&frame.speed, 1000));
            ESP_LOGD(TAG, "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                     minmea_tocoord(&frame.latitude),
                     minmea_tocoord(&frame.longitude),
                     minmea_tofloat(&frame.speed));

            // Populate gps_data_t structure
            data->latitude = minmea_tocoord(&frame.latitude);
            data->longitude = minmea_tocoord(&frame.longitude);
            data->lat_dir = (frame.latitude.value >= 0) ? 'N' : 'S';
            data->lon_dir = (frame.longitude.value >= 0) ? 'E' : 'W';
            data->speed_knots = minmea_tofloat(&frame.speed);
            data->speed_kmh = data->speed_knots * 1.852;    // Convert knots to km/h
            data->speed_mps = data->speed_knots * 0.514444; // Convert knots to m/s
            data->course = minmea_tofloat(&frame.course);
            data->valid_time = true;
            data->valid_date = true;
            data->valid_position = frame.valid;
            data->valid_speed = frame.valid;
            data->hour = frame.time.hours;
            data->minute = frame.time.minutes;
            data->second = frame.time.seconds;
            data->millisecond = frame.time.microseconds / 1000;
            data->day = frame.date.day;
            data->month = frame.date.month;
            data->year = frame.date.year + 2000; // Assuming 21st century
        }
        else
        {
            ESP_LOGD(TAG, "$xxRMC sentence is not parsed\n");
        }
    }
    break;

    case MINMEA_SENTENCE_GGA:
    {
        struct minmea_sentence_gga frame;
        if (minmea_parse_gga(&frame, sentence))
        {
            ESP_LOGD(TAG, "$xxGGA: fix quality: %d\n", frame.fix_quality);

            data->fix_quality = frame.fix_quality;
            data->satellites = frame.satellites_tracked;
            data->hdop = minmea_tofloat(&frame.hdop);
            data->altitude = minmea_tofloat(&frame.altitude);
            data->geoid_sep = minmea_tofloat(&frame.height);
            data->latitude = minmea_tocoord(&frame.latitude);
            data->longitude = minmea_tocoord(&frame.longitude);
            data->lat_dir = (frame.latitude.value >= 0) ? 'N' : 'S';
            data->lon_dir = (frame.longitude.value >= 0) ? 'E' : 'W';
            data->valid_position = (frame.fix_quality > 0);
            data->valid_altitude = (frame.fix_quality > 0);
            data->valid_satellites = true;
            data->valid_time = true;
            data->hour = frame.time.hours;
            data->minute = frame.time.minutes;
            data->second = frame.time.seconds;
            data->millisecond = frame.time.microseconds / 1000;
        }
        else
        {
            ESP_LOGD(TAG, "$xxGGA sentence is not parsed\n");
        }
    }
    break;

    case MINMEA_SENTENCE_GST:
    {
        struct minmea_sentence_gst frame;
        if (minmea_parse_gst(&frame, sentence))
        {
            ESP_LOGD(TAG, "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
                     frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
                     frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
                     frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
            ESP_LOGD(TAG, "$xxGST fixed point latitude,longitude and altitude error deviation"
                          " scaled to one decimal place: (%d,%d,%d)\n",
                     minmea_rescale(&frame.latitude_error_deviation, 10),
                     minmea_rescale(&frame.longitude_error_deviation, 10),
                     minmea_rescale(&frame.altitude_error_deviation, 10));
            ESP_LOGD(TAG, "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                     minmea_tofloat(&frame.latitude_error_deviation),
                     minmea_tofloat(&frame.longitude_error_deviation),
                     minmea_tofloat(&frame.altitude_error_deviation));

            data->vdop = minmea_tofloat(&frame.latitude_error_deviation);
            data->hdop = minmea_tofloat(&frame.longitude_error_deviation);
            data->pdop = minmea_tofloat(&frame.altitude_error_deviation);
            data->valid_position = true; // GST provides position error estimates, so position is valid
            data->valid_altitude = true; // Altitude error estimate is provided
            data->valid_time = true;     // Time is provided in GST
            data->hour = frame.time.hours;
            data->minute = frame.time.minutes;
            data->second = frame.time.seconds;
            data->millisecond = frame.time.microseconds / 1000;
        }
        else
        {
            ESP_LOGD(TAG, "$xxGST sentence is not parsed\n");
        }
    }
    break;

    case MINMEA_SENTENCE_GSV:
    {
        struct minmea_sentence_gsv frame;
        if (minmea_parse_gsv(&frame, sentence))
        {
            ESP_LOGD(TAG, "$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
            ESP_LOGD(TAG, "$xxGSV: satellites in view: %d\n", frame.total_sats);
            for (int i = 0; i < 4; i++)
                ESP_LOGD(TAG, "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                         frame.sats[i].nr,
                         frame.sats[i].elevation,
                         frame.sats[i].azimuth,
                         frame.sats[i].snr);

            data->valid_satellites = true;
            data->satellites = frame.total_sats; // Total satellites in view
            // TODO: Fix quality
            // data->fix_quality = (frame.total_sats > 0) ? 1 : 0; // Simple fix quality based on satellites in view
        }
        else
        {
            ESP_LOGD(TAG, "$xxGSV sentence is not parsed\n");
        }
    }
    break;

    case MINMEA_SENTENCE_VTG:
    {
        struct minmea_sentence_vtg frame;
        if (minmea_parse_vtg(&frame, sentence))
        {
            ESP_LOGD(TAG, "$xxVTG: true track degrees = %f\n",
                     minmea_tofloat(&frame.true_track_degrees));
            ESP_LOGD(TAG, "        magnetic track degrees = %f\n",
                     minmea_tofloat(&frame.magnetic_track_degrees));
            ESP_LOGD(TAG, "        speed knots = %f\n",
                     minmea_tofloat(&frame.speed_knots));
            ESP_LOGD(TAG, "        speed kph = %f\n",
                     minmea_tofloat(&frame.speed_kph));

            data->speed_knots = minmea_tofloat(&frame.speed_knots);
            data->speed_kmh = minmea_tofloat(&frame.speed_kph);
            data->speed_mps = data->speed_knots * 0.514444; // Convert knots to m/s
            data->course = minmea_tofloat(&frame.true_track_degrees);
            data->valid_speed = true;
            data->valid_position = true; // VTG provides course info, so position is valid
            data->valid_time = true;     // Time is not provided in VTG, but we assume it's valid if we have speed/course
        }
        else
        {
            ESP_LOGD(TAG, "$xxVTG sentence is not parsed\n");
        }
    }
    break;

    case MINMEA_SENTENCE_ZDA:
    {
        struct minmea_sentence_zda frame;
        if (minmea_parse_zda(&frame, sentence))
        {
            ESP_LOGD(TAG, "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                     frame.time.hours,
                     frame.time.minutes,
                     frame.time.seconds,
                     frame.date.day,
                     frame.date.month,
                     frame.date.year,
                     frame.hour_offset,
                     frame.minute_offset);

            data->valid_time = true;
            data->valid_date = true;
            data->hour = frame.time.hours;
            data->minute = frame.time.minutes;
            data->second = frame.time.seconds;
            data->millisecond = frame.time.microseconds / 1000;
            data->day = frame.date.day;
            data->month = frame.date.month;
            data->year = frame.date.year;
        }
        else
        {
            ESP_LOGD(TAG, "$xxZDA sentence is not parsed\n");
        }
    }
    break;

    case MINMEA_INVALID:
    {
        ESP_LOGD(TAG, "$xxxxx sentence is not valid\n");
        // Reset all fields in gps_data_t to indicate no valid data
        *data = new_gps_data_t();
    }
    break;

    default:
    {
        ESP_LOGD(TAG, "$xxxxx sentence is not parsed\n");
    }
    break;
    }
}

void debug_log_gps_data(const gps_data_t *data)
{
    char buffer[512];
    int pos = 0;

    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "\n");

    // Time
    if (data->valid_time)
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "  Time: %02d:%02d:%02d.%03d UTC\n",
                        data->hour, data->minute, data->second, data->millisecond);
    }
    else
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "  Time: Invalid\n");
    }

    // Date
    if (data->valid_date)
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "  Date: %02d/%02d/%04d\n",
                        data->day, data->month, data->year);
    }
    else
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "  Date: Invalid\n");
    }

    // Position
    if (data->valid_position)
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "  Position: %.6f° %c, %.6f° %c\n",
                        fabs(data->latitude), data->lat_dir,
                        fabs(data->longitude), data->lon_dir);
    }
    else
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "  Position: Invalid\n");
    }

    // Altitude
    if (data->valid_altitude)
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "  Altitude: %.2f m\n  Geoid Separation: %.2f m\n",
                        data->altitude, data->geoid_sep);
    }
    else
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "  Altitude: Invalid\n");
    }

    // Speed
    if (data->valid_speed)
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "  Speed: %.2f km/h (%.2f knots, %.2f m/s)\n  Course: %.2f°\n",
                        data->speed_kmh, data->speed_knots, data->speed_mps, data->course);
    }
    else
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "  Speed: Invalid\n  Course: Invalid\n");
    }

    // Satellites
    if (data->valid_satellites)
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "  Fix Quality: %d\n  Satellites in Use: %d\n"
                        "  HDOP: %.2f\n  VDOP: %.2f\n  PDOP: %.2f",
                        data->fix_quality, data->satellites,
                        data->hdop, data->vdop, data->pdop);
    }
    else
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "  Satellites: Invalid");
    }

    ESP_LOGI(TAG, "%s", buffer);
}

static SemaphoreHandle_t gps_mutex = NULL;
static gps_data_t gps_data;

// Currently returns a whole new copy of the data
// This isn't ideal, but idk how mutexes in FreeRTOS work yet
gps_data_t gps_get_data()
{
    if (gps_mutex == NULL)
    {
        ESP_LOGE(TAG, "GPS mutex not initialized");
        return new_gps_data_t(); // Return empty data
    }

    xSemaphoreTake(gps_mutex, portMAX_DELAY);
    gps_data_t data_copy = gps_data;
    xSemaphoreGive(gps_mutex);
    return data_copy;
}

void gps_task(void *arg)
{
    static char line_buffer[512]; // Buffer to accumulate partial lines
    static int line_pos = 0;      // Current position in line buffer

    // UART Data buffer
    uint8_t *data = (uint8_t *)malloc(GPS_UART_BUFFER_SIZE);
    if (!data)
    {
        ESP_LOGE(TAG, "malloc failed");
        return;
    }

    while (1)
    {
        // Read available bytes with shorter timeout for more responsive processing
        int len = uart_read_bytes(GPS_UART_NUM, data, GPS_UART_BUFFER_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            data[len] = '\0'; // NMEA is text; null-terminate for processing
            ESP_LOGD(TAG, "%s", (char *)data);

            // Process each character to split into individual NMEA sentences
            for (int i = 0; i < len; i++)
            {
                char c = data[i];

                // Add character to line buffer if there's space
                if (line_pos < sizeof(line_buffer) - 1)
                {
                    line_buffer[line_pos++] = c;
                }

                if (c == '\n')
                {
                    line_buffer[line_pos] = '\0'; // Null terminate the line

                    // Remove trailing \r\n or \n
                    if (line_pos > 0 && line_buffer[line_pos - 1] == '\n')
                    {
                        line_buffer[line_pos - 1] = '\0';
                        line_pos--;
                    }
                    if (line_pos > 0 && line_buffer[line_pos - 1] == '\r')
                    {
                        line_buffer[line_pos - 1] = '\0';
                        line_pos--;
                    }

                    // Parse the complete NMEA sentence if it's not empty
                    if (line_pos > 0 && line_buffer[0] == '$')
                    {
                        xSemaphoreTake(gps_mutex, portMAX_DELAY);
                        parse_nmea(line_buffer, &gps_data);
                        xSemaphoreGive(gps_mutex);
                    }

                    // Reset line buffer for next sentence
                    line_pos = 0;
                }
                else if (line_pos >= sizeof(line_buffer) - 1)
                {
                    // Line buffer overflow - reset
                    ESP_LOGW(TAG, "Line buffer overflow, resetting");
                    line_pos = 0;
                }
            }

            // ESP_LOGI("DEBUG", "Processed %d bytes from UART", len);
        }
        else
        {
            // Only delay when no data is available to prevent busy waiting
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void gps_init(int rx_pin, int tx_pin)
{
    if (rx_pin >= 0)
    {
        GPS_RX_PIN = rx_pin;
    }

    if (tx_pin >= 0)
    {
        GPS_TX_PIN = tx_pin;
    }

    // Initialize UART for GPS
    const uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_UART_BUFFER_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Initialize gps_data structure
    gps_data = new_gps_data_t();
    gps_mutex = xSemaphoreCreateMutex();
    if (gps_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create GPS mutex");
        return;
    }

    // Create GPS task
    xTaskCreate(gps_task, "gps_task", GPS_TASK_STACK_SIZE, NULL, 5, NULL);
}
