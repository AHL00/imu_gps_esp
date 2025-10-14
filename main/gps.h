#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

#define GPS_UART_NUM UART_NUM_1
#define GPS_UART_BAUD_RATE 9600
#define GPS_UART_BUFFER_SIZE 1024
#define GPS_UART_TIMEOUT_MS 1000
#define GPS_TASK_STACK_SIZE 2048

static int GPS_RX_PIN = 7;
static int GPS_TX_PIN = 6;

// Data type for parsed GPS data from NMEA sentences
typedef struct
{
    // Time and date
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    uint8_t day;
    uint8_t month;
    uint16_t year;

    // Position
    double latitude;
    double longitude;
    char lat_dir;
    char lon_dir;
    double altitude;
    double geoid_sep;

    // Velocity and course
    double speed_kmh;
    double speed_knots;
    double speed_mps;
    double course;

    uint8_t fix_quality;
    uint8_t satellites;
    double hdop;
    double vdop;
    double pdop;

    bool valid_time;
    bool valid_date;
    bool valid_position;
    bool valid_altitude;
    bool valid_speed;
    bool valid_satellites;

    // // Raw NMEA sentence for reference 
    // char raw_sentence[128];
} gps_data_t;


// Initializes global static gps_data structure
// Creates a FreeRTOS task to read and parse GPS data
// Pass -1 to use default pins (7=RX, 6=TX)
void gps_init(int rx_pin, int tx_pin);

// Only call after gps_init() has been called
// Currently returns a whole new copy of the data
// This isn't ideal, but idk how mutexes in FreeRTOS work yet
gps_data_t gps_get_data();

void debug_log_gps_data(const gps_data_t *data);

#endif // GPS_H
