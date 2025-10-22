#ifndef MAG_ENC_H
#define MAG_ENC_H

// NOTE: Written for AS5600 magnetic encoder

// Ref: https://ams.com/documents/20143/36005/AS5600_Datasheet.pdf

#include <stdbool.h>
#include "esp_err.h"


// Data type for parsed magnetic encoder data
typedef struct
{
    bool initialised;
    char *error_msg;

    // Angle readings
    float raw_angle;       // Raw angle in degrees (0-360)
    float angle;           // Processed angle in degrees (0-360)
    uint16_t raw_angle_raw; // Raw ADC value (0-4095)
    uint16_t angle_raw;    // Processed angle value (0-4095)

    // Status and diagnostics
    bool magnet_detected;
    bool magnet_too_strong;
    bool magnet_too_weak;

    uint8_t agc;       // Automatic Gain Control value
    uint16_t magnitude; // Magnetic field magnitude

    uint8_t status_code;
    bool valid_angle;
    bool valid_magnitude;

} mag_enc_data_t;

void debug_log_mag_enc_data(const mag_enc_data_t *mag_enc_data);

// Initializes global static mag_enc_data structure
// Creates a FreeRTOS task to read and parse magnetic encoder data
void mag_enc_init();

// Returns the latest magnetic encoder data
mag_enc_data_t mag_enc_get_data();

#endif // MAG_ENC_H