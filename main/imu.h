#ifndef IMU_H
#define IMU_H

// NOTE: Written for BNO055

// Ref? https://docs.circuitpython.org/projects/bno055/en/stable/api.html

#include <stdbool.h>
#include "esp_err.h"

#define IMU_I2C_NUM I2C_NUM_0
#define IMU_I2C_FREQ 400000
#define IMU_I2C_TIMEOUT_MS 1000
#define IMU_TASK_STACK_SIZE 2048
#define IMU_READ_INTERVAL_MS 10
#define IMU_INIT_RETRY_LIMIT 5  
#define IMU_INIT_RETRY_DELAY_MS 500


// Data type for parsed IMU data
typedef struct
{
    bool initialised;
    char* error_msg; 

    // Orientation
    float roll;
    float pitch;
    float yaw;

    // Acceleration
    float accel_x;
    float accel_y;
    float accel_z;

    // Gyroscope
    float gyro_x;
    float gyro_y;
    float gyro_z;

    // Magnetometer
    float mag_x;
    float mag_y;
    float mag_z;

    // Temperature
    float temperature;

    bool valid_orientation;
    bool valid_acceleration;
    bool valid_gyroscope;
    bool valid_magnetometer;
    bool valid_temperature;

    bool sys_calibrated;
    bool gyro_calibrated;
    bool accel_calibrated;
    bool mag_calibrated;

    unsigned char sys_stat_code;
    unsigned char sys_error_code;
} imu_data_t;

void debug_log_imu_data(const imu_data_t *imu_data);

// Initializes global static imu_data structure
// Creates a FreeRTOS task to read and parse IMU data
void imu_init();

// Returns the latest IMU data
imu_data_t imu_get_data();

#endif // IMU_H