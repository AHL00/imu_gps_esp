

// NOTE: https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf

#include "imu.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "i2c_bus.h"

#include "bno055/bno055.h"

#define TAG "imu"
#define ERR_CHECK ESP_ERROR_CHECK_WITHOUT_ABORT
#define ERR_CHECK_ABORT ESP_ERROR_CHECK

#define I2C_ADDR BNO055_I2C_ADDR2

void debug_log_imu_data(const imu_data_t *imu_data)
{
    if (!imu_data->initialised)
    {
        if (imu_data->error_msg)
        {
            ESP_LOGE(TAG, "IMU Data:\n  IMU not initialized\n  Error: %s", imu_data->error_msg);
        }
        else
        {
            ESP_LOGE(TAG, "IMU Data:\n  IMU not initialized");
        }
        return;
    }

    char buffer[512];
    char temp_str[32];

    if (imu_data->valid_temperature)
    {
        snprintf(temp_str, sizeof(temp_str), "%.1f °C", imu_data->temperature);
    }
    else
    {
        snprintf(temp_str, sizeof(temp_str), "Invalid");
    }

    snprintf(buffer, sizeof(buffer),
                "\n"
             "  Orientation: Yaw=%.1f°, Roll=%.1f°, Pitch=%.1f°\n"
             "  Acceleration: X=%.2f m/s², Y=%.2f m/s², Z=%.2f m/s²\n"
             "  Gyroscope: X=%.2f dps, Y=%.2f dps, Z=%.2f dps\n"
             "  Magnetometer: X=%.2f µT, Y=%.2f µT, Z=%.2f µT\n"
             "  Calibration: Sys=%s, Gyro=%s, Accel=%s, Mag=%s\n"
             "  Temperature: %s\n"
             "  System Status Code: 0x%02X\n"
             "  System Error Code: 0x%02X",
             imu_data->yaw, imu_data->roll, imu_data->pitch,
             imu_data->accel_x, imu_data->accel_y, imu_data->accel_z,
             imu_data->gyro_x, imu_data->gyro_y, imu_data->gyro_z,
             imu_data->mag_x, imu_data->mag_y, imu_data->mag_z,
             imu_data->sys_calibrated ? "✓" : "×",
             imu_data->gyro_calibrated ? "✓" : "×",
             imu_data->accel_calibrated ? "✓" : "×",
             imu_data->mag_calibrated ? "✓" : "×",
             temp_str,
             imu_data->sys_stat_code,
             imu_data->sys_error_code);

    ESP_LOGI(TAG, "%s", buffer);
}

imu_data_t new_imu_data_t()
{
    imu_data_t data = {0};
    data.initialised = false;
    data.error_msg = NULL;
    data.roll = 0.0f;
    data.pitch = 0.0f;
    data.yaw = 0.0f;
    data.accel_x = 0.0f;
    data.accel_y = 0.0f;
    data.accel_z = 0.0f;
    data.gyro_x = 0.0f;
    data.gyro_y = 0.0f;
    data.gyro_z = 0.0f;
    data.mag_x = 0.0f;
    data.mag_y = 0.0f;
    data.mag_z = 0.0f;
    data.temperature = 0.0f;
    data.valid_orientation = false;
    data.valid_acceleration = false;
    data.valid_gyroscope = false;
    data.valid_magnetometer = false;
    data.valid_temperature = false;
    data.sys_stat_code = 0;
    data.sys_error_code = 0;

    return data;
}

static i2c_master_dev_handle_t imu_dev_handle;

imu_data_t imu_data;
SemaphoreHandle_t imu_mutex = NULL;

imu_data_t imu_get_data()
{
    if (imu_mutex == NULL)
    {
        ESP_LOGE(TAG, "IMU mutex not initialized");
        return new_imu_data_t(); // Return empty data
    }

    xSemaphoreTake(imu_mutex, portMAX_DELAY);
    imu_data_t data_copy = imu_data;
    xSemaphoreGive(imu_mutex);
    return data_copy;
}

struct bno055_t bno055_hal;

// #define BNO055_WR_FUNC_PTR       s8 (*bus_write)
//         (u8, u8, u8 *, u8)

// #define BNO055_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)
//     bus_write(dev_addr, reg_addr, reg_data, wr_len)

int8_t bno055_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    // Make sure that the dev_addr matches the imu_dev_handle
    if (dev_addr != BNO055_I2C_ADDR2)
    {
        ESP_LOGE(TAG, "IMU WRITE BYTES FAILED: Invalid device address 0x%02X", dev_addr);
        ESP_LOGE(TAG, "Expected device address 0x%02X", BNO055_I2C_ADDR2);
        ESP_LOGE(TAG, "Something went horribly wrong, this is insane");
        return BNO055_ERROR; // BNO055_E_DEV_NOT_FOUND;
    }

    // For a write operation, the register address is typically the first byte sent.
    // We create a temporary buffer to hold the register address and the data.
    uint8_t *buffer = (uint8_t *)malloc(cnt + 1);
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for I2C write buffer");
        return BNO055_ERROR;
    }
    buffer[0] = reg_addr;
    memcpy(buffer + 1, reg_data, cnt);

    esp_err_t ret = i2c_master_transmit(imu_dev_handle, buffer, cnt + 1, IMU_I2C_TIMEOUT_MS);

    free(buffer);

    if (ret == ESP_OK)
    {
        return BNO055_SUCCESS; // Success
    }
    else
    {
        ESP_LOGE(TAG, "IMU WRITE BYTES FAILED: %s", esp_err_to_name(ret));
        return BNO055_ERROR; // BNO055_E_COMM_FAIL; // Failure
    }
}

// #define BNO055_RD_FUNC_PTR       s8
//     (*bus_read)(u8, u8, u8 *, u8)

// #define BNO055_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)
//     bus_read(dev_addr, reg_addr, reg_data, r_len)

int8_t bno055_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    if (dev_addr != BNO055_I2C_ADDR2)
    {
        ESP_LOGE(TAG, "IMU READ BYTES FAILED: Invalid device address 0x%02X", dev_addr);
        return BNO055_ERROR; // BNO055_E_DEV_NOT_FOUND;
    }

    // Use i2c_master_transmit_receive for a combined write (register address) and read transaction
    esp_err_t ret = i2c_master_transmit_receive(imu_dev_handle, &reg_addr, 1, reg_data, cnt, IMU_I2C_TIMEOUT_MS);

    if (ret == ESP_OK)
    {
        return BNO055_SUCCESS; // Success
    }
    else
    {
        ESP_LOGE(TAG, "IMU READ BYTES FAILED: %s", esp_err_to_name(ret));

        return BNO055_ERROR; // BNO055_E_COMM_FAIL; // Failure
    }
}

// #define BNO055_MDELAY_DATA_TYPE             u32

// /*< This refers BNO055 return type as s8 */
// #define BNO055_RETURN_FUNCTION_TYPE         s8
void bno055_delay_msec(u32 msec)
{
    vTaskDelay(pdMS_TO_TICKS(msec));
}

void set_operation_mode(s8 mode)
{
    s8 result = bno055_set_operation_mode(mode);
    if (result != BNO055_SUCCESS)
    {
        ESP_LOGE(TAG, "Failed to set operation mode: %d", result);
    }
}

void set_power_mode(s8 mode)
{
    s8 result = bno055_set_power_mode(mode);
    if (result != BNO055_SUCCESS)
    {
        ESP_LOGE(TAG, "Failed to set power mode: %d", result);
    }
}

void imu_task(void *arg)
{
    static u8 power_mode = BNO055_POWER_MODE_NORMAL;
    static u8 operation_mode = BNO055_OPERATION_MODE_NDOF;

    set_power_mode(power_mode);
    set_operation_mode(operation_mode);

    while (1)
    {
        imu_data_t new_data = new_imu_data_t();
        new_data.initialised = true;
        s8 result;

        // Confirm power and operation modes
        // TODO: Verify whether this is necessary (currently added in case of a sensor crash)
        u8 current_power_mode;
        result = bno055_get_power_mode(&current_power_mode);
        if (result != BNO055_SUCCESS)
        {
            ESP_LOGE(TAG, "Failed to read power mode: %d", result);
        }
        else if (current_power_mode != power_mode)
        {
            ESP_LOGW(TAG, "Power mode changed from %d to %d, resetting to %d", current_power_mode, power_mode, power_mode);
            set_power_mode(power_mode);
        }

        u8 current_operation_mode;
        result = bno055_get_operation_mode(&current_operation_mode);
        if (result != BNO055_SUCCESS)
        {
            ESP_LOGE(TAG, "Failed to read operation mode: %d", result);
        }
        else if (current_operation_mode != operation_mode)
        {
            ESP_LOGW(TAG, "Operation mode changed from %d to %d, resetting to %d", current_operation_mode, operation_mode, operation_mode);
            set_operation_mode(operation_mode);
        }

        u8 sys_stat_code;
        result = bno055_get_sys_stat_code(&sys_stat_code);
        if (result != BNO055_SUCCESS)
        {
            ESP_LOGE(TAG, "Failed to read system status code: %d", result);
        }

        u8 sys_error_code;
        result = bno055_get_sys_error_code(&sys_error_code);
        if (result != BNO055_SUCCESS)
        {
            ESP_LOGE(TAG, "Failed to read system error code: %d", result);
        }

        // Read Euler angles (yaw, roll, pitch)
        struct bno055_euler_t euler_data;
        result = bno055_read_euler_hrp(&euler_data);
        if (result == BNO055_SUCCESS)
        {
            new_data.yaw = euler_data.h / 16.0f;   // Convert to degrees
            new_data.roll = euler_data.r / 16.0f;  // Convert to degrees
            new_data.pitch = euler_data.p / 16.0f; // Convert to degrees
            new_data.valid_orientation = true;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read orientation data: %d", result);
            new_data.valid_orientation = false;
        }

        // Read acceleration data
        struct bno055_accel_t accel_data;
        result = bno055_read_accel_xyz(&accel_data);
        if (result == BNO055_SUCCESS)
        {
            new_data.accel_x = accel_data.x / 100.0f; // Convert to m/s²
            new_data.accel_y = accel_data.y / 100.0f; // Convert to m/s²
            new_data.accel_z = accel_data.z / 100.0f; // Convert to m/s²
            new_data.valid_acceleration = true;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read acceleration data: %d", result);
            new_data.valid_acceleration = false;
        }

        // Read temperature data
        s8 temp;
        result = bno055_read_temp_data(&temp);
        if (result == BNO055_SUCCESS)
        {
            new_data.temperature = (float)temp; // Temperature in °C
            new_data.valid_temperature = true;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read temperature data: %d", result);
            new_data.valid_temperature = false;
        }

        u8 sys, gyro, accel, mag;
        result = bno055_get_mag_calib_stat(&mag);
        result |= bno055_get_accel_calib_stat(&accel);
        result |= bno055_get_gyro_calib_stat(&gyro);
        result |= bno055_get_sys_calib_stat(&sys);

        if (result == BNO055_SUCCESS)
        {
            new_data.sys_calibrated = (sys == 3);
            new_data.gyro_calibrated = (gyro == 3);
            new_data.accel_calibrated = (accel == 3);
            new_data.mag_calibrated = (mag == 3);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read calibration status: %d", result);
            new_data.sys_calibrated = false;
            new_data.gyro_calibrated = false;
            new_data.accel_calibrated = false;
            new_data.mag_calibrated = false;
        }

        // Update the global imu_data safely
        xSemaphoreTake(imu_mutex, portMAX_DELAY);
        imu_data = new_data;
        xSemaphoreGive(imu_mutex);

        vTaskDelay(pdMS_TO_TICKS(IMU_READ_INTERVAL_MS));
    }
}

void imu_init()
{
    if (!is_i2c_initialized())
    {
        ESP_LOGE(TAG, "I2C must be initialized before initializing IMU");
        return;
    }

    int freq;
    get_i2c_config(NULL, NULL, &freq, NULL);

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO055_I2C_ADDR2,
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

    esp_err_t ret = i2c_master_bus_add_device(*bus_handle_ptr, &dev_conf, &imu_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return;
    }

    bno055_hal.bus_read = bno055_i2c_bus_read;
    bno055_hal.bus_write = bno055_i2c_bus_write;
    bno055_hal.delay_msec = bno055_delay_msec;
    bno055_hal.dev_addr = BNO055_I2C_ADDR2;

    ESP_LOGI(TAG, "Initializing BNO055 sensor...");
    s8 init_result = bno055_init(&bno055_hal);

    if (init_result != BNO055_SUCCESS)
    {
        ESP_LOGE(TAG, "BNO055 initialization failed with error code: %d", init_result);

        // Halt further initialization
        imu_data.initialised = false;
        imu_data.error_msg = "BNO055 init failed";
        return;
    }

    u8 chip_id = 0;
    s8 read_result = bno055_read_chip_id(&chip_id);
    if (read_result == BNO055_SUCCESS)
    {
        ESP_LOGI(TAG, "Successfully read Chip ID: 0x%02X (Expected: 0xA0)", chip_id);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read BNO055 Chip ID. Read attempt returned: %d", read_result);
    }

    ESP_LOGI(TAG, "BNO055 initialization successful.");

    // Initialize global static imu_data structure
    imu_data = new_imu_data_t();
    imu_mutex = xSemaphoreCreateMutex();

    // Create FreeRTOS task to read IMU data and perform initial setup stuff
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);

    imu_data.initialised = true;
    imu_data.error_msg = NULL;
}