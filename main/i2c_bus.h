#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
    This module is a way to manage the I2C master bus configuration and initialization.
    It does not handle device management or communication beyond consolidating the bus handle.
*/

void set_i2c_pins(int sda_pin, int scl_pin);
void set_i2c_frequency(int frequency_hz);
void set_i2c_timeout(int timeout_ms);

esp_err_t i2c_master_init();

void get_i2c_config(int *sda_pin, int *scl_pin, int *frequency_hz, int *timeout_ms);
bool is_i2c_initialized();

// Opaque handle to the I2C master bus
typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;

esp_err_t get_i2c_bus_handle(i2c_master_bus_handle_t **handle);
esp_err_t scan_i2c_bus(uint8_t *found_addresses, size_t max_addresses, int timeout_ms);

#endif