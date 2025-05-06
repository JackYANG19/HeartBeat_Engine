#pragma once
#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"


/**
 * @brief Write one or more bytes to a device register over I²C
 *
 * @param port      I2C port number (e.g. I2C_NUM_0)
 * @param dev_addr  7-bit device address
 * @param reg       Register address to write into
 * @param data      Pointer to data buffer to send
 * @param len       Number of bytes to write
 * @return ESP_OK   on success
 *         else error code
 */
esp_err_t i2c_write_reg(i2c_port_t port,
    uint8_t    dev_addr,
    uint8_t    reg,
    const uint8_t *data,
    size_t     len);

/**
* @brief Read one or more bytes from a device register over I²C
*
* @param port      I2C port number (e.g. I2C_NUM_0)
* @param dev_addr  7-bit device address
* @param reg       Register address to read from
* @param data      Pointer to buffer to receive data
* @param len       Number of bytes to read
* @return ESP_OK   on success
*         else error code
*/
esp_err_t i2c_read_reg(i2c_port_t port,
   uint8_t    dev_addr,
   uint8_t    reg,
   uint8_t   *data,
   size_t     len);

void i2c_master_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed);
void i2c_scan(i2c_port_t port);
