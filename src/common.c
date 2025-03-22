/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "bme280.h"
#include "common.h"

/******************************************************************************/
/*!                               Macros                                      */

#define BME280_SHUTTLE_ID  UINT8_C(0x33)

#define I2C_BUS "/dev/i2c-1"
#define BMP280_ADDR 0x76       // Change to 0x77 if needed
#define PRESSURE_MSB_ADDR 0xF7 // Start of pressure data

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr = BMP280_ADDR;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int file;
    char buf[1];

    // Open I2C bus
    if ((file = open(I2C_BUS, O_RDWR)) < 0)
    {
        perror("Failed to open I2C bus");
        return 1;
    }

    // Select BMP280
    if (ioctl(file, I2C_SLAVE, dev_addr) < 0)
    {
        perror("Failed to select BMP280");
        close(file);
        return 1;
    }
    
    // Write - Set Register Address
    buf[0] = reg_addr;
    if (write(file, buf, 1) != 1)
    {
        perror("Failed to set reg. addr.");
        close(file);
        return 1;
    }

    // Read - Get results
    if (read(file, reg_data, length) != length)
    {
        perror("Failed to read data");
        close(file);
        return 1;
    }
    
    close(file);

    return 0;
}

/*!
 * I2C write function map to COINES platform
 */
BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int file;
    char *buf;

    // Open I2C bus
    if ((file = open(I2C_BUS, O_RDWR)) < 0)
    {
        perror("Failed to open I2C bus");
        return 1;
    }

    // Select BMP280
    if (ioctl(file, I2C_SLAVE, dev_addr) < 0)
    {
        perror("Failed to select BMP280");
        close(file);
        return 1;
    }
    
    // Write - Set Register Address + send data
    buf = malloc(sizeof(char) * (1+length));
    memcpy(buf, &reg_addr,1); // Copy address
    memcpy((char*) (buf + 1), reg_data, length);
    if (write(file, buf, 1+length) != (1+length))
    {
        perror("Failed to set reg. addr.");
        close(file);
        return 1;
    }

    
    close(file);

    return 0;
}

/*!
 * Delay function map to COINES platform
 */
void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    sleep(period/1000);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bme280_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
            case BME280_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BME280_E_COMM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BME280_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BME280_E_INVALID_LEN:
                printf("Error [%d] : Invalid length error. It occurs when write is done with invalid length\r\n", rslt);
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bme280_interface_selection(struct bme280_dev *dev, uint8_t intf)
{
    // Set manually to I2C of Raspberry Pi 3
    dev->intf = intf;
    dev->read = bme280_i2c_read;
    dev->write = bme280_i2c_write;
    dev->delay_us = bme280_delay_us;
    return 0;
}

