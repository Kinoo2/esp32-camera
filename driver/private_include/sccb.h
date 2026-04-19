/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#ifndef __SCCB_H__
#define __SCCB_H__
/* Forward declaration of i2c_master_bus_handle_t from driver/i2c_master.h.
   Declared locally rather than pulling in the full header so this public
   interface compiles on IDF 5.1 (header absent) and IDF 6.0+ (header
   relocated to the esp_driver_i2c component). Implementations that need
   the full type (e.g. sccb-ng.c) include driver/i2c_master.h themselves. */
typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdint.h>
int SCCB_Init(int pin_sda, int pin_scl);
int SCCB_Use_Port(i2c_master_bus_handle_t i2c_bus, SemaphoreHandle_t lock);
int SCCB_Deinit(void);
int SCCB_Probe(uint8_t slv_addr);
uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);
uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg);
int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data);
uint8_t SCCB_Read16_Validate(uint8_t slv_addr, uint16_t reg);
int SCCB_Write16_Validate(uint8_t slv_addr, uint16_t reg, uint8_t data);
uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg);
int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data);
#endif // __SCCB_H__
