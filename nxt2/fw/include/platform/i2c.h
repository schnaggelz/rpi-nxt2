/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Driver for the I2C interface.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __I2C_H__
#define __I2C_H__

#include "systypes.h"

#define I2C_N_PORTS 4

#define I2C_STANDARD_MODE 0x0     /* Use standard i2c protocol */
#define I2C_LEGO_MODE     0x1     /* Use Lego compatible i2c protocol (default) */
#define I2C_ALWAYS_ACTIVE 0x10    /* Keep the i2c driver active between requests */
#define I2C_NO_RELEASE    0x100   /* Do not release the i2c bus between requests */
#define I2C_HIGH_SPEED    0x1000  /* Use high speed I/O (125KHz) */
#define I2C_MAX_IO        0x10000 /* Maximum read/write request length */

#define I2C_ERR_INVALID_PORT -1
#define I2C_ERR_NO_INIT -2
#define I2C_ERR_BUSY -3
#define I2C_ERR_FAULT -4
#define I2C_ERR_INVALID_LENGTH -5
#define I2C_ERR_BUS_BUSY -6

void i2c_disable(uint8 port);
void i2c_disable_all(void);
void i2c_init(void);

sint32 i2c_enable(uint8 port, uint8 mode);
sint32 i2c_status(uint8 port);
sint32 i2c_start(uint8 port,
                 uint8 ext_addr,
                 uint8 int_addr,
                 uint8 *data,
                 uint32 nbytes,
                 uint8 write);

uint8 i2c_wait_ready(uint8 port, uint32 wait_time_ms);

#endif
