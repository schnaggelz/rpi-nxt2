/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __AT91_TWI_H__
#define __AT91_TWI_H__

#include "systypes.h"

void twi_start_write(uint32 dev_addr, const uint8 *data, uint32 nBytes);
void twi_start_read(uint32 dev_addr, uint8 *data, uint32 nBytes);
void twi_reset(void);

int twi_init(void);
int twi_status(void);

#endif
