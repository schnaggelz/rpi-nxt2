/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Driver for the High Speed / RS485 interface.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef HS_H_
#define HS_H_

#include "systypes.h"

#define HS_RX_PIN  AT91C_PIO_PA5
#define HS_TX_PIN  AT91C_PIO_PA6
#define HS_RTS_PIN AT91C_PIO_PA7

void hs_init(void);
int hs_enable(int baud, int buf_sz);
void hs_disable(void);
uint32 hs_write(uint8 *buf, uint32 off, uint32 len);
uint32 hs_read(uint8 * buf, uint32 off, uint32 len);
uint32 hs_pending(void);

int hs_send(uint8 address, uint8 control, uint8 *data, int offset, int len, uint16 *crc_tab);
int hs_recv(uint8 *data, int len, uint16 *crc_tab, int reset);

#endif /*HS_H_*/
