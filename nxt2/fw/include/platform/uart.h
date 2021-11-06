/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __UART_H__
#define __UART_H__

#include "systypes.h"

int uart_init(uint32 u, uint32 baudRate, uint32 dataBits, uint32 stopBits, sint8 parity);
void uart_close(uint32 u);
int uart_holding(uint32 u);
int uart_get_byte(uint32 u, uint8 *b);
int uart_put_byte(uint32 u, uint8 b);
void uart_put_str(uint32 u, const uint8 *str);
int uart_clear_rx(uint32 u);
int uart_clear_tx(uint32 u);
int uart_set_break(uint32 u);
int uart_clear_break(uint32 u);

#endif /*__UART_H__*/
