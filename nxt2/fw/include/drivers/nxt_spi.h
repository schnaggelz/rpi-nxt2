/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_SPI_H__
#define __NXT_SPI_H__

#include "platform/systypes.h"

#define OSC 48054805
#define SPI_BITRATE 2000000

#define CS_PIN (1 << 10)
#define CD_PIN (1 << 12)

/*
 * Note that this is not a normal SPI interface, it is a dodged version as used
 * by the NXT's display. The display does not use MISO because you can only
 * write to it in serial mode.
 * Instead, the MISO pin is not used by the SPI and is instead driven as a PIO
 * pin for controlling CD.
 */
void nxt_spi_init(void);
void nxt_spi_write(uint32 CD, const uint8* data, uint32 n_bytes);
void nxt_spi_set_display(const uint8* disp);
void nxt_spi_refresh(void);

#endif /* __NXT_SPI_H__ */
