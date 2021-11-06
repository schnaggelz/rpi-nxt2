/*******************************************************************************
 * Copyright (C) 2012 T. Reich
 *
 * This file is part of rpi-nxt2 experiment. Some code was inspired by the
 * Lejos project.
 *
 * NXT display driver code.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "platform/systypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void nxt_display_init(void);
void nxt_display_update(void);
void nxt_display_force_update(void);
void nxt_display_clear(uint8 update);
void nxt_display_set_auto_update(uint8 flag);

void nxt_display_goto_xy(uint8 x, uint8 y);
void nxt_display_char(int c);
void nxt_display_string(const char* str);
void nxt_display_signed(sint32 val, uint32 places);
void nxt_display_hex(uint32 val, uint32 places);
void nxt_display_unsigned(uint32 val, uint32 places);
void nxt_display_bitmap_copy(const uint8* data, uint32 width, uint32 depth,
                             uint32 x, uint32 y);

#ifdef __cplusplus
}
#endif

uint8* nxt_display_get_buffer(void);

extern uint8 nxt_display_tick;
extern uint8 nxt_display_auto_update;

#endif
