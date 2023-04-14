/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_LCD_H__
#define __NXT_LCD_H__

#include "platform/systypes.h"

#define NXT_LCD_WIDTH 100
#define NXT_LCD_DEPTH 8

#ifdef __cplusplus
extern "C" {
#endif

void nxt_lcd_init(const uint8* disp);
void nxt_lcd_power_up(void);
void nxt_lcd_power_down(void);
void nxt_lcd_update();
void nxt_lcd_force_update();

#ifdef __cplusplus
}
#endif

#endif
