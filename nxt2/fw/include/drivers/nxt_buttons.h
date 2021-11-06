/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_BUTTONS_H__
#define __NXT_BUTTONS_H__

#include "nxt_devices.h"

#define NXT_BUTTON_ENTER 0x1
#define NXT_BUTTON_ESCAPE 0x8
#define NXT_BUTTON_LEFT 0x2
#define NXT_BUTTON_RIGHT 0x4

#ifdef __cplusplus
extern "C" {
#endif

uint8 nxt_check_buttons_event(uint8 filter);
uint8 nxt_enter_button_is_pressed(void);
uint8 nxt_escape_button_is_pressed(void);
uint8 nxt_right_button_is_pressed(void);
uint8 nxt_left_button_is_pressed(void);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_BUTTONS_H__ */
