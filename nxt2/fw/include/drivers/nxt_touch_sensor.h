/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_TOUCH_SENSOR_H__
#define __NXT_TOUCH_SENSOR_H__

#include "platform/systypes.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8 nxt_touch_sensor_is_pressed(uint8 port);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_TOUCH_SENSOR_H__ */