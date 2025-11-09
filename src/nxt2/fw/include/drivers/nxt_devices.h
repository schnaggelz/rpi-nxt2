/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_DEVICES_H__
#define __NXT_DEVICES_H__

#include "platform/systypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void nxt_devices_init(void);
void nxt_devices_background(void);
void nxt_devices_exit(void);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_DEVICES_H__ */
