/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_USB_H__
#define __NXT_USB_H__

#include "platform/systypes.h"

#define MAX_DEV_NAME_LEN 16
#define MAX_USB_DATA_LEN 64

typedef enum
{
    USB_NO_INIT,
    USB_INIT,
    USB_CONNECTED
} USB_STATUS_T;

#ifdef __cplusplus
extern "C" {
#endif

extern void nxt_usb_init(void);
extern void nxt_usb_term(void);
extern uint8 nxt_usb_connected(void);
extern sint32 nxt_usb_read(uint8* buf, uint16 off, uint16 len);
extern sint32 nxt_usb_write(uint8* buf, uint16 off, uint16 len);

#ifdef __cplusplus
}
#endif

extern sint32 nxt_usb_set_name(uint8* name);
extern sint32 nxt_disconnect_usb(void);
extern uint8 nxt_usb_1kHz_process(void);

#endif /* __NXT_USB_H__ */
