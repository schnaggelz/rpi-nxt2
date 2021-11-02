#ifndef _NXT_USB_H_
#define _NXT_USB_H_

#include "systypes.h"

#define MAX_DEV_NAME_LEN 16
#define MAX_USB_DATA_LEN 64

typedef enum {
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

#endif /* _NXT_USB_H_ */
