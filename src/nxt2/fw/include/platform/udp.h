/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __UDP_H__
#define __UDP_H__

#include "systypes.h"

void udp_isr_C(void);
void udp_init(void);
void udp_disable(void);
void udp_enable(sint32 reset);
void udp_reset(void);

sint32 udp_write(uint8* buf, uint16 off, uint16 len);
sint32 udp_read(uint8* buf, uint16 off, uint16 len);

void udp_set_serial_no(uint8 *no, uint16 len);
void udp_set_name(uint8 *name, uint16 len);

sint32 udp_get_state(void);

#define UDP_IO_ERR -1
#define UDP_IO_EOF -2

#define USB_TIMEOUT      0x0BB8
#define END_OF_BUS_RESET ((unsigned int) 0x1 << 12)
#define SUSPEND_INT      ((unsigned int) 0x1 << 8)
#define SUSPEND_RESUME   ((unsigned int) 0x1 << 9)
#define WAKEUP           ((unsigned int) 0x1 << 13)

/* USB standard request codes. */
#define STD_GET_STATUS_ZERO            0x0080
#define STD_GET_STATUS_INTERFACE       0x0081
#define STD_GET_STATUS_ENDPOINT        0x0082

#define STD_CLEAR_FEATURE_ZERO         0x0100
#define STD_CLEAR_FEATURE_INTERFACE    0x0101
#define STD_CLEAR_FEATURE_ENDPOINT     0x0102

#define STD_SET_FEATURE_ZERO           0x0300
#define STD_SET_FEATURE_INTERFACE      0x0301
#define STD_SET_FEATURE_ENDPOINT       0x0302

#define STD_SET_ADDRESS                0x0500
#define STD_GET_DESCRIPTOR             0x0680
#define STD_SET_DESCRIPTOR             0x0700
#define STD_GET_CONFIGURATION          0x0880
#define STD_SET_CONFIGURATION          0x0900
#define STD_GET_INTERFACE              0x0A81
#define STD_SET_INTERFACE              0x0B01
#define STD_SYNCH_FRAME                0x0C82

#define VENDOR_SET_FEATURE_INTERFACE   0x0341
#define VENDOR_CLEAR_FEATURE_INTERFACE 0x0141
#define VENDOR_GET_DESCRIPTOR          0x06c0

#endif /*__UDP_H__*/
