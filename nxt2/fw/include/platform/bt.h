/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Driver for the AT91SAM7's Bluetooth PIO.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __BT_H__
#define __BT_H__

#include "systypes.h"

#define MSG_BEGIN_INQUIRY 0
#define MSG_CANCEL_INQUIRY 1
#define MSG_CONNECT 2
#define MSG_OPEN_PORT 3
#define MSG_LOOKUP_NAME 4
#define MSG_ADD_DEVICE 5
#define MSG_REMOVE_DEVICE 6
#define MSG_DUMP_LIST 7
#define MSG_CLOSE_CONNECTION 8
#define MSG_ACCEPT_CONNECTION 9
#define MSG_PIN_CODE 10
#define MSG_OPEN_STREAM 11
#define MSG_START_HEART 12
#define MSG_HEARTBEAT 13
#define MSG_INQUIRY_RUNNING 14
#define MSG_INQUIRY_RESULT 15
#define MSG_INQUIRY_STOPPED 16
#define MSG_LOOKUP_NAME_RESULT 17
#define MSG_LOOKUP_NAME_FAILURE 18
#define MSG_CONNECT_RESULT 19
#define MSG_RESET_INDICATION 20
#define MSG_REQUEST_PIN_CODE 21
#define MSG_REQUEST_CONNECTION 22
#define MSG_LIST_RESULT 23
#define MSG_LIST_ITEM 24
#define MSG_LIST_DUMP_STOPPED 25
#define MSG_CLOSE_CONNECTION_RESULT 26
#define MSG_PORT_OPEN_RESULT 27
#define MSG_SET_DISCOVERABLE 28
#define MSG_CLOSE_PORT 29
#define MSG_CLOSE_PORT_RESULT 30
#define MSG_PIN_CODE_ACK 31
#define MSG_DISCOVERABLE_ACK 32
#define MSG_SET_FRIENDLY_NAME 33
#define MSG_SET_FRIENDLY_NAME_ACK 34
#define MSG_GET_LINK_QUALITY 35
#define MSG_LINK_QUALITY_RESULT 36
#define MSG_SET_FACTORY_SETTINGS 37
#define MSG_SET_FACTORY_SETTINGS_ACK 38
#define MSG_GET_LOCAL_ADDR 39
#define MSG_GET_LOCAL_ADDR_RESULT 40
#define MSG_GET_FRIENDLY_NAME 41
#define MSG_GET_DISCOVERABLE 42
#define MSG_GET_PORT_OPEN 43
#define MSG_GET_FRIENDLY_NAME_RESULT 44
#define MSG_GET_DISCOVERABLE_RESULT 45
#define MSG_GET_PORT_OPEN_RESULT 46
#define MSG_GET_VERSION 47
#define MSG_GET_VERSION_RESULT 48
#define MSG_GET_BRICK_STATUSBYTE_RESULT 49
#define MSG_SET_BRICK_STATUSBYTE_RESULT 50
#define MSG_GET_BRICK_STATUSBYTE 51
#define MSG_SET_BRICK_STATUSBYTE 52
#define MSG_GET_OPERATING_MODE 53
#define MSG_SET_OPERATING_MODE 54
#define MSG_SET_OPERATING_MODE_RESULT 55
#define MSG_GET_CONNECTION_STATUS 56
#define MSG_CONNECTION_STATUS_RESULT 57
#define MSG_GOTO_DFUMODE 58

#define STREAM_BREAKING_MODE 0
#define DONT_BREAK_STREAM_MODE 1

#define BT_RX_PIN AT91C_PIO_PA21
#define BT_TX_PIN AT91C_PIO_PA22
#define BT_SCK_PIN AT91C_PIO_PA23
#define BT_RTS_PIN AT91C_PIO_PA24
#define BT_CTS_PIN AT91C_PIO_PA25
#define BT_CS_PIN AT91C_PIO_PA31
#define BT_RST_PIN AT91C_PIO_PA11
#define BT_ARM7_CMD_PIN AT91C_PIO_PA27

#define BAUD_RATE 460800
#define CLOCK_RATE 48054850

void bt_init(void);
void bt_clear_arm7_cmd(void);
void bt_set_arm7_cmd(void);
void bt_set_reset_high(void);
void bt_set_reset_low(void);
void bt_send(uint8 *buf, uint32 len);
void bt_receive(uint8 * buf);
void bt_reset(void);

uint32 bt_get_mode(void);
uint32 bt_write(uint8 *buf, uint32 off, uint32 len);
uint32 bt_read(uint8 * buf, uint32 off, uint32 len);
uint32 bt_pending(void);

#endif /*__BT_H__*/
