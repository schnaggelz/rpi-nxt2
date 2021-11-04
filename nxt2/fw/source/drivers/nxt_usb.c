#include "drivers/nxt_usb.h"

#include "platform/udp.h"

#include <string.h>

/*==============================================================================
 * NXT USB API for LEGO fantom driver
 *=============================================================================*/
#define SERIAL_NO 				"123456780090"
#define NAME 					"nxt" /* default device name */

#define RESET 					0x40000000
#define USB_STREAM 				1

#define USB_STATE_CONFIGURED    0x10
#define USB_STATE_UNCONFIGURED  0x20
#define USB_MASK_CONFIGURED     0xf0

#define SYSTEM_COMMAND_REPLY 	0x01
#define REPLY_COMMAND 			0x02
#define GET_FIRMWARE_VERSION 	0x88
#define GET_DEVICE_INFO 		0x9B

#define DIRECT_MODE 		    0xFF

static volatile sint32 read_data_len;
static volatile uint8 usb_status = USB_NO_INIT;
static uint8 dev_name[MAX_DEV_NAME_LEN] = NAME;
static uint8 usb_buf[MAX_USB_DATA_LEN];

uint8 nxt_usb_1kHz_process(void)
{
    sint32 len;
    sint32 state; /* connection status (not USB status) */
    sint32 i;

    if (usb_status == USB_INIT)
    {
        state = udp_get_state();
        if ((state & USB_MASK_CONFIGURED) == USB_STATE_CONFIGURED)
        {
            /* A connection is established. */
            usb_status = USB_CONNECTED;
        }
    }
    else if (usb_status == USB_CONNECTED)
    {
        /* Keep existing data until USB read is called. */
        if (read_data_len == 0)
        {
            read_data_len = udp_read(usb_buf, 0, MAX_USB_DATA_LEN);
        }
    }
    return usb_status;
}

sint32 nxt_usb_set_name(uint8* name)
{
    if (usb_status == USB_INIT
            && strcmp((const char *)dev_name, (char *)name))
    {
        if (sizeof(name) > MAX_DEV_NAME_LEN)
            return 0;

        strcpy((char *)dev_name, (char *) name);
        udp_set_name((uint8 *)dev_name, sizeof(name));

        return 1;
    }
    return 0;
}

uint8 nxt_usb_connected(void)
{
    if (usb_status == USB_CONNECTED)
    {
        return 1;
    }
    return 0;
}

sint32 nxt_usb_disconnect(void)
{
    if (usb_status == USB_CONNECTED)
    {
        nxt_usb_term();
        nxt_usb_init(); /* default device name is set */
        return 1;
    }
    return 0;
}

sint32 nxt_usb_read(uint8* buf, uint16 off, uint16 len)
{
    if (usb_status != USB_CONNECTED)
        return 0;

    if (len > MAX_USB_DATA_LEN)
        len = MAX_USB_DATA_LEN;

    if (read_data_len > 0)
    {
        if (read_data_len > len)
            read_data_len = len;

        memcpy(&buf[off], usb_buf, read_data_len);

        len = read_data_len;
        read_data_len = 0; /* data is read by application */
    }
    else
    {
        len = 0;
    }
    return len;
}

sint32 nxt_usb_write(uint8* buf, uint16 off, uint16 len)
{
    if (usb_status != USB_CONNECTED)
        return 0;

    return udp_write(buf, off, len);
}

void nxt_usb_init(void)
{
    if (usb_status == USB_NO_INIT)
    {
        udp_init();
        udp_disable();
        udp_set_name((uint8 *)NAME, sizeof(NAME));
        udp_set_serial_no((uint8 *) SERIAL_NO, sizeof(SERIAL_NO));
        udp_enable(0); /* no reset */
        memset(usb_buf, 0, sizeof(usb_buf)); /* flush buffer */

        usb_status = USB_INIT;
    }
}

void nxt_usb_term(void)
{
    udp_disable();
    usb_status = USB_NO_INIT;
}
