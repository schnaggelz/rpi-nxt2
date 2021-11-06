/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Driver for the High Speed / RS485 interface.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "platform/hs.h"

#include "platform/aic.h"
#include "platform/display.h"
#include "platform/systick.h"
#include "platform/sensors.h"
#include "platform/usart.h"

#include "platform/at91/at91sam7.h"

#include  <string.h>

/* Max data size. */
#define BUFSZ 128

/* Extra bytes needed for packet header etc. */
#define EXTRA 6

/* Max size of a a packet assuming worse case byte stuffing. */
#define MAXBUF ((BUFSZ+EXTRA)*2)
#define IN_BUF_SZ (MAXBUF/2)
#define OUT_BUF_SZ MAXBUF
#define BAUD_RATE 921600

usart *hs;

/**
 * Enable the high speed RS485 interface, using the specified baud rate and
 * buffer size. If the base rate or buffer size is specified as zero use
 * values that are suitable for BitBus transactions.
 */
int hs_enable(int baud, int buf_sz)
{
    if (baud == 0)
        baud = BAUD_RATE;
    if (buf_sz == 0)
        buf_sz = MAXBUF;
    if (hs == NULL)
    {
        hs = usart_allocate(AT91C_BASE_US0, AT91C_BASE_PDC_US0, buf_sz / 2,
                buf_sz);
        if (hs == NULL)
            return 0;
    }

    /* Enable power to the device. */
    *AT91C_PMC_PCER = (1 << AT91C_ID_US0);
    /* Disable pull ups. */
    *AT91C_PIOA_PPUDR = HS_RX_PIN | HS_TX_PIN | HS_RTS_PIN;
    /* Disable PIO A on I/O lines. */
    *AT91C_PIOA_PDR = HS_RX_PIN | HS_TX_PIN | HS_RTS_PIN;
    /* Enable device control. */
    *AT91C_PIOA_ASR = HS_RX_PIN | HS_TX_PIN | HS_RTS_PIN;
    /* Now program up the device. */
    *AT91C_US0_CR = AT91C_US_RSTSTA;
    *AT91C_US0_CR = AT91C_US_STTTO;
    *AT91C_US0_RTOR = 2400;
    *AT91C_US0_IDR = AT91C_US_TIMEOUT;
    *AT91C_US0_MR = AT91C_US_USMODE_RS485;
    *AT91C_US0_MR &= ~AT91C_US_SYNC;
    *AT91C_US0_MR |= AT91C_US_CLKS_CLOCK | AT91C_US_CHRL_8_BITS
            | AT91C_US_PAR_NONE | AT91C_US_NBSTOP_1_BIT | AT91C_US_OVER;
    *AT91C_US0_BRGR = ((CLOCK_FREQUENCY / 8 / baud)
            | (((CLOCK_FREQUENCY / 8) - ((CLOCK_FREQUENCY / 8 / baud) * baud))
                    / ((baud + 4) / 8)) << 16);

    aic_mask_off(AT91C_ID_US0);
    aic_clear(AT91C_ID_US0);
    usart_enable(hs);

    return 1;
}

void hs_disable(void)
{
    if (hs != NULL)
        usart_free(hs);
    hs = NULL;
    /* Turn off the device and make the pins available for other uses. */
    *AT91C_PMC_PCDR = (1 << AT91C_ID_US0);
    sp_reset(RS485_PORT);
}

void hs_init(void)
{
    /* Initial state is off. */
    hs = NULL;
    hs_disable();
}

uint32 hs_write(uint8 *buf, uint32 off, uint32 len)
{
    return usart_write(hs, buf, off, len);
}

uint32 hs_pending()
{
    return usart_status(hs);
}

uint32 hs_read(uint8 * buf, uint32 off, uint32 len)
{
    return usart_read(hs, buf, off, len);
}

/* The following provides a set of low level BitBus frame I/O routines.
   These functions are C versions of the original Java routines. */

/* Packet construction constants. */
#define BB_FLAG 0x7e
#define BB_ESCAPE 0x7d
#define BB_XOR 0x20
#define CRC_INIT 0xffff
#define ST_FLAG  0
#define ST_ESCAPE  1
#define ST_DATA  2

/* "Class vars". */
uint8 *frame;     /* pointer to he curent frame. */
uint16 frameCRC;  /* Accumulated CRC value. */
uint32 frameLen;  /* current frame length. */
uint32 state;     /* input state. */
uint16 *CRCTable; /* pointer to initialised CRC lookup table. */

/**
 * Add a single byte to the current frame. Include the value in the CRC. Byte
 * stuff if needed.
 */
static void addByte(uint8 b)
{
    /* Update CRC. */
    frameCRC = (uint16) ((frameCRC << 8)
            ^ CRCTable[(b ^ (frameCRC >> 8)) & 0xff]);
    /* Byte stuff?. */
    if (b == BB_FLAG || b == BB_ESCAPE)
    {
        frame[frameLen++] = BB_ESCAPE;
        frame[frameLen++] = (uint8) (b ^ BB_XOR);
    }
    else
        frame[frameLen++] = b;
}

/**
 * Add a series of bytes to the current frame, add to CRC, byte stuff if needed.
 */
static void addBytes(uint8 *data, int len)
{
    while (len-- > 0)
    {
        uint8 b = *data++;
        frameCRC = (uint16) ((frameCRC << 8)
                ^ CRCTable[(b ^ (frameCRC >> 8)) & 0xff]);
        if (b == BB_FLAG || b == BB_ESCAPE)
        {
            frame[frameLen++] = BB_ESCAPE;
            frame[frameLen++] = (uint8) (b ^ BB_XOR);
        }
        else
            frame[frameLen++] = (uint8) b;
    }
}

/**
 * Add the CRC value (FCS Frame Check Sum). Note this value must be byte stuffed
 * but must not impact the actual CRC.
 */
static void addFCS(uint16 FCS)
{
    addByte((uint8) (FCS >> 8));
    addByte((uint8) FCS);
}

/**
 * Create and send a frame.
 */
int hs_send(uint8 address, uint8 control, uint8 *data, int offset, int len,
        uint16 *CRCTab)
{
    // Make sure we have room
    frame = usart_get_write_buffer(hs);
    if (!frame)
        return 0;
    /* Set things up. */
    CRCTable = CRCTab;
    /* Create the frame. */
    frameCRC = CRC_INIT;
    frameLen = 0;
    /* Framing character. */
    frame[frameLen++] = BB_FLAG;
    /* Header. */
    addByte(address);
    addByte(control);
    /* Data. */
    addBytes(data + offset, len);
    addFCS(frameCRC);
    /* Framing character. */
    frame[frameLen++] = BB_FLAG;
    /* Send the data */
    usart_write_buffer(hs, frameLen);
    return frameLen;
}

/**
 * Assemble and return a packet. Uses a state machine to track packet
 * content. Return > 0 packet length and < 0 packet not yet started.
 * If 0 packet being assembled but not yet complete.
 */
int hs_recv(uint8 *data, int len, uint16 *CRCTab, int reset)
{
    uint8 cur;
    frame = data;
    CRCTable = CRCTab;
    /* If we have timed out we may need to reset. */
    if (reset)
        state = ST_FLAG;
    while (usart_read(hs, &cur, 0, 1) > 0)
    {
        switch (state)
        {
        case ST_FLAG:
            /* Waiting for packet start. */
            if (cur == BB_FLAG)
            {
                frameLen = 0;
                frameCRC = CRC_INIT;
                state = ST_DATA;
            }
            break;
        case ST_ESCAPE:
            /* Previous byte was an escape, so escape current byte. */
            cur ^= BB_XOR;
            if (frameLen >= len)
                state = ST_FLAG;
            else
            {
                // Add the byte into the frame.
                frame[frameLen++] = (uint8) cur;
                frameCRC = (uint16) ((frameCRC << 8)
                        ^ CRCTable[(cur ^ (frameCRC >> 8)) & 0xff]);
            }
            state = ST_DATA;
            break;
        case ST_DATA:
            /* Check for end of frame. */
            if (cur == BB_FLAG)
            {
                /* Check that we have a good CRC. */
                state = ST_FLAG;
                if (frameCRC == 0)
                    return frameLen;
            }
            else if (cur == BB_ESCAPE)
                state = ST_ESCAPE;
            else if (frameLen >= len)
                state = ST_FLAG;
            else
            {
                frame[frameLen++] = (uint8) cur;
                frameCRC = (uint16) ((frameCRC << 8)
                        ^ CRCTable[(cur ^ (frameCRC >> 8)) & 0xff]);
            }
            break;
        }
    }
    return state != ST_FLAG ? 0 : -1;
}
