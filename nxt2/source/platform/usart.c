/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Generic methods to support I/O to an AT91 usart. The actual detailed
* hardware setup must be performed by a hardware specific set of
* functions before calling the enable methods provided here.
*
* License notes see LICENSE.txt
*******************************************************************************/

/**
 * Generic methods to support I/O to an AT91 usart. The actual detailed
 * hardware setup must be performed by a hardware specific set of
 * functions before calling the enable methos provided here.
 */
#include "platform/usart.h"

#include "platform/aic.h"
#include "platform/hs.h"
#include "platform/ports.h"
#include "platform/systick.h"

#include "platform/at91/at91sam7.h"

#include <string.h>

/**
 * Allocate the required buffers and structures from system memory to allow
 * use of the specified usart and associated dma channel.
 */
usart* usart_allocate(AT91S_USART *dev, AT91S_PDC *dma, int inSz, int outSz)
{
    usart *us;
    /* Do memory allocation for buffer space. */
    uint8 *mem = system_allocate(
            sizeof(usart) + inSz * BUF_CNT + outSz * BUF_CNT);
    if (mem == NULL)
        return NULL;

    us = (usart *) mem;
    us->dma = dma;
    us->dev = dev;
    mem += sizeof(usart);
    us->in_buf[0] = mem;
    mem += inSz;
    us->in_buf[1] = mem;
    mem += inSz;
    us->out_buf[0] = mem;
    mem += outSz;
    us->out_buf[1] = mem;
    us->in_offset = 0;
    us->in_base = us->out_base = 0;
    us->in_size = inSz;
    us->out_size = outSz;

    return us;
}

/**
 * Enable the specified usart device. after this call the device may
 * have read/read operations performed on it.
 */
void usart_enable(usart *us)
{
    AT91S_PDC *dma = us->dma;
    AT91S_USART *dev = us->dev;
    uint8 trash;

    dma->PDC_PTCR = (AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);
    dma->PDC_RPR = (unsigned int) (us->in_buf[0]);
    dma->PDC_RCR = us->in_size;
    dma->PDC_RNPR = (unsigned int) (us->in_buf[1]);
    dma->PDC_RNCR = us->in_size;
    trash = dev->US_RHR;
    trash = dev->US_CSR;
    dma->PDC_PTCR = AT91C_PDC_RXTEN | AT91C_PDC_TXTEN;
    dev->US_CR = AT91C_US_RXEN | AT91C_US_TXEN;
}

/**
 * Disable the usart device.
 */
void usart_disable(usart *us)
{
    AT91S_PDC *dma = us->dma;
    AT91S_USART *dev = us->dev;

    dma->PDC_PTCR = (AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);
    dev->US_CR = AT91C_US_RXDIS | AT91C_US_TXDIS;
    dma->PDC_RPR = 0;
    dma->PDC_RCR = 0;
    dma->PDC_RNPR = 0;
    dma->PDC_RNCR = 0;
}

/**
 * Release the system resources associated with the specified usart device.
 */
void usart_free(usart *us)
{
    usart_disable(us);
    system_free((byte *) us);
}

/**
 * Return the current read/write state of the usart.
 */
uint32 usart_status(usart *us)
{
    /* Return the state of any pending i/o requests 
       one bit for input one bit for output. */
    AT91S_PDC *dma = us->dma;
    int ret = 0;
    int bytes_ready;

    /* First check for any input. */
    bytes_ready = us->in_size - dma->PDC_RCR;
    if (dma->PDC_RNCR == 0)
        bytes_ready = us->in_size * 2 - dma->PDC_RCR;
    if (bytes_ready > us->in_offset)
        ret |= US_READABLE;

    /* Now check for the write state of the device. */
    if (dma->PDC_TNCR == 0)
    {
        ret |= US_WRITEABLE;
        if (dma->PDC_TCR == 0)
            ret |= US_WRITEEMPTY;
    }
    return ret;
}

/**
 * write bytes to the device. Return the number of bytes actually written.
 */
uint32 usart_write(usart *us, uint8 *buf, uint32 off, uint32 len)
{
    AT91S_PDC *dma = us->dma;

    if (dma->PDC_TNCR == 0)
    {
        if (len > us->out_size)
            len = us->out_size;
        memcpy(us->out_buf[us->out_base], buf + off, len);
        dma->PDC_TNPR = (unsigned int) us->out_buf[us->out_base];
        dma->PDC_TNCR = len;
        us->out_base = (us->out_base + 1) % BUF_CNT;
        return len;
    }
    else
        return 0;
}

/**
 * Read bytes from the device. Return the number of bytes read into the
 * buffer or -1 if the a buffer overflow has occurred.
 */
uint32 usart_read(usart *us, uint8 * buf, uint32 off, uint32 len)
{
    AT91S_PDC *dma = us->dma;
    int read_len;
    int in_sz = us->in_size;
    int len2 = 0;
    int len1 = in_sz - dma->PDC_RCR;
    int idx = us->in_offset;
    int i = 0;
    int cnt = 0;

    if (dma->PDC_RNCR == 0)
    {
        len1 = in_sz;
        len2 = in_sz - dma->PDC_RCR;
    }

    if (len1 > idx)
    {
        cnt = len1 - idx;
        uint8 *buf_ptr = us->in_buf[us->in_base];
        if (cnt > len)
            cnt = len;
        for (; i < cnt; i++)
            buf[off + i] = buf_ptr[idx++];
    }
    if (len2 > 0)
    {
        uint8 *buf_ptr = us->in_buf[(us->in_base + 1) % BUF_CNT];
        idx -= in_sz;
        cnt += len2 - idx;
        if (cnt > len)
            cnt = len;
        for (; i < cnt; i++)
            buf[off + i] = buf_ptr[idx++];
        idx += in_sz;
    }
    read_len = i;

    /* Recycle now processed buffers. */
    if (idx >= in_sz)
    {
        // do we have a buffer overrun?
        if (us->dev->US_CSR & AT91C_US_OVRE)
        {
            /* Do we have emptied the good data? */
            if (idx >= in_sz * BUF_CNT && read_len <= 0)
            {
                /* Yes, so reset things and return an error. */
                us->dev->US_CR |= AT91C_US_RSTSTA;
                read_len = -1;
            }
            else
            {
                us->in_offset = idx;
                return read_len;
            }
        }
        /* Switch current buffer, and set up next. */
        while (idx >= in_sz && dma->PDC_RNCR == 0)
        {
            dma->PDC_RNPR = (unsigned int) us->in_buf[us->in_base];
            us->in_base = (us->in_base + 1) % BUF_CNT;
            dma->PDC_RNCR = in_sz;
            idx -= in_sz;
        }
    }
    us->in_offset = idx;
    return read_len;
}

/**
 * Return a pointer to the current write buffer. It is up to the calling code
 * to ensure that this buffer is large enough for any write operations.
 */
uint8 * usart_get_write_buffer(usart *us)
{
    if (us->dma->PDC_TNCR != 0)
        return NULL;
    return us->out_buf[us->out_base];
}

/**
 * Write len bytes from the current write write buffer.
 */
void usart_write_buffer(usart *us, uint32 len)
{
    AT91S_PDC *dma = us->dma;
    dma->PDC_TNPR = (unsigned int) us->out_buf[us->out_base];
    dma->PDC_TNCR = len;
    us->out_base = (us->out_base + 1) % BUF_CNT;
}
