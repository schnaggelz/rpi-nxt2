/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Generic methods to support I/O to an AT91 usart. The actual detailed
* hardware setup must be performed by a hardware specific set of
* functions before calling the enable methods provided here.
*
* License notes see LICENSE.txt
*******************************************************************************/

/* Bit values used for events */
#define US_READABLE 0x1
#define US_WRITEABLE 0x2
#define US_WRITEEMPTY 0x4

#define BUF_CNT 2
typedef struct
{
    AT91S_PDC* dma;
    AT91S_USART* dev;
    uint8* in_buf[BUF_CNT];
    uint8* out_buf[BUF_CNT];
    int in_offset;
    int in_size;
    int out_size;
    uint8 in_base;
    uint8 out_base;
} usart;

void usart_enable(usart* us);
void usart_disable(usart* us);
void usart_free(usart* us);
void usart_write_buffer(usart* us, uint32 len);

uint32 usart_status(usart* us);
uint32 usart_write(usart* us, uint8* buf, uint32 off, uint32 len);
uint32 usart_read(usart* us, uint8* buf, uint32 off, uint32 length);
uint8* usart_get_write_buffer(usart* us);
usart* usart_allocate(AT91S_USART* dev, AT91S_PDC* dma, int32 in_size,
                      int32 out_size);
