#ifndef __BFIFO_H__
#define __BFIFO_H__

#include "include/platform/systypes.h"

/* The byte FIFO control structure. */
struct byte_fifo
{
    /* The buffer containing the FIFO. */
    uint8 *buffer;
    /* Pointer to the end of the buffer. */
    uint8 *buffer_end;
    /* Size in bytes of the buffer. */
    uint32 buffer_size;
    /* Current head (freshest data) of the FIFO in the buffer. */
    uint8 *head;
    /* Current tail (oldest data) of the FIFO in the buffer. */
    uint8 *tail;
    /* Number of bytes currently enqueued in the FIFO. */
    uint32 holding;
};

void byte_fifo_clear(struct byte_fifo *f);
void byte_fifo_init(struct byte_fifo *f, uint8 *buffer, uint32 buffer_size);
int byte_fifo_put(struct byte_fifo *f, uint32 force, uint8 b);
int byte_fifo_get(struct byte_fifo *f, uint8 *b);

#endif
