#include "include/utils/byte_fifo.h"

void byte_fifo_clear(struct byte_fifo *f)
{
    f->tail = f->head = f->buffer;
    f->holding = 0;
}

void byte_fifo_init(struct byte_fifo *f, uint8 *buffer, uint32 buffer_size)
{
    f->buffer = buffer;
    f->buffer_size = buffer_size;
    f->buffer_end = buffer + buffer_size;
    byte_fifo_clear(f);
}

int byte_fifo_put(struct byte_fifo *f, uint32 force, uint8 b)
{
    /* If the FIFO is full, only proceed if the operation was
       forced. Otherwise, fail now. */
    if (f->holding >= f->buffer_size && !force)
        return 0;

    /* Enqueue the byte and advance the FIFO head. */
    *(f->head) = b;
    f->holding++;
    f->head++;
    if (f->head >= f->buffer_end)
        f->head = f->buffer;

    /* If the operation was forced and overwrote old data, the tail
       needs to be moved as well. */
    if (f->tail == f->head)
    {
        f->holding--;
        f->tail++;
        if (f->tail >= f->buffer_end)
            f->tail = f->buffer;
    }

    return 1;
}

int byte_fifo_get(struct byte_fifo *f, uint8 *b)
{
    /* Fail if the FIFO is empty. */
    if (!f->holding)
        return 0;

    /* Dequeue a byte and advance the tail. */
    *b = *(f->tail);
    f->holding--;
    f->tail++;
    if (f->tail >= f->buffer_end)
        f->tail = f->buffer;

    return 1;
}
