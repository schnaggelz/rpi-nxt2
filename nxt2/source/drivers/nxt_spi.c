#include "../../include/drivers/nxt_spi.h"

#include "at91sam7.h"
#include "irqs.h"
#include "aic.h"

const uint8 *display = (uint8 *) 0;
volatile uint8 dirty = 0;
volatile uint8 page = 0;
volatile const uint8 *data = (uint8 *) 0;
uint8 mode = 0xff;

static void spi_set_mode(uint8 m)
{
    if (m == mode)
        return;

    /* Wait until all bytes have been sent. */
    uint32 status;

    
    if (m == mode)
    {
        /* Nothing to do if we are already in the correct mode. */
        return;
    }
    do
    {
        status = *AT91C_SPI_SR;
    } 
    while (!(status & 0x200));

    /* Set command or data mode. */
    if (m)
        *AT91C_PIOA_SODR = CD_PIN;
    else
        *AT91C_PIOA_CODR = CD_PIN;

    /* Remember the current mode. */
    mode = m;
}

void spi_isr_handler(void) 
{
    if (page == 0)
    {
        /* Check to see if we have data to display. */
        if (dirty != 0)
        {
            data = display;
            dirty = 0;
        }
        else
        {
            /* No so turn things off. It will get re-set
               if we ever have anything to display. */
            *AT91C_SPI_IDR = AT91C_SPI_ENDTX;
            return;
        }
    }

    /* Make sure we are in data mode. */
    spi_set_mode(1);

    /* Now do the transfer. We make use of the auto-wrap function so simply
     * need to send 8*132 bytes to get back to where we started. However the
     * display buffer is structured as series of 100 byte lines, so we need to
     * get tricky. I've made the display one line longer (9 lines) and so when we
     * send the data we send 100 bytes from the actual line plus 32 padding bytes
     * (that are not actually seen), from the next line. The extra line means
     * that this is safe to do. If we can redefine the display as a 8*132 then
     * we could just use a single DMA transfer (instead of 8, 132 byte ones).
     * However I'm not sure if this would be safe.
     */
    *AT91C_SPI_TNPR = (uint32) data;
    *AT91C_SPI_TNCR = 132;
    page = (page + 1) % 8;
    data += 100;
}

void nxt_spi_init(void)
{
    int i_state = irqs_get_and_disable();

    *AT91C_PMC_PCER = (1L << AT91C_ID_SPI); /* Enable MCK clock */

    *AT91C_PIOA_PER = AT91C_PIO_PA12;       /* EnableA0onPA12 */
    *AT91C_PIOA_OER = AT91C_PIO_PA12;
    *AT91C_PIOA_CODR = AT91C_PIO_PA12;
    *AT91C_PIOA_PDR = AT91C_PA14_SPCK;      /* EnableSPCKonPA14 */
    *AT91C_PIOA_ASR = AT91C_PA14_SPCK;
    *AT91C_PIOA_ODR = AT91C_PA14_SPCK;
    *AT91C_PIOA_OWER = AT91C_PA14_SPCK;
    *AT91C_PIOA_MDDR = AT91C_PA14_SPCK;
    *AT91C_PIOA_PPUDR = AT91C_PA14_SPCK;
    *AT91C_PIOA_IFDR = AT91C_PA14_SPCK;
    *AT91C_PIOA_CODR = AT91C_PA14_SPCK;
    *AT91C_PIOA_IDR = AT91C_PA14_SPCK;
    *AT91C_PIOA_PDR = AT91C_PA13_MOSI;      /* EnablemosionPA13 */
    *AT91C_PIOA_ASR = AT91C_PA13_MOSI;
    *AT91C_PIOA_ODR = AT91C_PA13_MOSI;
    *AT91C_PIOA_OWER = AT91C_PA13_MOSI;
    *AT91C_PIOA_MDDR = AT91C_PA13_MOSI;
    *AT91C_PIOA_PPUDR = AT91C_PA13_MOSI;
    *AT91C_PIOA_IFDR = AT91C_PA13_MOSI;
    *AT91C_PIOA_CODR = AT91C_PA13_MOSI;
    *AT91C_PIOA_IDR = AT91C_PA13_MOSI;
    *AT91C_PIOA_PDR = AT91C_PA10_NPCS2;     /* Enablenpcs0onPA10 */
    *AT91C_PIOA_BSR = AT91C_PA10_NPCS2;
    *AT91C_PIOA_ODR = AT91C_PA10_NPCS2;
    *AT91C_PIOA_OWER = AT91C_PA10_NPCS2;
    *AT91C_PIOA_MDDR = AT91C_PA10_NPCS2;
    *AT91C_PIOA_PPUDR = AT91C_PA10_NPCS2;
    *AT91C_PIOA_IFDR = AT91C_PA10_NPCS2;
    *AT91C_PIOA_CODR = AT91C_PA10_NPCS2;
    *AT91C_PIOA_IDR = AT91C_PA10_NPCS2;
    *AT91C_SPI_CR = AT91C_SPI_SWRST;        /* Soft reset */
    *AT91C_SPI_CR = AT91C_SPI_SPIEN;        /* Enable SPI */
    *AT91C_SPI_MR = AT91C_SPI_MSTR | AT91C_SPI_MODFDIS | (0xB << 16);
    AT91C_SPI_CSR[2] = ((OSC / SPI_BITRATE) << 8) | AT91C_SPI_CPOL;

    /* Set mode to unknown. */
    mode = 0xff;

    /* Set up safe DMA refresh state. */
    data = display = (uint8 *)0;
    dirty = 0;
    page = 0;

    /* Install the interrupt handler. */
    aic_mask_off(AT91C_ID_SPI);
    aic_set_vector(AT91C_ID_SPI, AIC_INT_LEVEL_NORMAL, (uint32)spi_isr_handler);
    aic_mask_on(AT91C_ID_SPI);
    *AT91C_SPI_PTCR = AT91C_PDC_TXTEN;

    if (i_state)
        irqs_enable();
}

void nxt_spi_write(uint32 CD, const uint8 *data, uint32 n_bytes)
{
    uint32 status;
    uint32 cd_mask = (CD ? 0x100 : 0);

    spi_set_mode(CD);
    while (n_bytes)
    {
        *AT91C_SPI_TDR = (*data | cd_mask);
        data++;
        n_bytes--;
        /* Wait until byte sent. */
        do
        {
            status = *AT91C_SPI_SR;
        } while (!(status & 0x200));
    }
}

void nxt_spi_set_display(const uint8 *disp)
{
    /* Set the display buffer to be used for DMA refresh.
     * It is really only safe to set the display once. 
     * Should probably sort this out so that it is set 
     * separately from requesting a refresh.
     */
    if (!display)
        display = disp;
}

void nxt_spi_refresh(void)
{
    /* Request the start of a DMA refresh of the display.
     * If the display is not set nothing to do. */
    if (!display)
        return;

    /* Say we have changes. */
    dirty = 1;

    /* Start the DMA refresh. */
    *AT91C_SPI_IER = AT91C_SPI_ENDTX;
}
