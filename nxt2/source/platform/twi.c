/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Driver for the I2C/TWI interface
*
* Provide read/write to an I2C/TWI device (in this case the ATMega
* co-processor). Uses the hardware TWI device in interrupt mode.
* NOTES
* This code does not support single byte read/write operation.
* This code does not support internal register addressing.
* Runs at high priority interrupt to minimize chance of early
* write termination (have never seen this but...).
* For read operations we do not wait for the complete event before
* marking the read as over. We do this because the time window for
* a read when talking to the ATMega is very tight, so finishing
* slightly early avoids a data over-run. It is a little iffy though!
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "platform/twi.h"

#include "platform/irqs.h"
#include "platform/aic.h"
#include "platform/systick.h"

#include "platform/at91/at91sam7.h"

// Required clock divisor
#define I2CCLK 400000L
#define CLDIV  (((CLOCK_FREQUENCY/I2CCLK)/2)-3)
// Pins
#define TWCK (1 << 4)
#define TWD (1 << 3)

static enum
{
    TWI_UNINITIALISED = 0,
    TWI_FAILED,
    TWI_IDLE,
    TWI_DONE,
    TWI_RX_BUSY,
    TWI_TX_BUSY,
} twi_state;

static uint32 twi_pending;
static uint8 *twi_ptr;
static uint32 twi_mask;

// Accumulate states
#ifdef USE_STATS
static struct
{
    uint32 rx_done;
    uint32 tx_done;
    uint32 bytes_tx;
    uint32 bytes_rx;
    uint32 unre;
    uint32 ovre;
    uint32 nack;
} twi_stats;
#define STATS(code) code;
#else
#define STATS(code)
#endif

/**
 * Return the status of the twi device.
 * 0 == Ready for use
 * 1 == Busy
 * -1 == Error or closed
 */
int twi_status(void)
{
    return (twi_state > TWI_DONE ? 1 : (twi_state < TWI_IDLE ? -1 : 0));
}

/**
 * Process TWI interrupts.
 * Assumes that only valid interrupts will be enabled and that twi_mask
 * will have been set to only contain the valid bits for the current
 * I/O state. This means that we do not have to test this state at
 * interrupt time.
 */
static void twi_isr_handler(void)
{
    uint32 status = *AT91C_TWI_SR & twi_mask;
    if (status & AT91C_TWI_RXRDY)
    {
        STATS(twi_stats.bytes_rx++)
        *twi_ptr++ = *AT91C_TWI_RHR;
        twi_pending--;
        if (twi_pending == 1)
        {
            /* Second last byte -- issue a stop on the next byte. */
            *AT91C_TWI_CR = AT91C_TWI_STOP;
        }
        if (!twi_pending)
        {
            /* All bytes have been sent. Mark operation as complete. */
            STATS(twi_stats.rx_done++)
            twi_state = TWI_DONE;
            *AT91C_TWI_IDR = AT91C_TWI_RXRDY;
        }
    }
    else if (status & AT91C_TWI_TXRDY)
    {
        if (twi_pending)
        {
            /* Still stuff to send. */
            *AT91C_TWI_THR = *twi_ptr++;
            twi_pending--;
            STATS(twi_stats.bytes_tx++)
        }
        else
        {
            /* Everything has been sent, now wait for complete. */
            STATS(twi_stats.tx_done++);
            *AT91C_TWI_IDR = AT91C_TWI_TXRDY;
            *AT91C_TWI_IER = AT91C_TWI_TXCOMP;
            twi_mask = AT91C_TWI_TXCOMP | AT91C_TWI_NACK;
        }
    }
    else if (status & AT91C_TWI_TXCOMP)
    {
        twi_state = TWI_DONE;
        *AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
    }

    if (status & AT91C_TWI_NACK)
    {
        STATS(twi_stats.nack++)
        *AT91C_TWI_IDR = ~0;
        twi_state = TWI_UNINITIALISED;
    }
}

/**
 * Force a device reset. 
 */
void twi_reset(void)
{
    uint32 clocks = 9;

    *AT91C_TWI_IDR = ~0;

    *AT91C_PMC_PCER = (1 << AT91C_ID_PIOA) | /* Need PIO too */
    (1 << AT91C_ID_TWI); /* TWI clock domain */

    /* Set up pin as an IO pin for clocking till clean. */
    *AT91C_PIOA_MDER = TWD | TWCK;
    *AT91C_PIOA_PER = TWD | TWCK;
    *AT91C_PIOA_ODR = TWD;
    *AT91C_PIOA_OER = TWCK;

    while (clocks > 0 && !(*AT91C_PIOA_PDSR & TWD))
    {
        *AT91C_PIOA_CODR = TWCK;
        systick_wait_ns(1500);
        *AT91C_PIOA_SODR = TWCK;
        systick_wait_ns(1500);
        clocks--;
    }

    *AT91C_PIOA_PDR = TWD | TWCK;
    *AT91C_PIOA_ASR = TWD | TWCK;

    *AT91C_TWI_CR = AT91C_TWI_SWRST | AT91C_TWI_MSDIS;/* Disable & reset */

    *AT91C_TWI_CWGR = ((CLDIV << 8) | CLDIV); /* Set for 400kHz */
    *AT91C_TWI_CR = AT91C_TWI_MSEN; /* Enable as master */
    *AT91C_TWI_IER = AT91C_TWI_NACK;
    twi_mask = 0;
}

/**
 * Initialize the device.
 */
int twi_init(void)
{
    int i_state;

    i_state = irqs_get_and_disable();

    *AT91C_TWI_IDR = ~0; /* Disable all interrupt sources */
    aic_mask_off(AT91C_ID_TWI);
    aic_set_vector(AT91C_ID_TWI, AIC_INT_LEVEL_ABOVE_NORMAL, (int) twi_isr_handler);
    aic_mask_on(AT91C_ID_TWI);

    twi_reset();

    /* Init peripheral */
    twi_state = TWI_IDLE;

    if (i_state)
        irqs_enable();

    return 1;
}

/**
 * Start a read operation to the device. The operation will complete
 * asynchronously and can be monitored using twi_status. Note that we
 * do not support single byte reads, or internal register addresses.
 */
void twi_start_read(uint32 dev_addr, uint8 *data, uint32 nBytes)
{
    if (twi_state < TWI_RX_BUSY)
    {
        twi_state = TWI_RX_BUSY;
        twi_ptr = data;
        twi_pending = nBytes;
        *AT91C_TWI_MMR = AT91C_TWI_IADRSZ_NO | AT91C_TWI_MREAD
                | ((dev_addr & 0x7f) << 16);
        twi_mask = AT91C_TWI_RXRDY | AT91C_TWI_NACK;
        *AT91C_TWI_CR = AT91C_TWI_START;
        *AT91C_TWI_IER = AT91C_TWI_RXRDY;
    }
}

/**
 * Start a write operation to the device. The operation will complete
 * asynchronously and can be monitored using twi_status. Note that we
 * do not support single byte reads, or internal register addresses.
 */
void twi_start_write(uint32 dev_addr, const uint8 *data, uint32 nBytes)
{
    if (twi_state < TWI_RX_BUSY)
    {
        twi_state = TWI_TX_BUSY;
        twi_ptr = (uint8 *) data;
        twi_pending = nBytes;

        *AT91C_TWI_MMR = AT91C_TWI_IADRSZ_NO | ((dev_addr & 0x7f) << 16);
        *AT91C_TWI_THR = *twi_ptr++;
        twi_pending--;
        STATS(twi_stats.bytes_tx++)
        twi_mask = AT91C_TWI_TXRDY | AT91C_TWI_NACK;
        *AT91C_TWI_IER = AT91C_TWI_TXRDY;
    }
}
