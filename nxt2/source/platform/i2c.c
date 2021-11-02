#include "platform/i2c.h"

#include "platform/aic.h"
#include "platform/irqs.h"
#include "platform/ports.h"
#include "platform/systick.h"

#include "platform/at91/at91sam7s256.h"

#include <string.h>

#define I2C_CLOCK 9600
#define I2C_HS_CLOCK 125000
#define I2C_MAX_PTS 3
#define I2C_N_PORTS 4
#define I2C_CLOCK 9600

struct i2c_pin_pair
{
    uint32 scl;
    uint32 sda;
};

static const struct i2c_pin_pair i2c_pin[4] = {
    {1 << 23, 1 << 18},
    {1 << 28, 1 << 19},
    {1 << 29, 1 << 20},
    {1 << 30, 1 << 2}
};

struct i2c_pt
{
    uint8 start : 1;
    uint8 restart : 1;
    uint8 stop : 1;
    uint8 tx : 1;
    uint8 last_pt : 1; /* Last pt in transaction */
    uint16 nbytes;     /* N bytes to transfer */
    uint8* data;       /* Data buffer */
};

typedef enum
{
    I2C_DISABLED = 0,
    I2C_IDLE,
    I2C_BEGIN,
    I2C_RESTART1,
    I2C_START1,
    I2C_START2,
    I2C_START3,
    I2C_START_RECLOCK1,
    I2C_LOW0,
    I2C_LOW1,
    I2C_HIGH0,
    I2C_HIGH1,
    I2C_STOP0,
    I2C_STOP1,
    I2C_STOP2,
    I2C_STOP3,
} i2c_port_state;

typedef struct
{
    uint32 scl_pin;
    uint32 sda_pin;
    uint8 addr_int[2]; /* Device address with internal address */
    uint8 addr;        /* Just device address */

    struct i2c_pt pt[I2C_MAX_PTS];
    struct i2c_pt* current_pt;

    i2c_port_state state;

    uint8* data;
    uint32 nbits;

    uint8 pt_num;
    uint8 pt_begun : 1;
    uint8 fault : 1;
    uint8 transmitting : 1;
    uint8 ack_slot : 1;
    uint8 ack_slot_pending : 1;
    uint8 port_bit;
} i2c_port;

static i2c_port i2c_ports[I2C_N_PORTS] = {{0}};
static i2c_port* i2c_port_ptrs[I2C_N_PORTS] = {0};

static uint32 i2c_port_busy = 0;

/* The I2C state machines are pumped by a timer interrupt running at 4x
   the bit speed. Move the port state from one step to another toggling
   the clock line on alternate calls. */
void i2c_timer_isr_handler(void)
{
    i2c_port* p = NULL;

    uint32 codr = 0;
    uint32 sodr = 0;
    uint32 oer = 0;
    uint32 odr = 0;
    uint32 inputs = *AT91C_PIOA_PDSR;
    uint32 dummy = *AT91C_TC0_SR;
    uint8 idx = 0;

    for (idx = 0; idx < I2C_N_PORTS; idx++)
    {
        p = i2c_port_ptrs[idx];
        if (p == NULL)
            continue; /* Port uninitialized. */

        switch (p->state)
        {
        default:
        case I2C_DISABLED:
            /* Port uninitialized. */
            break;
        case I2C_IDLE:
            /* Not in a transaction. */
            break;
        case I2C_BEGIN:
        {
            /* Start the current partial transaction. */
            p->pt_begun |= (1 << p->pt_num);
            oer |= p->sda_pin;
            oer |= p->scl_pin;

            if (p->current_pt && p->current_pt->nbytes)
            {
                p->data = p->current_pt->data;
                p->nbits = p->current_pt->nbytes * 8;
                p->transmitting = p->current_pt->tx;
                p->ack_slot = 0;
                p->ack_slot_pending = 0;
                p->fault = 0;

                if (!p->transmitting)
                    *(p->data) = 0;

                if (p->current_pt->restart)
                {
                    /* Make sure both SDA and SCL are high. */
                    sodr |= p->scl_pin;
                    sodr |= p->sda_pin;
                    p->state = I2C_RESTART1;
                }
                else if (p->current_pt->start)
                {
                    sodr |= p->sda_pin;
                    p->state = I2C_START1;
                }
                else
                {
                    codr |= p->scl_pin;
                    p->state = I2C_LOW0;
                }
            }
            else
            {
                p->state = I2C_IDLE;
            }
            break;
        }
        case I2C_RESTART1:
        {
            /* SDA high, take SCL low.*/
            codr |= p->scl_pin;
            p->state = I2C_START1;
            break;
        }
        case I2C_START1:
        {
            /* SDA high, take SCL high. */
            sodr |= p->scl_pin;
            p->state = I2C_START2;
            break;
        }
        case I2C_START2:
        {
            if (inputs & p->sda_pin)
            {
                /* Take SDA low while SCL is high. */
                codr |= p->sda_pin;
                p->state = I2C_START3;
            }
            else
            {
                /* SDA was not high, so do a clock. */
                codr |= p->scl_pin;
                p->state = I2C_START_RECLOCK1;
            }
            break;
        }
        case I2C_START_RECLOCK1:
        {
            codr |= p->scl_pin;
            p->state = I2C_START1;
            break;
        }
        case I2C_START3:
        {
            /* Take SCL low. */
            codr |= p->scl_pin;
            p->state = I2C_LOW0;
            break;
        }
        case I2C_LOW0:
        {
            /* SCL is low. */
            if (p->ack_slot_pending)
            {
                p->ack_slot = 1;
                p->ack_slot_pending = 0;
            }
            else
            {
                p->ack_slot = 0;
            }
            if (p->nbits || p->ack_slot)
            {
                if (p->ack_slot)
                {
                    if (p->transmitting)
                    {
                        odr |= p->sda_pin;
                    }
                    else
                    {
                        oer |= p->sda_pin;
                        codr |= p->sda_pin;
                    }
                }
                else if (!p->transmitting)
                {
                    odr |= p->sda_pin;
                }
                else
                {
                    /* Transmitting, and not an ack slot,
                     * so send next bit. */
                    oer |= p->sda_pin;
                    p->nbits--;
                    if (((*(p->data)) >> (p->nbits & 7)) & 0x01)
                    {
                        sodr |= p->sda_pin;
                    }
                    else
                    {
                        codr |= p->sda_pin;
                    }
                    if ((p->nbits & 7) == 0)
                    {
                        p->data++;
                        if (p->nbits || p->transmitting)
                        {
                            p->ack_slot_pending = 1;
                        }
                    }
                }
                p->state = I2C_LOW1;
            }
            else if (p->current_pt->stop)
            {
                p->state = I2C_STOP0;
            }
            else
            {
                p->current_pt++;
                p->pt_num++;
                sodr |= p->sda_pin;
                p->state = I2C_BEGIN;
            }
            break;
        }
        case I2C_LOW1:
        {
            /* Take SCL high. */
            sodr |= p->scl_pin;
            p->state = I2C_HIGH0;
            break;
        }
        case I2C_HIGH0:
        {
            /* Wait for high pulse width. If someone else
             * is not holding the pin down, then advance. */
            if (inputs & p->scl_pin)
            {
                p->state = I2C_HIGH1;
            }
            break;
        }
        case I2C_HIGH1:
        {
            if (p->transmitting && p->ack_slot)
            {
                /* Expect ack from slave. */
                if (inputs & p->sda_pin)
                {
                    p->fault = 1;
                    codr |= p->scl_pin;
                    p->state = I2C_STOP0;
                }
                else
                {
                    codr |= p->scl_pin;
                    p->state = I2C_LOW0;
                }
            }
            else
            {
                /* Read pin if needed, then take SCL low. */
                if (!p->transmitting && !p->ack_slot)
                {
                    /* Receive a bit. */
                    uint8* d = p->data;
                    p->nbits--;
                    if (inputs & p->sda_pin)
                    {
                        *d |= (1 << (p->nbits & 7));
                    }
                    if (p->nbits && ((p->nbits & 7) == 0))
                    {
                        p->data++;
                        d = p->data;
                        p->ack_slot_pending = 1;
                        *d = 0;
                    }
                }
                codr |= p->scl_pin;
                p->state = I2C_LOW0;
            }
            break;
        }
        case I2C_STOP0:
        {
            /* Take SDA low (SCL is already low). */
            oer |= p->sda_pin;
            codr |= p->sda_pin;
            p->state = I2C_STOP1;
            break;
        }
        case I2C_STOP1:
        {
            /* Take SCL high. */
            sodr |= p->scl_pin;
            p->state = I2C_STOP2;
            break;
        }
        case I2C_STOP2:
        {
            /* Take SDA pin high. */
            sodr |= p->sda_pin;
            p->state = I2C_STOP3;
            break;
        }
        case I2C_STOP3:
        {
            if (p->current_pt->last_pt)
            {
                if (i2c_port_busy & p->port_bit)
                {
                    i2c_port_busy &= ~p->port_bit;
                }
                else
                {
                    /* All done */
                    p->state = I2C_IDLE;
                }
            }
            else
            {
                p->current_pt++;
                p->pt_num++;
                p->state = I2C_BEGIN;
            }
            break;
        }
        }
    }

    if (codr)
    {
        *AT91C_PIOA_CODR = codr;
    }
    if (sodr)
    {
        *AT91C_PIOA_SODR = sodr;
    }
    if (oer)
    {
        *AT91C_PIOA_OER = oer;
    }
    if (odr)
    {
        *AT91C_PIOA_ODR = odr;
    }
}

/* Disables an I2C port. */
void i2c_disable(uint8 port)
{
    i2c_port* p;

    if (port < I2C_N_PORTS && i2c_port_ptrs[port])
    {
        p = i2c_port_ptrs[port];

        uint32 pinmask = p->scl_pin | p->sda_pin;

        *AT91C_PIOA_ODR = pinmask;

        i2c_port_busy &= ~(1 << port);
        i2c_port_ptrs[port] = NULL;

        sp_reset(port);
    }
}

/* Disables all I2C ports. */
void i2c_disable_all()
{
    int i;
    for (i = 0; i < I2C_N_PORTS; i++)
        i2c_disable(i);
}

/* Enable an I2C port. */
sint32 i2c_enable(uint8 port, uint8 mode)
{
    uint32 pinmask;
    i2c_port* p;

    if (port >= 0 && port < I2C_N_PORTS)
    {
        p = i2c_port_ptrs[port];
        if (!p)
        {
            p = (i2c_port*)&i2c_ports[port];
            i2c_port_ptrs[port] = p;
        }
        pinmask = p->scl_pin | p->sda_pin;
        p->state = I2C_IDLE;
        /* Set clock pin for output, open collector driver, with
         * pull-ups enabled. Set data to be enabled for output
         * with pull-ups disabled. */
        *AT91C_PIOA_SODR = pinmask;
        *AT91C_PIOA_OER = pinmask;
        *AT91C_PIOA_MDER = p->scl_pin;
        *AT91C_PIOA_PPUDR = p->sda_pin;
        *AT91C_PIOA_PPUER = p->scl_pin;

        p->port_bit = 1 << port;

        return 1;
    }
    return -1;
}

/* Initializes the module. */
void i2c_init(void)
{
    int i;
    int i_state;
    uint32 dummy;
    i2c_port* p;

    for (i = 0; i < I2C_N_PORTS; i++)
    {
        i2c_port_ptrs[i] = NULL;

        p = &i2c_ports[i];
        p->state = I2C_IDLE;
        p->scl_pin = i2c_pin[i].scl;
        p->sda_pin = i2c_pin[i].sda;
        i2c_disable(i);
    }

    i_state = irqs_get_and_disable();

    /* Set up Timer Counter 0 to drive standard speed I2C. */
    *AT91C_PMC_PCER = (1 << AT91C_ID_TC0); /* Power enable */

    *AT91C_TC0_CCR = AT91C_TC_CLKDIS; /* Disable */
    *AT91C_TC0_IDR = ~0;
    dummy = *AT91C_TC0_SR;
    *AT91C_TC0_CMR = AT91C_TC_CLKS_TIMER_DIV1_CLOCK |
                     AT91C_TC_CPCTRG; /* MCLK/2, RC compare trigger */
    *AT91C_TC0_RC = ((CLOCK_FREQUENCY / 2) / (4 * I2C_CLOCK)) / 1;
    *AT91C_TC0_IER = 0x10; /* Enable RC trigger interrupt */
    *AT91C_TC0_CCR = 0x1;  /* Enable */

    aic_mask_off(AT91C_ID_TC0);
    aic_set_vector(AT91C_ID_TC0, AIC_INT_LEVEL_NORMAL,
                   (int)i2c_timer_isr_handler);
    aic_mask_on(AT91C_ID_TC0);

    *AT91C_TC0_CCR = 0x04; /* Software trigger */

    if (i_state)
        irqs_enable();
}

/* Is the port busy? */
sint32 i2c_status(uint8 port)
{
    i2c_port* p;
    sint8 state = 0;

    if (port < 0 || port >= I2C_N_PORTS)
        return I2C_ERR_INVALID_PORT;

    if ((i2c_port_busy & (1 << port)) != 0)
        return I2C_ERR_BUSY;

    p = i2c_port_ptrs[port];

    if (p == NULL)
        state = I2C_ERR_NO_INIT;

    else if (p->state != I2C_IDLE)
        state = I2C_ERR_BUSY;

    return state;
}

/* Start a transaction. */
sint32 i2c_start(uint8 port, uint8 ext_addr, uint8 int_addr,
                 uint8* data, uint32 nbytes, uint8 write)
{
    i2c_port* p;
    struct i2c_pt* pt;
    sint32 status = i2c_status(port);
    if (status < 0)
        return status;
    p = i2c_port_ptrs[port];
    p->pt_num = 0;
    p->pt_begun = 0;
    pt = p->pt;
    p->current_pt = pt;

    memset(pt, 0, sizeof(p->pt));

    /* Set up command to write the internal address to the device. */
    p->addr_int[0] = (ext_addr << 1); /* this is a write */
    p->addr_int[1] = int_addr;

    /* Set up first partial transaction:
       Start address and internal address if required. */
    pt->start = 1;
    /* Extra stop for the odd Lego I2C sensor, but only on a read. */
    pt->stop = (write ? 0 : 1);
    pt->tx = 1;
    pt->data = p->addr_int;
    pt->nbytes = 2;

    pt++;

    if (!write)
    {
        /* Set up second partial transaction:
         * Restart and address. */
        pt->start = 0;
        pt->restart = !pt->start;
        pt->stop = 0;
        pt->tx = 1;
        p->addr = (ext_addr << 1) | (write ? 0 : 1);
        pt->data = &p->addr;
        pt->nbytes = 1;

        pt++;
    }

    /* Set up third partial transaction:
     * Data and stop. */
    pt->start = 0;
    pt->stop = 1;
    pt->tx = (write ? 1 : 0);
    pt->data = data;
    pt->nbytes = nbytes;
    pt->last_pt = 1;

    i2c_port_busy |= 1 << port;

    /* Start the transaction. */
    p->state = I2C_BEGIN;

    return 0;
}

uint8 i2c_wait_ready(uint8 port, uint32 wait_time_ms)
{
    volatile uint32 exit_time_ms;

    exit_time_ms = systick_get_ms() + wait_time_ms;
    while(systick_get_ms() <= exit_time_ms)
    {
        if (i2c_status(port) == 0) return 1;
    }
    return 0; /* timed out */
}
