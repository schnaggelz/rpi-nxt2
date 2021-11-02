#include "platform/udp.h"
#include "platform/aic.h"
#include "platform/irqs.h"
#include "platform/systick.h"

#include "platform/at91/at91sam7.h"

#include <string.h>

#define MAX_BUF   64
#define EP_OUT    1
#define EP_IN     2

#define AT91C_PERIPHERAL_ID_UDP        11

#define AT91C_UDP_CSR0  ((AT91_REG *)   0xFFFB0030)
#define AT91C_UDP_CSR1  ((AT91_REG *)   0xFFFB0034)
#define AT91C_UDP_CSR2  ((AT91_REG *)   0xFFFB0038)
#define AT91C_UDP_CSR3  ((AT91_REG *)   0xFFFB003C)

#define AT91C_UDP_FDR0  ((AT91_REG *)   0xFFFB0050)
#define AT91C_UDP_FDR1  ((AT91_REG *)   0xFFFB0054)
#define AT91C_UDP_FDR2  ((AT91_REG *)   0xFFFB0058)
#define AT91C_UDP_FDR3  ((AT91_REG *)   0xFFFB005C)

/* The following functions are used to set/clear bits in the control
   register. This must be synchronized against the actual hardware.
   Care must also be taken to avoid clearing bits that may have been
   set by the hardware during the operation. The actual code comes
   from the Atmel sample drivers. Bitmap for all status bits in CSR. */

#define AT91C_UDP_STALLSENT AT91C_UDP_ISOERROR
#define REG_NO_EFFECT_1_ALL AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1 |\
                            AT91C_UDP_STALLSENT | AT91C_UDP_RXSETUP |\
                            AT91C_UDP_TXCOMP

/* Sets the specified bit(s) in the UDP_CSR register. Takes the endpoint
   number of the CSR to process and the bitmap flags to set to 1. */

#define UDP_SETEPFLAGS(csr, flags) \
    { \
        volatile unsigned int reg; \
        reg = (csr) ; \
        reg |= REG_NO_EFFECT_1_ALL; \
        reg |= (flags); \
        do (csr) = reg; \
        while ( ((csr) & (flags)) != (flags)); \
    }

/* Sets the specified bit(s) in the UDP_CSR register. Takes the endpoint
   number of the CSR to process and the bitmap flags to set to 0. */

#define UDP_CLEAREPFLAGS(csr, flags) \
    { \
        volatile unsigned int reg; \
        reg = (csr); \
        reg |= REG_NO_EFFECT_1_ALL; \
        reg &= ~(flags); \
        do (csr) = reg; \
        while ( ((csr) & (flags)) != 0); \
    }

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* USB hardware states */
#define ST_READY       0x0
#define ST_CONFIGURED  0x1
#define ST_SUSPENDED   0x2

/* Driver flags */
#define ST_DISABLED    0x8000
#define ST_NEEDRESET   0x4000

#define SET_STATE(s) (config_state = (config_state & (ST_DISABLED|ST_NEEDRESET)) | (s))

/* USB events */
#define USB_READABLE     0x1
#define USB_WRITEABLE    0x2
#define USB_CONFIGURED   0x10
#define USB_UNCONFIGURED 0x20

/* Critical section macros. Disable and enable interrupts. */
#define ENTER_CS() (*AT91C_UDP_IDR = (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM))
#define LEAVE_CS() (*AT91C_UDP_IER = (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM))

static uint8 cur_config;
static uint32 cur_feat;

static unsigned cur_rx_bank;
static int config_state = (ST_DISABLED | ST_NEEDRESET);
static int new_address;

static uint8 *out_ptr;
static uint32 out_count;
static uint8 delay_on = 0;

/* Device descriptor */
static const uint8 dd[] =
{
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08,
    0x94, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x01
};

/* Configuration descriptor */
static const uint8 cfd[] =
{
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xC0,
    0x00, 0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF,
    0xFF, 0x00, 0x07, 0x05, 0x01, 0x02, 0x40, 0x00,
    0x00, 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00
};

/* Serial number descriptor. */
static uint8 snd[] =
{
    0x1A,           // Descriptor length
    0x03,           // Descriptor type 3 == string
    0x31, 0x00,     // MSD of Lap (Lap[2,3]) in UNICODE
    0x32, 0x00,     // Lap[4,5]
    0x33, 0x00,     // Lap[6,7]
    0x34, 0x00,     // Lap[8,9]
    0x35, 0x00,     // Lap[10,11]
    0x36, 0x00,     // Lap[12,13]
    0x37, 0x00,     // Lap[14,15]
    0x38, 0x00,     // LSD of Lap (Lap[16,17]) in UNICODE
    0x30, 0x00,     // MSD of Nap (Nap[18,19]) in UNICODE
    0x30, 0x00,     // LSD of Nap (Nap[20,21]) in UNICODE
    0x39, 0x00,     // MSD of Uap in UNICODE
    0x30, 0x00      // LSD of Uap in UNICODE
};

/* Name descriptor, we allow up to 16 UNICODE characters. */
static uint8 named[] =
{
    0x08,           // Descriptor length
    0x03,           // Descriptor type 3 == string
    0x6e,
    0x00,     // n
    0x78,
    0x00,     // x
    0x74,
    0x00,     // t
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};

static const uint8 ld[] = { 0x04, 0x03, 0x09, 0x04 }; // Language descriptor

static void reset()
{
    /* Setup default configuration state. */
    cur_config = 0;
    cur_feat = 0;
    cur_rx_bank = AT91C_UDP_RX_DATA_BK0;

    SET_STATE(ST_READY);

    delay_on = 0;
    new_address = -1;
    out_count = 0;
}

void force_reset()
{
    int i_state;

    /* Get the hardware offline. */
    *AT91C_PIOA_PER = (1 << 16);
    *AT91C_PIOA_OER = (1 << 16);
    *AT91C_PIOA_SODR = (1 << 16);
    *AT91C_PMC_SCDR = AT91C_PMC_UDP;
    *AT91C_PMC_PCDR = (1 << AT91C_ID_UDP);

    systick_wait_ms(2);

    /* Now bring it back online. */
    i_state = irqs_get_and_disable();

    /* Make sure the USB PLL and clock are set up. */
    *AT91C_CKGR_PLLR |= AT91C_CKGR_USBDIV_1;
    *AT91C_PMC_SCER = AT91C_PMC_UDP;
    *AT91C_PMC_PCER = (1 << AT91C_ID_UDP);
    *AT91C_UDP_FADDR = 0;
    *AT91C_UDP_GLBSTATE = 0;

    /* Enable the UDP pull up by outputting a zero on PA.16. */
    *AT91C_PIOA_PER = (1 << 16);
    *AT91C_PIOA_OER = (1 << 16);
    *AT91C_PIOA_CODR = (1 << 16);
    *AT91C_UDP_IDR = ~0;

    /* Set up default state. */
    reset();

    *AT91C_UDP_IER = (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM);

    if (i_state)
        irqs_enable();
}

sint32 udp_read(uint8* buf, uint16 off, uint16 len)
{
    int i, act_len = 0;

    if (len == 0)
        return 0;

    if (config_state != ST_CONFIGURED)
        return UDP_IO_ERR;

    /* Perform a non-blocking read operation. We use double buffering
       (ping-pong) operation to provide better throughput. */
    if ((*AT91C_UDP_CSR1) & cur_rx_bank) /* data to read */
    {
        act_len = ((*AT91C_UDP_CSR1) & AT91C_UDP_RXBYTECNT) >> 16;
        if (act_len > len)
            act_len = len;

        /* Transfer the data. */
        for (i = 0; i < act_len; i++)
            buf[off + i] = *AT91C_UDP_FDR1;

        /* Flip bank. */
        ENTER_CS();

        UDP_CLEAREPFLAGS(*AT91C_UDP_CSR1, cur_rx_bank);

        if (cur_rx_bank == AT91C_UDP_RX_DATA_BK0)
            cur_rx_bank = AT91C_UDP_RX_DATA_BK1;
        else
            cur_rx_bank = AT91C_UDP_RX_DATA_BK0;

        /* We may have an enable/reset pending - do it now if there
           is no data in the buffers. */
        if (delay_on && ((*AT91C_UDP_CSR1) & AT91C_UDP_RXBYTECNT) == 0)
        {
            delay_on = 0;

            UDP_CLEAREPFLAGS(*AT91C_UDP_CSR1, AT91C_UDP_FORCESTALL);

            (*AT91C_UDP_RSTEP) |= AT91C_UDP_EP1;
            (*AT91C_UDP_RSTEP) &= ~AT91C_UDP_EP1;
        }

        LEAVE_CS();

        /* Use special case for a real zero length packet so we can
           take it to indicate EOF. */
        if (act_len == 0)
            return UDP_IO_EOF;
        else
            return act_len;
    }

    return 0;
}

sint32 udp_write(uint8* buf, uint16 off, uint16 len)
{
    /* Perform a non-blocking write. Return the number of bytes
       actually written. */
    int i, act_len = len;

    if (len == 0)
        return 0;

    if (config_state != ST_CONFIGURED)
        return UDP_IO_ERR;

    /* Can we write ? */
    if ((*AT91C_UDP_CSR2 & AT91C_UDP_TXPKTRDY) != 0)
        return 0;

    /* Limit to max transfer size. */
    if (act_len > MAX_BUF)
        act_len = MAX_BUF;

    for (i = 0; i < act_len; i++)
        *AT91C_UDP_FDR2 = buf[off + i];

    ENTER_CS();

    UDP_SETEPFLAGS(*AT91C_UDP_CSR2, AT91C_UDP_TXPKTRDY);
    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR2, AT91C_UDP_TXCOMP);

    LEAVE_CS();

    return act_len;
}

static void udp_send_null()
{
    UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_TXPKTRDY);
}

static void udp_send_stall()
{
    UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_FORCESTALL);
}

static void udp_send_control(uint8* p, int len)
{
    int i;

    out_ptr = p;
    out_count = len;

    /* Start sending the first part of the data. */
    for (i = 0; i < 8 && i < out_count; i++)
        *AT91C_UDP_FDR0 = out_ptr[i];

    UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_TXPKTRDY);
}

static void udp_stall_endpoints()
{
    UDP_SETEPFLAGS(*AT91C_UDP_CSR1, AT91C_UDP_FORCESTALL);
    UDP_SETEPFLAGS(*AT91C_UDP_CSR2, AT91C_UDP_FORCESTALL);
    UDP_SETEPFLAGS(*AT91C_UDP_CSR3, AT91C_UDP_FORCESTALL);
}

static void udp_unstall_endpoints()
{
    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR1, AT91C_UDP_FORCESTALL);
    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR2, AT91C_UDP_FORCESTALL);
    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR3, AT91C_UDP_FORCESTALL);
}

static void udp_enumerate()
{
    uint8 bt, br;
    int i, req, len, ind, val;
    short status;

    /* First we deal with any completion states. */
    if ((*AT91C_UDP_CSR0) & AT91C_UDP_TXCOMP)
    {
        /* Write operation has completed. Send config data if needed.
           Send a zero length packet to mark the end of the data if
           an exact multiple of 8. */
        if (out_count >= 8)
        {
            out_count -= 8;
            out_ptr += 8;

            /* Send next part of the data */
            for (i = 0; i < 8 && i < out_count; i++)
                *AT91C_UDP_FDR0 = out_ptr[i];
            UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_TXPKTRDY);
        }
        else
            out_count = 0;

        /* Clear the state */
        UDP_CLEAREPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_TXCOMP);
        if (new_address >= 0)
        {
            /* Set new address */
            *AT91C_UDP_FADDR = (AT91C_UDP_FEN | new_address);
            *AT91C_UDP_GLBSTATE = (new_address) ? AT91C_UDP_FADDEN : 0;
            new_address = -1;
        }
    }
    if ((*AT91C_UDP_CSR0) & (AT91C_UDP_RX_DATA_BK0))
    {
        /* Got transfer complete ack, clear the state */
        UDP_CLEAREPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_RX_DATA_BK0);
    }
    if (*AT91C_UDP_CSR0 & AT91C_UDP_ISOERROR)
    {
        /* Clear the state */
        UDP_CLEAREPFLAGS(*AT91C_UDP_CSR0,
                         (AT91C_UDP_ISOERROR | AT91C_UDP_FORCESTALL));
    }

    if (!((*AT91C_UDP_CSR0) & AT91C_UDP_RXSETUP))
        return;

    bt = *AT91C_UDP_FDR0;
    br = *AT91C_UDP_FDR0;

    val = ((*AT91C_UDP_FDR0 & 0xFF) | (*AT91C_UDP_FDR0 << 8));
    ind = ((*AT91C_UDP_FDR0 & 0xFF) | (*AT91C_UDP_FDR0 << 8));
    len = ((*AT91C_UDP_FDR0 & 0xFF) | (*AT91C_UDP_FDR0 << 8));

    if (bt & 0x80)
    {
        UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_DIR);
    }

    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_RXSETUP);

    req = br << 8 | bt;

    /* If we are disabled we respond to some requests with a stall the idea is
       to allow initialization/enumeration operations to continue to work when
       a program that is not using USB is running, but to prevent attempts to
       perform actual data transfers. */
    if ((config_state & (ST_DISABLED | ST_CONFIGURED)) ==
            (ST_DISABLED | ST_CONFIGURED) &&
        (req < STD_GET_STATUS_ZERO || req > STD_GET_STATUS_ENDPOINT))
    {
        udp_send_stall();
        return;
    }
    switch (req)
    {
    case STD_GET_DESCRIPTOR: {
        if (val == 0x100) /* Device descriptor */
        {
            udp_send_control((uint8*)dd, MIN(sizeof(dd), len));
        }
        else if (val == 0x200) /* Configuration descriptor */
        {
            udp_send_control((uint8*)cfd, MIN(sizeof(cfd), len));
        }
        else if ((val & 0xF00) == 0x300)
        {
            switch (val & 0xFF)
            {
            case 0x00:
                udp_send_control((uint8*)ld, MIN(sizeof(ld), len));
                break;
            case 0x01:
                udp_send_control(snd, MIN(sizeof(snd), len));
                break;
            default:
                udp_send_stall();
                break;
            }
        }
        else
        {
            udp_send_stall();
        }
        break;
    }
    case STD_SET_ADDRESS:
    {
        new_address = val;
        udp_send_null();
        break;
    }
    case STD_SET_CONFIGURATION:
    {
        cur_config = val;
        if (val)
        {
            SET_STATE(ST_CONFIGURED);
            *AT91C_UDP_GLBSTATE = AT91C_UDP_CONFG;
            delay_on = 0;

            /* Make sure we are not stalled. */
            udp_unstall_endpoints();

            /* Now enable the endpoints ... */
            UDP_SETEPFLAGS(*AT91C_UDP_CSR1,
                           (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_OUT));
            UDP_SETEPFLAGS(*AT91C_UDP_CSR2,
                           (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN));
            UDP_SETEPFLAGS(*AT91C_UDP_CSR3, AT91C_UDP_EPTYPE_INT_IN);

            /* ... and reset them. */
            (*AT91C_UDP_RSTEP) |=
                (AT91C_UDP_EP1 | AT91C_UDP_EP2 | AT91C_UDP_EP3);
            (*AT91C_UDP_RSTEP) &=
                ~(AT91C_UDP_EP1 | AT91C_UDP_EP2 | AT91C_UDP_EP3);
            if (config_state & ST_DISABLED)
            {
                /* We are disabled so we stall the endpoints for now. */
                udp_stall_endpoints();
            }
        }
        else
        {
            SET_STATE(ST_READY);
            *AT91C_UDP_GLBSTATE = AT91C_UDP_FADDEN;
            delay_on = 0;
            UDP_CLEAREPFLAGS(*AT91C_UDP_CSR1,
                             AT91C_UDP_EPEDS | AT91C_UDP_FORCESTALL);
            UDP_CLEAREPFLAGS(*AT91C_UDP_CSR2,
                             AT91C_UDP_EPEDS | AT91C_UDP_FORCESTALL);
            *AT91C_UDP_CSR3 = 0;
        }

        udp_send_null();

        break;
    }
    case STD_SET_FEATURE_ENDPOINT:
    {
        ind &= 0x0F;

        if ((val == 0) && ind && (ind <= 3))
        {
            switch (ind)
            {
            case 1:
                UDP_SETEPFLAGS(*AT91C_UDP_CSR1, AT91C_UDP_FORCESTALL);
                delay_on = 0;
                break;
            case 2:
                UDP_SETEPFLAGS(*AT91C_UDP_CSR2, AT91C_UDP_FORCESTALL);
                break;
            case 3:
                UDP_SETEPFLAGS(*AT91C_UDP_CSR3, AT91C_UDP_FORCESTALL);
                break;
            }
            udp_send_null();
        }
        else
        {
            udp_send_stall();
        }
        break;
    }
    case STD_CLEAR_FEATURE_ENDPOINT:
    {
        ind &= 0x0F;

        if ((val == 0) && ind && (ind <= 3))
        {
            /* Enable and reset the end point. */
            int res = 0;
            switch (ind)
            {
            case 1:
            {
                /* We need to take special care for the input end point because
                   we may have data in the hardware buffer. If we do then the
                   reset will cause this to be lost. To prevent this loss we
                   delay the enable until the data has been read. */
                if ((*AT91C_UDP_CSR1) & cur_rx_bank)
                {
                    UDP_SETEPFLAGS(*AT91C_UDP_CSR1, AT91C_UDP_FORCESTALL);
                    delay_on = 1;
                }
                else
                {
                    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR1, AT91C_UDP_FORCESTALL);
                    delay_on = 0;
                    res = AT91C_UDP_EP1;
                }
                break;
            }
            case 2:
            {
                UDP_CLEAREPFLAGS(*AT91C_UDP_CSR2, AT91C_UDP_FORCESTALL);
                res = AT91C_UDP_EP2;
                break;
            }
            case 3:
            {
                UDP_CLEAREPFLAGS(*AT91C_UDP_CSR3, AT91C_UDP_FORCESTALL);
                res = AT91C_UDP_EP3;
                break;
            }
            }
            (*AT91C_UDP_RSTEP) |= res;
            (*AT91C_UDP_RSTEP) &= ~res;
            udp_send_null();
        }
        else
        {
            udp_send_stall();
        }
        break;
    }
    case STD_GET_CONFIGURATION:
    {
        udp_send_control((uint8*)&(cur_config), MIN(sizeof(cur_config), len));
        break;
    }
    case STD_GET_STATUS_ZERO:
    {
        status = 0x01;
        udp_send_control((uint8*)&status, MIN(sizeof(status), len));
        break;
    }
    case STD_GET_STATUS_INTERFACE:
    {
        status = 0;
        udp_send_control((uint8*)&status, MIN(sizeof(status), len));
        break;
    }
    case STD_GET_STATUS_ENDPOINT:
    {
        status = 0;
        ind &= 0x0F;

        if (((*AT91C_UDP_GLBSTATE) & AT91C_UDP_CONFG) && (ind <= 3))
        {
            switch (ind)
            {
            case 1:
                status = ((*AT91C_UDP_CSR1) & AT91C_UDP_FORCESTALL) ? 1 : 0;
                break;
            case 2:
                status = ((*AT91C_UDP_CSR2) & AT91C_UDP_FORCESTALL) ? 1 : 0;
                break;
            case 3:
                status = ((*AT91C_UDP_CSR3) & AT91C_UDP_FORCESTALL) ? 1 : 0;
                break;
            }
            udp_send_control((uint8*)&status, MIN(sizeof(status), len));
        }
        else if (((*AT91C_UDP_GLBSTATE) & AT91C_UDP_FADDEN) && (ind == 0))
        {
            status = ((*AT91C_UDP_CSR0) & AT91C_UDP_EPEDS) ? 0 : 1;
            udp_send_control((uint8*)&status, MIN(sizeof(status), len));
        }
        else
        {
            udp_send_stall(); /* Illegal request */
        }
        break;
    }
    case VENDOR_SET_FEATURE_INTERFACE:
    {
        ind &= 0xf;
        cur_feat |= (1 << ind);
        udp_send_null();
        break;
    }
    case VENDOR_CLEAR_FEATURE_INTERFACE:
    {
        ind &= 0xf;
        cur_feat &= ~(1 << ind);
        udp_send_null();
        break;
    }
    case VENDOR_GET_DESCRIPTOR:
    {
        udp_send_control((uint8*)named, MIN(named[0], len));
        break;
    }
    case STD_SET_FEATURE_INTERFACE:
    case STD_CLEAR_FEATURE_INTERFACE:
    {
        udp_send_null();
        break;
    }
    case STD_SET_INTERFACE:
    case STD_SET_FEATURE_ZERO:
    case STD_CLEAR_FEATURE_ZERO:
    default:
    {
        udp_send_stall();
        break;
    }
    }
}

void udp_isr_handler(void)
{
    /* Process interrupts. We mainly use these during the configuration
       and enumeration stages. */
    if (*AT91C_UDP_ISR & END_OF_BUS_RESET)
    {
        *AT91C_UDP_ICR = END_OF_BUS_RESET;
        *AT91C_UDP_ICR = SUSPEND_RESUME;
        *AT91C_UDP_ICR = WAKEUP;
        *AT91C_UDP_RSTEP = 0xFFFFFFFF;
        *AT91C_UDP_RSTEP = 0x0;
        *AT91C_UDP_FADDR = AT91C_UDP_FEN;

        reset();

        UDP_SETEPFLAGS(*AT91C_UDP_CSR0,
                       (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL));
        *AT91C_UDP_IER =
            (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM);
        return;
    }
    if (*AT91C_UDP_ISR & SUSPEND_INT)
    {
        if ((config_state & ~ST_DISABLED) == ST_CONFIGURED)
        {
            SET_STATE(ST_SUSPENDED);
        }
        else
        {
            SET_STATE(ST_READY);
        }

        *AT91C_UDP_ICR = SUSPEND_INT;

        cur_rx_bank = AT91C_UDP_RX_DATA_BK0;
    }
    if (*AT91C_UDP_ISR & SUSPEND_RESUME)
    {
        if ((config_state & ~ST_DISABLED) == ST_SUSPENDED)
        {
            SET_STATE(ST_CONFIGURED);
        }
        else
        {
            SET_STATE(ST_READY);
        }

        *AT91C_UDP_ICR = WAKEUP;
        *AT91C_UDP_ICR = SUSPEND_RESUME;
    }
    if (*AT91C_UDP_ISR & AT91C_UDP_EPINT0)
    {
        *AT91C_UDP_ICR = AT91C_UDP_EPINT0;
        udp_enumerate();
    }
}

sint32 udp_get_state()
{
    /* Return the current status of the USB connection. This information
       can be used to determine if the connection can be used. We return
       the connected state, the currently selected configuration and the
       currently active features. This latter item is used by cooperating
       software on the PC and NXT to indicate the start and end of a
       stream connection. */
    sint32 ret = 0;

    if (config_state == ST_CONFIGURED)
    {
        ret |= USB_CONFIGURED;
        if ((*AT91C_UDP_CSR1) & cur_rx_bank)
            ret |= USB_READABLE;
        if ((*AT91C_UDP_CSR2 & AT91C_UDP_TXPKTRDY) == 0)
            ret |= USB_WRITEABLE;
    }
    else
    {
        ret = USB_UNCONFIGURED;
    }
    return ret;
}

void udp_enable(sint32 reset)
{
    /* Enable the processing of USB requests. */
    if (reset & 0x2)
    {
        return;
    }

    int i_state = irqs_get_and_disable();

    /* Initialize the interrupt handler. We use a very low priority because
       some of the USB operations can run for a relatively long time. */
    aic_mask_off(AT91C_PERIPHERAL_ID_UDP);
    aic_set_vector(AT91C_PERIPHERAL_ID_UDP, AIC_INT_LEVEL_LOWEST,
                   (uint32)udp_isr_handler);
    aic_mask_on(AT91C_PERIPHERAL_ID_UDP);

    *AT91C_UDP_IER = (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM);

    reset = reset || (config_state & ST_NEEDRESET);
    config_state &= ~(ST_DISABLED | ST_NEEDRESET);

    if (i_state)
        irqs_enable();

    if (reset)
    {
        force_reset();
    }
    else if (config_state & ST_CONFIGURED)
    {
        /* Un-stall the endpoints if we previously stalled them... */
        udp_unstall_endpoints();
    }
}

void udp_disable()
{
    /* Disable processing of USB requests. */
    uint8 buf[MAX_BUF];

    /* Stall the endpoints, note we can not reset them at this point as
       this will screw up the data toggle and result in lost data. */
    if (config_state & ST_CONFIGURED)
    {
        udp_stall_endpoints();

        /* Discard any input */
        while (udp_read(buf, 0, sizeof(buf)) > 0)
            ;
    }

    unsigned i_state = irqs_get_and_disable();

    config_state |= ST_DISABLED;
    cur_feat = 0;

    if (i_state)
    {
        irqs_enable();
    }
}

void udp_set_serial_no(uint8* ser_no, uint16 len)
{
    /* Set the USB serial number, which is a 12 character UNICODE string
       containing the USB serial number. */
    if (len == (sizeof(snd) - 2) / 2)
    {
        memcpy(snd + 2, ser_no, len * 2);
    }
}

void udp_set_name(uint8* name, uint16 len)
{
    if (len <= (sizeof(named) - 2) / 2)
    {
        memcpy(named + 2, name, len * 2);
        named[0] = len * 2 + 2;
    }
}

void udp_init(void)
{
    udp_disable();

    config_state = (ST_DISABLED | ST_NEEDRESET);
}

void udp_reset(void)
{
    if (config_state & ST_DISABLED)
    {
        return;
    }

    force_reset();
    udp_disable();
}
