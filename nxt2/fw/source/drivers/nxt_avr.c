/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * This module handles communications between the NXT main ARM processor and
 * the AVR processor. The AVR provides support for the motors, analogue sensors
 * keyboard and power. The two processors are linked via a TWI connection. This
 * is used to exchange a message every 1ms. The message alternates between
 * sending commands to the AVR and receiving status from it.
 * NOTES:
 * The time window for read requests is very tight. On some NXT devices it can
 * be exceeded. This code has been optimized to maximize the read window and
 * to handle the times that the window is exceeded.
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "drivers/nxt_avr.h"

#include "drivers/nxt_motors.h"

#include "platform/twi.h"

#include <string.h>

/* Link states */
#define LS_CLOSED 0
#define LS_INIT1 1
#define LS_INIT2 2
#define LS_RUNNING 3
#define LS_RETRY 4
#define LS_RESET 5

/* Input data is double buffered by buffer flipping. */
#define NEXT_BUF() ((avr_input_buf_idx + 1) & 0x1)

/* 50ms Debounce time. Button read is called every other 1000Hz tick. */
#define BUTTON_DEBOUNCE_CNT 50 / 2;

/* This string is used to establish communication with the AVR. */
const char avr_brainwash_string[] =
    "\xCC"
    "Let's samba nxt arm in arm, (c)LEGO System A/S";

/* The following Raw values are read/written directly to the AVR.
   So byte order, packing etc. must match. */
typedef struct
{
    // Raw values
    uint8 power;
    uint8 pwm_frequency;
    sint8 output_percent[NXT_AVR_N_OUTPUTS];
    uint8 output_mode;
    uint8 input_power;
} __attribute__((packed)) avr_output_data_t;

static avr_output_data_t avr_output_data;

/* Output data is double buffered via the following (note extra
   space for checksum. */
static uint8 avr_output_data_buf[5 + NXT_AVR_N_OUTPUTS];

typedef struct
{
    /* Raw values */
    uint16 adc_value[NXT_AVR_N_INPUTS];
    uint16 buttons_value;
    uint16 extra;
    uint8 checksum;
} __attribute__((packed)) avr_input_data_t;

static avr_input_data_t avr_input_data[2];
static avr_input_data_t* avr_input_buf;

static uint32 avr_input_buf_idx;
static uint16 button_state;
static uint16 button_prev_state;
static uint16 button_debounce_state;
static uint16 button_debounce_cnt;

/* TX/RX statistics */
static struct
{
    uint32 good_rx;
    uint32 bad_rx;
    uint32 resets;
    uint32 still_busy;
    uint32 not_ok;
} avr_stats;

/**
 * Start to read the status data from the AVR. The actual I/O takes place
 * via DMA/Interrupt handling.
 */
static void nxt_avr_start_read(void)
{
    twi_start_read(NXT_AVR_ADDRESS,
                   (uint8*)(&avr_input_data[avr_input_buf_idx]),
                   sizeof(avr_input_data_t));
}

/**
 * Start to send command data to the AVR.
 */
static void nxt_avr_start_send(void)
{
    uint32 check_byte = 0;
    uint8* a = avr_output_data_buf;
    uint8* b = (uint8*)(&avr_output_data);
    uint8* e = b + sizeof(avr_output_data);

    /* Copy over the data and create the checksum. */
    while (b < e)
    {
        check_byte += *b;
        *a++ = *b++;
    }

    *a = ~check_byte;

    twi_start_write(NXT_AVR_ADDRESS, avr_output_data_buf,
                    sizeof(avr_output_data_buf));
}

/**
 * Tell the AVR to power down the NXT.
 */
void nxt_avr_power_down(void)
{
    avr_output_data.power = 0x5a;
    avr_output_data.pwm_frequency = 0x00;
}

/**
 * Tell the AVR to enter SAMBA mode.
 */
void nxt_avr_firmware_update_mode(void)
{
    avr_output_data.power = 0xA5;
    avr_output_data.pwm_frequency = 0x5A;
}

/**
 * Initialise the link with the AVR by sending the handshake string. Note that
 * because of the length of this string we need to allow more than 1ms for it
 * to go.
 */
void nxt_avr_link_init(void)
{
    twi_start_write(NXT_AVR_ADDRESS, (const uint8*)avr_brainwash_string,
                    strlen(avr_brainwash_string));
}

/**
 * Unpack the status data from the AVR. Also need to check the checksum byte
 * is ok.
 */
static void nxt_avr_unpack(void)
{
    uint8 checksum = 0;
    uint8* p;
    uint8* end;
    uint16 buttons_value;
    uint16 new_state;

    /* Calculate the checksum. */
    p = (uint8*)(&avr_input_data[avr_input_buf_idx]);
    end = p + sizeof(avr_input_data_t);
    while (p < end)
    {
        checksum += *p++;
    }
    if (checksum != 0xff)
    {
        avr_stats.bad_rx++;
        return;
    }
    avr_stats.good_rx++;

    /* Flip the buffers. */
    avr_input_buf = &avr_input_data[avr_input_buf_idx];
    avr_input_buf_idx = NEXT_BUF();
    buttons_value = avr_input_buf->buttons_value;

    if (buttons_value > 60 || button_state)
    {
        /* Process the buttons. First we drop any noisy inputs. */
        if (buttons_value != button_prev_state)
        {
            button_prev_state = buttons_value;
        }
        else
        {
            /* Work out which buttons are down. We allow chording
               of the enter button with other buttons. */
            new_state = 0;
            if (buttons_value > 1500)
            {
                new_state |= 1;
                buttons_value -= 0x7ff;
            }

            if (buttons_value > 720)
                new_state |= 0x08;
            else if (buttons_value > 270)
                new_state |= 0x04;
            else if (buttons_value > 60)
                new_state |= 0x02;

            /* Debounce things... */
            if (new_state != button_debounce_state)
            {
                button_debounce_cnt = BUTTON_DEBOUNCE_CNT;
                button_debounce_state = new_state;
            }
            else if (button_debounce_cnt > 0)
            {
                button_debounce_cnt--;
            }
            else
            {
                /* Got a good key, make a note of it. */
                button_state = button_debounce_state;
            }
        }
    }
}

static uint32 update_count;
static uint32 link_state = LS_CLOSED;
static uint8 avr_initialized = 0;

/**
 * Set things up ready to start talking to the AVR.
 */
void nxt_avr_init(void)
{
    memset(avr_input_data, 0, sizeof(avr_input_data));
    memset(&avr_output_data, 0, sizeof(avr_output_data));

    link_state = LS_RESET;

    button_state = 0;
    button_prev_state = 0;
    button_debounce_state = 0;
    button_debounce_cnt = BUTTON_DEBOUNCE_CNT;

    avr_input_buf = &avr_input_data[1];
    avr_input_buf_idx = 0;
    avr_output_data.power = 0;
    avr_output_data.pwm_frequency = 8;
    avr_initialized = 1;
}

/**
 * Main processing function. Called from a low priority interrupt every 1ms.
 */
void nxt_avr_1kHz_update(void)
{
    if (!avr_initialized)
        return;

    int state;

    switch (link_state)
    {
    case LS_CLOSED:
        break;

    case LS_INIT1:
    case LS_INIT2:
        /* Add extra wait states during initialisation. */
        link_state++;
        break;

    case LS_RUNNING:
    case LS_RETRY:
        /* Check to make sure the link is ok. */
        state = twi_status();
        if (state == 0)
        {
            /* Everything looks good, so do the real work. */
            if (update_count++ & 1)
            {
                nxt_avr_start_read();
            }
            else
            {
                nxt_avr_start_send();
                nxt_avr_unpack();
            }
            link_state = LS_RUNNING;
        }
        else
        {
            if (state < 0)
            {
                avr_stats.not_ok++;
                link_state = LS_RESET;
            }
            else
            {
                avr_stats.still_busy++;
                /* If the link is still busy (normally it should not be).
                   We allow it a little extra time to see if it will
                   complete. If not then reset it. */
                if (link_state == LS_RUNNING)
                    link_state = LS_RETRY;
                else
                    link_state = LS_RESET;
            }
        }
        break;

    case LS_RESET:
    default:
        /* Either we are just starting or we have had a problem.
           So reset the HW and try and re-establish the link. */
        link_state = LS_INIT1;
        update_count = 0;

        twi_init();
        nxt_avr_link_init();

        avr_stats.resets++;
        break;
    }
}

/**
 * Return a bitmask giving the current (debounced) button state.
 */
uint32 nxt_avr_get_button_state(void)
{
    return button_state;
}

/**
 * Return the current state of the battery.
 */
uint32 nxt_avr_get_battery_voltage(void)
{
    /* Figure out voltage: The units are 13.848 mV per bit.
       To prevent fp, we substitute 13.848 with 14180/1024. */
    uint32 voltage_val = avr_input_buf->extra;
    voltage_val &= 0x3ff; /* Toss unwanted bits. */
    voltage_val *= 14180;
    voltage_val >>= 10;

    return voltage_val;
}

/**
 * Return the requests sensor analoge reading.
 */
uint32 nxt_avr_get_sensor_adc(uint8 port)
{
    if (port < NXT_AVR_N_INPUTS)
    {
        return avr_input_buf->adc_value[port];
    }

    return 0;
}

/**
 * Set the motor power for a particular motor.
 */
void nxt_avr_set_motor(uint8 port, sint8 power_percent, uint8 brake)
{
    if (port < NXT_NUM_MOTOR_PORTS)
    {
        avr_output_data.output_percent[port] = power_percent;
        if (brake)
            avr_output_data.output_mode |= (1 << port);
        else
            avr_output_data.output_mode &= ~(1 << port);
    }
}

/**
 * Control the power supplied to an input sensor.
 */
void nxt_avr_set_input_power(uint8 port, uint32 power_type)
{
    /* The power to the sensor is controlled by a bit in
     * each of the two nibbles of the byte. There is one
     * bit for each of the four sensors. if the low nibble
     * bit is set then the sensor is "ACTIVE" and 9V is
     * supplied to it but it will be pulsed off to allow
     * the sensor to be be read. A 1 in the high nibble
     * indicates that it is a 9V always on sensor and
     * 9V will be supplied constantly. If both bits are
     * clear then 9V is not supplied to the sensor.
     * Having both bits set is currently not supported. */
    if (port < NXT_AVR_N_INPUTS && power_type <= 2)
    {
        uint8 val =
            (power_type & 0x2 ? 0x10 << port : 0) | ((power_type & 1) << port);

        avr_output_data.input_power &= ~(0x11 << port);
        avr_output_data.input_power |= val;
    }
}
