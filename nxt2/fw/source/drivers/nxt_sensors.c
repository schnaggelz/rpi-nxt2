/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C driver code.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "drivers/nxt_sensors.h"

#include "drivers/nxt_avr.h"

#include "platform/i2c.h"

#include "platform/at91/at91sam7s256.h"

typedef struct
{
    sint8 type;
    sint8 mode;
    sint8 boolean;
    sint16 raw;
    sint16 value;
} sensor_port;

static sensor_port sensor_ports[NXT_NUM_SENSOR_PORTS] =
{
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}
};

void nxt_sensor_set_digi0(uint8 port)
{
    /* Enable output on the pin. */
    int functions[] = {AT91C_PIO_PA23, AT91C_PIO_PA28, AT91C_PIO_PA29,
                       AT91C_PIO_PA30};

    *AT91C_PIOA_PER |= functions[port];
    *AT91C_PIOA_OER |= functions[port];

    /* Set high. */
    *AT91C_PIOA_SODR |= functions[port];
}

void nxt_sensor_unset_digi0(uint8 port)
{
    /* Enable output on the pin. */
    int functions[] = {AT91C_PIO_PA23, AT91C_PIO_PA28, AT91C_PIO_PA29,
                       AT91C_PIO_PA30};

    *AT91C_PIOA_PER |= functions[port];
    *AT91C_PIOA_OER |= functions[port];

    /* Set low. */
    *AT91C_PIOA_CODR |= functions[port];
}

void nxt_sensor_set_digi1(uint8 port)
{
    /* Enable output on the pin. */
    int functions[] = {AT91C_PIO_PA18, AT91C_PIO_PA19, AT91C_PIO_PA20,
                       AT91C_PIO_PA2};

    *AT91C_PIOA_PER |= functions[port];
    *AT91C_PIOA_OER |= functions[port];

    /* Set high. */
    *AT91C_PIOA_SODR |= functions[port];
}

void nxt_sensor_unset_digi1(uint8 port)
{
    /* Enable output on the pin. */
    int functions[] = {AT91C_PIO_PA18, AT91C_PIO_PA19, AT91C_PIO_PA20,
                       AT91C_PIO_PA2};

    *AT91C_PIOA_PER |= functions[port];
    *AT91C_PIOA_OER |= functions[port];

    /* Set low. */
    *AT91C_PIOA_CODR |= functions[port];
}

void nxt_sensors_init(void)
{
    uint8 port;

    for (port = 0; port < NXT_NUM_SENSOR_PORTS; port++)
    {
        nxt_sensor_unset_digi0(port);
        nxt_sensor_unset_digi1(port);
        nxt_avr_set_input_power(port, 0);
    }
    /* Ensure RS485 is inactive. Otherwise it can interfere with
     * the operation of port 4. */
    *AT91C_PIOA_PER |= AT91C_PIO_PA5 | AT91C_PIO_PA6 | AT91C_PIO_PA7;
    *AT91C_PIOA_PPUDR |= AT91C_PIO_PA5 | AT91C_PIO_PA6 | AT91C_PIO_PA7;
    *AT91C_PIOA_OER |= AT91C_PIO_PA5 | AT91C_PIO_PA6 | AT91C_PIO_PA7;
    *AT91C_PIOA_CODR |= AT91C_PIO_PA5 | AT91C_PIO_PA6 | AT91C_PIO_PA7;
}

void nxt_sensors_poll(void)
{
    uint8 port;

    for (port = 0; port < NXT_NUM_SENSOR_PORTS; port++)
    {
        sensor_ports[port].value = nxt_avr_get_sensor_adc(port);
    }
}

void nxt_sensor_init_i2c(uint8 port, uint8 type)
{
    nxt_avr_set_input_power(port, type);

    i2c_enable(port, I2C_LEGO_MODE);
}
