#include "nxt_color_sensor.h"

#include "at91sam7.h"
#include "ports.h"
#include "systick.h"

/*  NXT color sensor mode commands (each bit is sent to the sensor) */
#define COLORSENSOR        (0xB0)
#define LIGHTSENSOR_BLUE   (0x08)
#define LIGHTSENSOR_RED    (0x70)
#define LIGHTSENSOR_GREEN  (0xF0)
#define LIGHTSENSOR_NONE   (0x88)

typedef struct
{
    uint16 color;
    uint16 light;
    sint16 rgb[3];
    volatile uint8 mode;
    volatile uint8 prev_mode;
    volatile uint8 port_in_use;
} color_sensor_port;

static color_sensor_port sensor_ports[NXT_NUM_SENSOR_PORTS] = 
{
    { .port_in_use = 0 },
    { .port_in_use = 0 },
    { .port_in_use = 0 },
    { .port_in_use = 0 }
};

const uint8 sensor_protocols[NUM_COLORSENSOR_MODES] =
{
    COLORSENSOR,
    LIGHTSENSOR_RED,
    LIGHTSENSOR_GREEN,
    LIGHTSENSOR_BLUE,
    COLORSENSOR,
    LIGHTSENSOR_NONE,
    LIGHTSENSOR_NONE
};

static void nxt_color_sensor_sp_send_byte(uint8 port, uint8 value)
{
    sp_set_mode(port, SP_DIGI0, SP_MODE_OUTPUT);
    sp_set_mode(port, SP_DIGI1, SP_MODE_OUTPUT);

    for (volatile int i = 0; i < 200; i++) { }
    sp_set(port, SP_DIGI0, 0);
    
    systick_wait_ms(100);

    for (volatile int i = 0; i < 8; i++)
    {
        for (volatile int j = 0; j < 200; j++) { }
        sp_set(port, SP_DIGI0, 1);

        if ((value & 0x80) == 0x80)
        {
            sp_set(port, SP_DIGI1, 1);
        }
        else
        {
            sp_set(port, SP_DIGI1, 0);
        }
        value = value << 1;

        for (volatile int j = 0; j < 200; j++) { }
        sp_set(port, SP_DIGI0, 0);
    }

    sp_set_mode(port, SP_DIGI1, SP_MODE_INPUT);
}

static void nxt_color_sensor_sp_receive_byte(uint8 port, uint32 *data)
{
    uint32 value = 0;

    for (volatile int i = 0; i < 8; i++)
    {
        for (volatile int j = 0; j < 100; j++) { }
        sp_set(port, SP_DIGI0, 1);

        for (volatile int j = 0; j < 100; j++) { }
        sp_set(port, SP_DIGI0, 0);

        value = value << 1;
        value |= sp_get(port, SP_DIGI1);
    }

    *data = value;
}

static void nxt_color_sensor_sp_init(uint8 port, uint8 address)
{
    uint32 data;

    sp_reset(port);

    nxt_color_sensor_sp_send_byte(port, address);

    for (volatile int i = 0; i < 3200; i++) { }
    for (volatile int i = 0; i < 52; i++)
    {
        nxt_color_sensor_sp_receive_byte(port, &data);
        for (volatile int j = 0; j < 3800; j++) { }
    }

    nxt_color_sensor_sp_receive_byte(port, &data);
    nxt_color_sensor_sp_receive_byte(port, &data);
}

static void nxt_color_sensor_update_light_data(uint8 port)
{
    volatile int dummy;
    uint16 light = 0;

    sp_set_mode(port, SP_DIGI1, SP_MODE_ADC);
    dummy = sp_read(port, SP_DIGI1);
    while (1)
    {
        if (((*AT91C_ADC_SR & sensor_pins[port].adc_channel)
                        == sensor_pins[port].adc_channel))
        {
            light = sp_read(port, SP_DIGI1);
            break;
        }
    }
    sensor_ports[port].light = light;
}

static void nxt_color_sensor_update_color_data(uint8 port)
{
    volatile int dummy;
    int color = NXT_COLOR_UNKNOWN;
    int red, green, blue = 0;

    sp_set(port, SP_DIGI0, 1);
    dummy = sp_read(port, SP_DIGI1);
    while (1)
    {
        if (((*AT91C_ADC_SR & sensor_pins[port].adc_channel)
                        == sensor_pins[port].adc_channel))
        {
            red = sp_read(port, SP_DIGI1); /* red */
            break;
        }
    }
    sp_set(port, SP_DIGI0, 0);
    dummy = sp_read(port, SP_DIGI1);
    while (1)
    {
        if (((*AT91C_ADC_SR & sensor_pins[port].adc_channel)
                        == sensor_pins[port].adc_channel))
        {
            green = sp_read(port, SP_DIGI1); /* green */
            break;
        }
    }
    sp_set(port, SP_DIGI0, 1);
    dummy = sp_read(port, SP_DIGI1);
    while (1)
    {
        if (((*AT91C_ADC_SR & sensor_pins[port].adc_channel)
                        == sensor_pins[port].adc_channel))
        {
            blue = sp_read(port, SP_DIGI1); /* blue */
            break;
        }
    }
    sp_set(port, SP_DIGI0, 0);

    /* Analyse color based on RGB data. */
    if (red > green && green > blue && green > blue * 1.1)
    {
        if (green > 350)
            color = NXT_COLOR_YELLOW;
        else
            color = NXT_COLOR_ORANGE;
    }
    else if (red > blue && red > green && red > green * 1.5)
    {
        if (green > blue && green > 240)
            color = NXT_COLOR_ORANGE;
        else
            color = NXT_COLOR_RED;
    }
    else if (green < blue && green < red && blue < red)
    {
        if ((red + green + blue) > 900)
            color = NXT_COLOR_WHITE;
        else
            color = NXT_COLOR_BLACK;
    }
    else if (green > blue && green > red)
    {
        color = NXT_COLOR_GREEN;
    }
    else if (blue > red && blue > green)
    {
        color = NXT_COLOR_BLUE;
    }
    sensor_ports[port].rgb[0] = red;
    sensor_ports[port].rgb[1] = green;
    sensor_ports[port].rgb[2] = blue;
    sensor_ports[port].color = color;
}

static void nxt_color_sensor_ports_init(uint8 port)
{
    sensor_ports[port].rgb[0] = 0;
    sensor_ports[port].rgb[1] = 0;
    sensor_ports[port].rgb[2] = 0;
    sensor_ports[port].light = 0;
    sensor_ports[port].color = NXT_COLOR_UNKNOWN;
}

void nxt_color_sensor_init(uint8 port)
{
    nxt_color_sensor_ports_init(port);

    sensor_ports[port].mode = NXT_COLORSENSOR_DEACTIVATE;
    sensor_ports[port].prev_mode = NXT_COLORSENSOR_DEACTIVATE;
    sensor_ports[port].port_in_use = 0;
}

void nxt_color_sensor_set_mode(uint8 port, uint8 mode)
{
    sensor_ports[port].mode = mode;

    if (mode == NXT_COLORSENSOR_DEACTIVATE)
    {
        sensor_ports[port].port_in_use = 0;
    }
    else
    {
        sensor_ports[port].port_in_use = 1;
    }
}

uint16 nxt_color_sensor_get_light(uint8 port)
{
    return sensor_ports[port].light;
}

void nxt_color_sensor_get_rgb_data(uint8 port, sint16 rgb[3])
{
    rgb[0] = sensor_ports[port].rgb[0];
    rgb[1] = sensor_ports[port].rgb[1];
    rgb[2] = sensor_ports[port].rgb[2];
}

uint16 nxt_color_sensor_get_color(uint8 port)
{
    return sensor_ports[port].color;
}

uint8 nxt_color_sensor_get_mode(uint8 port)
{
    return sensor_ports[port].mode;
}

void nxt_color_sensor_term(uint8 port)
{
    sp_set(port, SP_DIGI0, 1);

    for (volatile int i = 0; i < 200; i++) { }
    sp_set(port, SP_DIGI0, 0);

    for (volatile int i = 0; i < 200; i++) { }
    sp_set(port, SP_DIGI0, 1);

    for (volatile int i = 0; i < 200; i++) { }
    sp_set(port, SP_DIGI0, 0);

    nxt_color_sensor_sp_init(port, LIGHTSENSOR_NONE); /* turn off the light */
}

void nxt_color_sensor_update(uint8 port)
{
    if (sensor_ports[port].port_in_use)
    {
        uint8 mode = sensor_ports[port].mode;
        if (mode != sensor_ports[port].prev_mode)
        {
            nxt_color_sensor_sp_init(port, LIGHTSENSOR_NONE);
            nxt_color_sensor_sp_init(port, sensor_protocols[mode]);
            nxt_color_sensor_ports_init(port);

            sensor_ports[port].prev_mode = mode;
        }

        nxt_color_sensor_update_light_data(port);

        if (mode == NXT_COLORSENSOR)
        {
            nxt_color_sensor_update_color_data(port);
        }
    }
}
