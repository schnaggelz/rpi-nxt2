/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* NXT C driver code.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __NXT_COLOR_SENSOR_H__
#define __NXT_COLOR_SENSOR_H__

#include "nxt_devices.h"
#include "nxt_sensors.h"

/* NXT color sensor mode macros */
#define NXT_COLORSENSOR              0 /* activates as a color sensor */
#define NXT_LIGHTSENSOR_RED          1 /* activates as a light sensor with red lamp */
#define NXT_LIGHTSENSOR_GREEN        2 /* activates as a light sensor with green lamp */
#define NXT_LIGHTSENSOR_BLUE         3 /* activates as a light sensor with blue lamp */
#define NXT_LIGHTSENSOR_WHITE        4 /* activates as a light sensor with white lamp */
#define NXT_LIGHTSENSOR_NONE         5 /* activates as a light sensor with no lamp */
#define NXT_COLORSENSOR_DEACTIVATE   6 /* deactivates the sensor */

/* NXT color sensor color number macros */
#define NXT_COLOR_BLACK              0
#define NXT_COLOR_BLUE               1
#define NXT_COLOR_GREEN              2
#define NXT_COLOR_YELLOW             3
#define NXT_COLOR_ORANGE             4
#define NXT_COLOR_RED                5
#define NXT_COLOR_WHITE              6
#define NXT_COLOR_UNKNOWN            99

#define NUM_COLORSENSOR_MODES        7

#ifdef __cplusplus
extern "C" {
#endif

void nxt_color_sensor_init(uint8 port);
void nxt_color_sensor_update(uint8 port);
void nxt_color_sensor_set_mode(uint8 port, uint8 mode);
uint8 nxt_color_sensor_get_mode(uint8 port);
uint16 nxt_color_sensor_get_light(uint8 port);
uint16 nxt_color_sensor_get_color(uint8 port);
void nxt_color_sensor_get_rgb_data(uint8, sint16[3]);
void nxt_color_sensor_term(uint8 port);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_COLOR_SENSOR_H__ */
