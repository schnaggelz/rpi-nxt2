#ifndef __NXT_LIGHT_SENSOR_H__
#define __NXT_LIGHT_SENSOR_H__

#include "nxt_devices.h"
#include "nxt_sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

void nxt_light_sensor_init(uint8 port);
void nxt_light_sensor_term(uint8 port);
sint16 nxt_light_sensor_get_brightness(uint8 port);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_LIGHT_SENSOR_H__ */
