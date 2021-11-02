#ifndef __NXT_HT_COMPASS_SENSOR_H__
#define __NXT_HT_COMPASS_SENSOR_H__

#include "nxt_devices.h"

#ifdef __cplusplus
extern "C" {
#endif

void nxt_ht_compass_sensor_init(uint8 port);
uint8 nxt_ht_compass_sensor_calibrate(uint8 port);
sint16 nxt_ht_compass_sensor_get_direction(uint8 port);
void nxt_ht_compass_sensor_term(uint8 port);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_HT_COMPASS_SENSOR_H__ */
