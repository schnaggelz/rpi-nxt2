#ifndef __NXT_SONAR_SENSOR_H__
#define __NXT_SONAR_SENSOR_H__

#include "platform/systypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void nxt_sonar_sensor_init(uint8 port);
void nxt_sonar_sensor_term(uint8 port);
sint32 nxt_sonar_sensor_get_distance(uint8 port);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_SONAR_SENSOR_H__ */
