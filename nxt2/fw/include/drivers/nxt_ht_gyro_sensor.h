/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_HT_GYRO_SENSOR_H__
#define __NXT_HT_GYRO_SENSOR_H__

#include "nxt_devices.h"

#ifdef __cplusplus
extern "C" {
#endif

void nxt_ht_gyro_sensor_init(uint8 port);
uint16 nxt_ht_gyro_sensor_get_angular_velocity_preview(uint8 port);
uint16 nxt_ht_gyro_sensor_get_angular_velocity(uint8 port);
void nxt_ht_gyro_sensor_term(uint8 port);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_HT_GYRO_SENSOR_H__ */
