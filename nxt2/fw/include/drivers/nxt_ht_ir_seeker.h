#ifndef __NXT_HT_IR_SEEKER_H__
#define __NXT_HT_IR_SEEKER_H__

#include "nxt_devices.h"

#ifdef __cplusplus
extern "C" {
#endif

void nxt_ht_ir_seeker_init(uint8 port);
void nxt_ht_ir_seeker_get_data(uint8 port, sint8 buf[12]);
void nxt_ht_ir_seeker_term(uint8);

#ifdef __cplusplus
}
#endif

#endif /* __NXT_HT_IR_SEEKER_H__ */
