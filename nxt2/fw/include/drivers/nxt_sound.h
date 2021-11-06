/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_SOUND_H__
#define __NXT_SOUND_H__

#include "platform/systypes.h"

void nxt_sound_init(void);
void nxt_sound_enable(void);
void nxt_sound_disable(void);
void nxt_sound_isr_C(void);

void nxt_sound_freq(uint32 freq, uint32 ms);
void nxt_sound_freq_vol(uint32 freq, uint32 ms, uint8 vol);
void nxt_sound_play_sample(uint8* data, uint32 length, uint32 freq, uint8 vol);
void nxt_sound_set_volume(uint8 vol);
int nxt_sound_get_volume(void);
int nxt_sound_get_time(void);

#define MAXVOL 100

#endif /*__NXT_SOUND_H__*/
