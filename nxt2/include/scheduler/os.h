#ifndef __OS_H__
#define __OS_H__

#include "systypes.h"
#include "systick.h"
#include "scheduler.h"
#include "tasks.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Init application callback. */
extern void os_app_init();

#ifdef __cplusplus
}
#endif

/* OS control functions. */
void os_init(void);
void os_start(void);
void os_tick(void);

uint8 os_running(void);

#endif /*__OS_H__*/