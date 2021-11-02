#include "at91/at91sam7s256.h"

#include "../../include/platform/platform.h"
#include "../../include/platform/irqs.h"
#include "../../include/platform/aic.h"
#include "../../include/platform/twi.h"
#include "../../include/platform/systick.h"
#include "../../include/platform/i2c.h"

void at91_platform_init()
{
    /* Init interrupt controller. */
    aic_init();

    /* Enable all interrupts. */
    irqs_enable();

    /* Initialize SPI. */
    twi_init();

    /* Initialize timer. */
    systick_init();

    /* Initialize I2C. */
    i2c_init();
}
