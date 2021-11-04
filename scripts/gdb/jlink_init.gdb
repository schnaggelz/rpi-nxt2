flash breakpoints = 0
flash download = 0
reset
long 0xffffff60 0x00320100  # set flash wait state (AT91C_MC_FMR)
long 0xfffffd44 0xa0008000  # watchdog disable (AT91C_WDTC_WDMR)
long 0xfffffc20 0xa0000601  # enable main oscillator (AT91C_PMC_MOR)
sleep 100
long 0xfffffc2c 0x00480a0e  # set PLL register (AT91C_PMC_PLLR)
sleep 200
long 0xfffffc30 0x7         # set master clock to PLL (AT91C_PMC_MCKR)
sleep 100
long 0xfffffd08 0xa5000401  # enable user reset (AT91C_RSTC_RMR)
