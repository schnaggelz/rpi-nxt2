set FLASH::ForceUnlockBeforeWrite 1
send_file {Flash} "~/Develop/rpi-nxt2/build/nxt_bios.bin" 0x100000 0
