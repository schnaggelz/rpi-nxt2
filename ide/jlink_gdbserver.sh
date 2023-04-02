#!/bin/bash

# Assuming soft-link for `JLinkGDBServerCLExe`

LOCAL_DIR=$(dirname $(readlink -e "$0"))

echo Starting GDB server in "$LOCAL_DIR" ...
gnome-terminal --hide-menubar --title jlink-gdbsvr --geometry 80x30 -- bash -l -c \
        "jlink-gdbserver -device AT91SAM7S256 -endian little -if JTAG -speed 30 -ir -vd -notimeout -halt -xc ${LOCAL_DIR}/gdb/jlink_init.gdb"
echo ... done
