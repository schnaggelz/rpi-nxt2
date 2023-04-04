#!/bin/bash

# Assuming soft-link for `JLinkGDBServerCLExe`

LOCAL_DIR=$(dirname $(readlink -e "$0"))

echo Starting GDB server in "$LOCAL_DIR" ...

jlink-gdbserver -device AT91SAM7S256 -endian little -if JTAG -speed 30 -ir -vd -notimeout -halt -xc ${LOCAL_DIR}/gdb/jlink_init.gdb

echo ... done
