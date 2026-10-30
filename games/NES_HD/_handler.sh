#!/bin/sh
#
# MiSTer Frontier -- NES HD core handler.
#
# Master_Daemon spawns this whenever /tmp/CORENAME is "NES_HD" (the core's own
# CONF_STR setname, so it never collides with the stock NES core). It starts the
# ARM half of the HD pack route (games/NES_HD/NES_HD), which idles until the core
# publishes a loaded pack. Master_Daemon kills it on the next core change.

GAMEDIR="/media/fat/games/NES_HD"
LOGDIR="/media/fat/logs/NES_HD"
BINARY="NES_HD"

cd "$GAMEDIR" || exit 1
mkdir -p "$LOGDIR" "$GAMEDIR/HD_Packs"

[ -x "./$BINARY" ] || chmod +x "./$BINARY" 2>/dev/null

# FPGA settle time, as every hybrid handler does
sleep 1

# Rotate immediately before the exec that recreates the log (see the handler contract)
mv -f "$LOGDIR/hdarm.log" "$LOGDIR/hdarm.prev.log" 2>/dev/null
exec taskset 0x03 "./$BINARY" > "$LOGDIR/hdarm.log" 2>&1
