#!/bin/sh
# build_arm.sh - build the NES_HD ARM daemon (games/NES_HD/NES_HD) for the MiSTer's Cortex-A9.
#
#   sh src/build_arm.sh <output file>
#
# Needs arm-linux-gnueabihf-gcc (Debian: apt-get install gcc-arm-linux-gnueabihf). CI runs this in
# debian:bookworm. The binary is static, so it runs on the MiSTer's Buildroot without any library.
# -O3 and no position-independent code were measured fastest on the A9; --build-id=none keeps two
# builds of the same source byte-identical.
set -e
OUT="${1:?usage: build_arm.sh <output file>}"
SRC="$(cd "$(dirname "$0")" && pwd)"
CC=arm-linux-gnueabihf-gcc
F="-O3 -fno-pie -mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon -Wall"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
$CC $F -DHDARM_LIB -c -o "$TMP/hdarm4.o"     "$SRC/hdarm4.c"
$CC $F             -c -o "$TMP/daemon.o"     "$SRC/hdarm_daemon.c"
$CC $F             -c -o "$TMP/romfix_arm.o" "$SRC/romfix_arm.c"
$CC $F             -c -o "$TMP/aud_arm.o"    "$SRC/aud_arm.c"
$CC -static -no-pie -Wl,--build-id=none -o "$OUT" \
    "$TMP/daemon.o" "$TMP/hdarm4.o" "$TMP/romfix_arm.o" "$TMP/aud_arm.o" -lm -lpthread
[ -s "$OUT" ] || { echo "ERROR: $OUT was not produced" >&2; exit 1; }
ls -l "$OUT"
