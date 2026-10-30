#!/bin/sh
# Builds tools/audio/oggdecode.exe (Mesen2's stb_vorbis + Hermite resampler, see oggdecode.cpp).
#   sh tools/audio/build.sh
set -e
cd "$(dirname "$0")"
export PATH=/c/msys64/mingw64/bin:$PATH
g++ -O2 -std=c++17 -I. -Imesen -o oggdecode.exe oggdecode.cpp mesen/stb_vorbis.cpp mesen/HermiteResampler.cpp -static
echo "built tools/audio/oggdecode.exe"
