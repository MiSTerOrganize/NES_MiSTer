// Shim for Mesen2's precompiled header so its stb_vorbis.cpp / HermiteResampler.cpp
// build unchanged outside the Mesen tree.
#pragma once
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <string>
using std::vector;
using std::string;
#ifndef __forceinline
#define __forceinline inline
#endif
