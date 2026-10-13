// Minimal Arduino stand-in so pure-logic modules compile on a PC.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <algorithm>
using std::max;
using std::min;

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

template <typename T, typename L, typename H>
inline T constrain(T v, L lo, H hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }
