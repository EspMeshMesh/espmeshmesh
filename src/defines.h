#pragma once
#include <cstdint>

#define USE_MULTIPATH_PROTOCOL
#define USE_POLITE_BROADCAST_PROTOCOL
#define USE_CONNECTED_PROTOCOL

#ifdef IDF_VER
uint32_t millis();
#endif

uint32_t random_uint32();
