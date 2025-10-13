#pragma once
#include "meshaddress.h"
#include <cstdint>
#include <functional>

namespace espmeshmesh {

#define PROTOCOL_BROADCAST 1
#define PROTOCOL_UNICAST 2
#define PROTOCOL_PREROUTED 3
#define PROTOCOL_MULTIPATH 4
#define PROTOCOL_POLITEBRD 5
#define PROTOCOL_BROADCAST_V2 6
#define PROTOCOL_CONNPATH 7
#define PROTOCOL_LAST 8

#define CMD_FLASH_GETMD5 0x01
#define CMD_FLASH_ERASE 0x02
#define CMD_FLASH_WRITE 0x03
#define CMD_FLASH_EBOOT 0x04
#define CMD_FLASH_PREPARE 0x05

#ifdef IDF_VER
uint32_t millis();
#endif

uint32_t random_uint32();

using namespace std::placeholders;


inline uint32_t uint32FromBuffer(const uint8_t *buffer, bool little = false) {
  if (little)
    return (uint32_t) ((buffer[3]) + (buffer[2] << 8) + (buffer[1] << 16) + (buffer[0] << 24));
  else
    return (uint32_t) ((buffer[0]) + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24));
}

inline uint16_t uint16FromBuffer(const uint8_t *buffer) { return (uint16_t) ((buffer[0]) + (buffer[1] << 8)); }

inline void uint32toBuffer(uint8_t *buffer, uint32_t value) {
  buffer[0] = (value & 0xFF);
  buffer[1] = ((value >> 8) & 0xFF);
  buffer[2] = ((value >> 16) & 0xFF);
  buffer[3] = ((value >> 24) & 0xFF);
}

inline void uint16toBuffer(uint8_t *buffer, uint16_t value) {
  buffer[0] = (value & 0xFF);
  buffer[1] = ((value >> 8) & 0xFF);
}


class RadioPacket;
class PacketBufProtocol;

typedef std::function<void(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi)> ReceiveHandler;

typedef std::function<int8_t(const uint8_t *data, uint16_t size, MeshAddress from, int16_t rssi)> PacketFrameHandler;

typedef std::function<void(int8_t status, RadioPacket *pkt)> SentStatusHandler;

}  // namespace espmeshmesh
