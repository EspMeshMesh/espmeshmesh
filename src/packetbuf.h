#pragma once
#include "defines.h"
#include "meshaddress.h"

#include <list>
#include <map>
#include <functional>
#include <cstdint>

/* -------------------------------------------------------
 * 0x00 | 2 | Frame control
 * 0x02 | 2 | Duration ID
 * 0x04 | 6 | Address1
 * 0x0A | 6 | Address2
 * 0x10 | 6 | Address3
 * 0x16 | 2 | Sequence control
 * -------------------------------------------------------
 * Address1 = FF FF FF FF FF FF
 * Address2 = FF 7X FF FF FF FF
 * Address3 = FF FF FF FF FF FF
------------------------------------------------------- */

namespace espmeshmesh {

class PacketBuf;
class RadioPacket;
class PacketBufProtocol;
class WifiDrv;

/**
 * @brief PacketBuf 802.11 packet buffer implementation
 */
class PacketBuf {
public:
  static PacketBuf *singleton;
  static PacketBuf *getInstance();

public:
  uint32_t nodeId() const { return pktbufNodeId; }
  uint8_t *nodeIdPtr() const { return (uint8_t *) &pktbufNodeId; }

public:
  bool sendBusy() { return pktbufSent != nullptr; }
  void send(RadioPacket *pkt);
  void setup(const uint8_t *aeskey);
  void setRecvHandler(MeshAddress::DataSrc protocol, PacketBufProtocol *handler) { this->mRecvHandler[protocol] = handler; }
 private:
  void injectFrameCallback(uint8_t status);
  void captureFrameCallback(const uint8_t *data, uint16_t len, int16_t rssi);

 public:
  void setLockdownMode(bool active) { isLockdownModeActive = active; }

 private:
  bool isLockdownModeActive = false;

 private:
  RadioPacket *pktbufSent = nullptr;
  std::list<RadioPacket *> mPacketQueue;

  uint32_t pktbufNodeId = 0;
  uint8_t *pktbufNodeIdPtr = nullptr;
  uint16_t lastpktLen = 0;

 private:


private:
  WifiDrv *mWifiDrv = nullptr;
  std::map<MeshAddress::DataSrc, PacketBufProtocol *> mRecvHandler;
};

}  // namespace espmeshmesh
