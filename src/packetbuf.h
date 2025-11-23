#pragma once
#include "defines.h"
#include "meshaddress.h"

#ifdef IDF_VER
#include <esp_wifi.h>
typedef struct _event_ {
  uint32_t _tevent;
  uint32_t _tparam;
} os_event_t;
#endif
#ifdef ESP8266
#include <osapi.h>
#include <user_interface.h>
#include <user_config.h>
#endif

#include <list>
#include <map>

#include <functional>

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

#include <cstdint>

namespace espmeshmesh {

class PacketBuf;

#ifdef USE_ESP32
#define RxPacket wifi_promiscuous_pkt_t
#else
struct RxControl {
  signed rssi : 8;
  unsigned rate : 4;
  unsigned is_group : 1;
  unsigned : 1;
  unsigned sig_mode : 2;
  unsigned sig_len : 12;
  unsigned damatch0 : 1;
  unsigned damatch1 : 1;
  unsigned bssidmatch0 : 1;
  unsigned bssidmatch1 : 1;
  unsigned MCS : 7;
  unsigned CWB : 1;
  unsigned HT_length : 16;
  unsigned Smoothing : 1;
  unsigned Not_Sounding : 1;
  unsigned : 1;
  unsigned Aggregation : 1;
  unsigned STBC : 2;
  unsigned FEC_CODING : 1;
  unsigned SGI : 1;
  unsigned rxend_state : 8;
  unsigned ampdu_cnt : 8;
  unsigned channel : 4;
  unsigned : 12;
};

typedef struct RxPacket_st {
  struct RxControl rx_ctrl;
  uint8_t payload[];
} RxPacket;
#endif

struct pktbuf_recvTask_packet_st {
  uint32_t length;
  uint8_t *data;
  int16_t rssi;
};

typedef struct pktbuf_recvTask_packet_st pktbuf_recvTask_packet_t;

#define PKT_SEND_OK 0
#define PKT_SEND_ERR 1

#define PACKETBUF_TASK_PRIO 2
#define PACKETBUF_TASK_QUEUE_LEN 12

class RadioPacket;
class PacketBufProtocol;

#define PACKETBUF_80211_SIZE 24

class RadioPacket {
 public:
  explicit RadioPacket(PacketBufProtocol *owner, SentStatusHandler cb) {
    this->mOwner = owner;
    this->mCallback = std::move(cb);
  }
  virtual ~RadioPacket();
  bool isAutoDelete() const { return mAutoDelete; }
  bool isBroadcast() const { return mIsBroadcast; }
  void setAutoDelete(bool autodel) { mAutoDelete = autodel; }

  void fromRawData(const uint8_t *buf, uint16_t size);

 public:
  virtual void allocClearData(uint16_t size);
  void allocAndCopyClearData(uint8_t *data, uint16_t size);
  uint16_t clearDataSize() const { return mClearDataSize; }
  uint8_t *clearData() { return mClearData; }
  const uint8_t *clearData() const { return mClearData; }
  bool encryptClearData();

 public:
  uint16_t encryptedDataSize() const { return mEncryptedDataSize; }
  uint8_t *encryptedData() const { return mEncryptedData; }

  void reportSentStatus(uint8_t status, RadioPacket *data);
  void setCallback(SentStatusHandler cb) { this->mCallback = std::move(cb); }
  SentStatusHandler getCallback() { return this->mCallback; }
  void callCallback(uint8_t status, RadioPacket *data) {
    if (this->mCallback) {
      this->mCallback(status, data);
    }
  }


  void sendFreedom();

 public:
  // pktbuf_header_t *header() { return (pktbuf_header_t *)mEncryptedData; }
  uint8_t *ptr80211() const { return (uint8_t *) mEncryptedData; }
  uint16_t len80211() const { return mEncryptedDataSize + PACKETBUF_80211_SIZE; }
  void fill80211(uint8_t *targetId, uint8_t *pktbufNodeIdPtr);
  uint32_t target8211() const;
  uint8_t *ptrData() const { return (uint8_t *) (mEncryptedData + PACKETBUF_80211_SIZE); }

 protected:
  void setIsBroadcast() { mIsBroadcast = true; }

 private:
  SentStatusHandler mCallback{nullptr};
  PacketBufProtocol *mOwner{nullptr};
  // Full encrypted 802.11 packet
  uint8_t *mEncryptedData = nullptr;
  // Size of  encrypted data
  uint16_t mEncryptedDataSize = 0;
  // Clear payload that will be encrypted in finnal radio packet
  uint8_t *mClearData = nullptr;
  // Size of clear payload data
  uint16_t mClearDataSize = 0;
  // Autodelete packet after been sent
  bool mAutoDelete = true;
  // Is a broadcast packet
  bool mIsBroadcast = false;
};

/**
 * @brief PacketBufProtocol base class for all protocols
 */
class PacketBufProtocol {
public:
 PacketBufProtocol(PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr, MeshAddress::DataSrc protocol=MeshAddress::SRC_NONE);
 virtual ~PacketBufProtocol() {}
 virtual void setup() {};
 virtual void loop() {};

 MeshAddress::DataSrc protocolType() const { return mProtocolType; }
 bool isPortAvailable(uint16_t port) const;
 bool bindPort(uint16_t port, ReceiveHandler handler);
 void unbindPort(uint16_t port);
 void callReceiveHandler(const uint8_t *payload, uint16_t size,const MeshAddress &from, int16_t rssi);

 virtual void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) = 0;
 virtual void radioPacketSent(uint8_t status, RadioPacket *pkt) { pkt->callCallback(status, pkt); };

protected:
 PacketBuf *mPacketBuf{nullptr};
 std::map<uint16_t, ReceiveHandler> mBindedPorts;
 MeshAddress::DataSrc mProtocolType = MeshAddress::SRC_NONE;
};

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
  uint8_t send(RadioPacket *pkt);
  void rawRecv(RxPacket *pkt);
  void setup(const uint8_t *aeskey, int aeskeylen);
  void setRecvHandler(MeshAddress::DataSrc protocol, PacketBufProtocol *handler) { this->mRecvHandler[protocol] = handler; }
#ifdef IDF_VER
  void loop();
#endif
 private:
  void freedomCallback(uint8_t status);
#ifdef IDF_VER
  static void wifiTxDoneCb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus);
#else
  static void freedomCallback_cb(uint8_t status);
  void recvTask(os_event_t *events);
#endif
  void recvTask(uint32_t index);

 public:
  void setLockdownMode(bool active) { isLockdownModeActive = active; }

 private:
  bool isLockdownModeActive = false;

 private:
  RadioPacket *pktbufSent = nullptr;
  std::list<RadioPacket *> mPacketQueue;

#ifdef IDF_VER
  QueueHandle_t mRecvQueue;
#endif
  uint32_t pktbufNodeId = 0;
  uint8_t *pktbufNodeIdPtr = nullptr;
  uint16_t lastpktLen = 0;

 private:
#ifndef IDF_VER
  os_event_t pktbufRecvTaskQueue[PACKETBUF_TASK_QUEUE_LEN];
#endif
  pktbuf_recvTask_packet_t pktbufRecvTaskPacket[PACKETBUF_TASK_QUEUE_LEN];
  uint32_t pktbufRecvTaskIndex;

 private:
  std::map<MeshAddress::DataSrc, PacketBufProtocol *> mRecvHandler;
};

}  // namespace espmeshmesh
