#pragma once
#include "defines.h"

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

#ifdef USE_ESP32
extern "C" {
// typedef void (* wifi_tx_done_cb_t)(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus);
// esp_err_t esp_wifi_set_tx_done_cb(wifi_tx_done_cb_t cb);
}
#endif

#include <cstdint>

namespace espmeshmesh {

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

#define PROTOCOL_BROADCAST    1
#define PROTOCOL_UNICAST      2
#define PROTOCOL_PREROUTED    3
#define PROTOCOL_MULTIPATH    4
#define PROTOCOL_POLITEBRD    5
#define PROTOCOL_BROADCAST_V2 6
#define PROTOCOL_CONNPATH     7
#define PROTOCOL_LAST         8

#define PKT_SEND_OK 0
#define PKT_SEND_ERR 1

#define PACKETBUF_TASK_PRIO 2
#define PACKETBUF_TASK_QUEUE_LEN 12

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
typedef void (*pktbufSentCbFn)(void *arg, uint8_t status, RadioPacket *data);

class Broadcast;
class Broadcast2;
class Unicast;
#ifdef USE_MULTIPATH_PROTOCOL
class MultiPath;
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
class PoliteBroadcastProtocol;
#endif
#ifdef USE_CONNECTED_PROTOCOL
class ConnectedPath;
#endif

#define PACKETBUF_80211_SIZE 24

class RadioPacket {
 public:
  explicit RadioPacket(pktbufSentCbFn cb, void *arg) : mCallback(cb), mCallbackArg(arg) {}
  virtual ~RadioPacket();
  bool isAutoDelete() const { return mAutoDelete; }
  bool isBroadcast() const { return mIsBroadcast; }
  void setAutoDelete(bool autodel) { mAutoDelete = autodel; }
  void setCallback(pktbufSentCbFn cb, void *arg) {
    mCallback = cb;
    mCallbackArg = arg;
  }
  void fromRawData(uint8_t *buf, uint16_t size);

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
  void callCallback(uint8_t status, RadioPacket *data);
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
  pktbufSentCbFn mCallback = nullptr;
  void *mCallbackArg = nullptr;
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

class PacketBuf {
 public:
  static PacketBuf *singleton;
  static PacketBuf *getInstance();

 public:
  Broadcast *getBroadcast(void) const { return broadcast; }
  void setBroadcast(Broadcast *b) { broadcast = b; }
  Broadcast2 *getBroadcast2(void) const { return broadcast2; }
  void setBroadcast2(Broadcast2 *b) { broadcast2 = b; }
  Unicast *getUnicast(void) const { return unicast; }
  void setUnicast(Unicast *u) { unicast = u; }
#ifdef USE_MULTIPATH_PROTOCOL
  void setMultiPath(MultiPath *m) { multipath = m; }
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  void setPoliteBroadcast(PoliteBroadcastProtocol *protocol) { mPoliteBroadcast = protocol; }
#endif
#ifdef USE_CONNECTED_PROTOCOL
  void setConnectedPath(ConnectedPath *protocol) { mConnectedPath = protocol; }
#endif
  uint32_t nodeId() const { return pktbufNodeId; }
  uint8_t *nodeIdPtr() const { return (uint8_t *) &pktbufNodeId; }

 public:
  bool sendBusy() { return pktbufSent != nullptr; }
  uint8_t send(RadioPacket *pkt);
  void rawRecv(RxPacket *pkt);
  void setup(const uint8_t *aeskey, int aeskeylen);
#ifdef IDF_VER
  void loop();
#endif
 private:
  void freedomCallback(uint8_t status);
#ifdef IDF_VER
  void recvTask(uint32_t index);
  static void wifiTxDoneCb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus);
#else
  static void freedomCallback_cb(uint8_t status);
  static void recvTask_cb(os_event_t *events);
  void recvTask(os_event_t *events);
#endif

 public:
  void setLockdownMode(bool active) { isLockdownModeActive = active; }
 private:
  bool isLockdownModeActive = false;

 private:
  RadioPacket *pktbufSent = nullptr;
  std::list<RadioPacket *> mPacketQueue;
  pktbufSentCbFn pktbufSentCb = nullptr;
  void *pktbufSentCbArgs = nullptr;
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
  Broadcast *broadcast = nullptr;
  Broadcast2 *broadcast2 = nullptr;
  Unicast *unicast = nullptr;
#ifdef USE_MULTIPATH_PROTOCOL
  MultiPath *multipath = nullptr;
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  PoliteBroadcastProtocol *mPoliteBroadcast = nullptr;
#endif
#ifdef USE_CONNECTED_PROTOCOL
  ConnectedPath *mConnectedPath = nullptr;
#endif
};

}  // namespace espmeshmesh

