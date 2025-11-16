#pragma once
#include "defines.h"

#include "packetbuf.h"
#include "discovery.h"
#include "broadcast2.h"
#include "memringbuffer.h"
#include "log.h"

#include <string>

#ifdef ESP8266
#include <HardwareSerial.h>
#endif  // ESP8266

#ifdef IDF_VER
#include <driver/uart.h>
#endif  // IDF_VER

namespace espmeshmesh {

typedef std::function<int8_t(const uint8_t *data, uint16_t len, uint32_t from)> HandleFrameCbFn;
typedef void (*EspHomeDataReceivedCbFn)(uint16_t, uint8_t *, uint16_t);

typedef enum { WAIT_START, WAIT_DATA, WAIT_ESCAPE, WAIT_CRC16_1, WAIT_CRC16_2 } RecvState;
typedef enum {
  CODE_DATA_START = 0xFE,
  CODE_DATA_START_CRC16 = 0xFD,
  CODE_DATA_END = 0xEF,
  CODE_DATA_ESCAPE = 0xEA
} SpcialByteCodes;

class Broadcast;
class Broadcast2;
class Unicast;
class MultiPath;
class PoliteBroadcastProtocol;
#ifdef USE_CONNECTED_PROTOCOL
class ConnectedPath;
#endif

class MeshSocket;

#define HANDLE_UART_OK 0
#define HANDLE_UART_ERROR 1
#define FRAME_NOT_HANDLED -1

typedef struct {
  const char *hostname;
  uint8_t channel;
  uint8_t txPower;
} EspMeshMeshSetupConfig;

class EspMeshMesh {
 public:
 public:
  static EspMeshMesh *singleton;
  static EspMeshMesh *getInstance();
#ifdef USE_CONNECTED_PROTOCOL
  ConnectedPath *getConnectedPath() const { return mConnectedPath; }
#endif
 public:
  EspMeshMesh(int baud_rate, int tx_buffer, int rx_buffer);
  void pre_setup();
#ifdef IDF_VER
  static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
  bool setupIdfWifiAP(const char *hostname, uint8_t channel, uint8_t txPower);
  bool setupIdfWifiStation(const char *hostname, uint8_t channel, uint8_t txPower);
#endif
  void setupWifi(const char *hostname, uint8_t channel, uint8_t txPower);
  void setup(EspMeshMeshSetupConfig *config);
  void setAesPassword(const char *password) { mAesPassword = password; }
  void dump_config();
  void loop();

 public:
  void setLockdownMode(bool active) { packetbuf->setLockdownMode(active); }
  static void wifiInitMacAddr(uint8_t index);
  void commandReply(const uint8_t *buff, uint16_t len);
  void uartSendData(const uint8_t *buff, uint16_t len);

  MeshAddress::DataSrc lastCommandSourceProtocol() const { return mFromAddress.sourceProtocol; }
  int16_t lastPacketRssi() const { return mRssiHandle; }
  const MeshAddress &lastFromAddress() const { return mFromAddress; }

  /**
   * @brief Send data using broadcast2 protocol with port selector.
   * @param buff Data to send
   * @param len Length of data to send
   * @param port Port to send data to. Data will be received only from callbacks registered with this port.
   */
  void broadcastSendData(const uint8_t *buff, uint16_t len);
  void broadcastSendData(const uint8_t *buff, uint16_t len, uint16_t port);
  /**
   * @brief Send data using unicast protocol.
   * @param buff Data to send
   * @param len Length of data to send
   * @param addr Address to send data to
   * @param port Port to send data to. Data will be received only from callbacks registered with this port.
   */
  void unicastSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint16_t port = 0);
  void multipathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen, uint32_t *path);
 public:
  static unsigned long elapsedMillis(unsigned long t2, unsigned long t1) {
    return t2 >= t1 ? t2 - t1 : (~(t1 - t2)) + 1;
  }

 private:
#ifdef IDF_VER
  void initIdfUart();
#endif
  void user_uart_recv_data(uint8_t byte);
  void flushUartTxBuffer();

 private:
  void handleFrame(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi=0);
  void replyHandleFrame(const uint8_t *buf, uint16_t len, const MeshAddress &from, int16_t rssi = 0);

 private:
#ifdef USE_CONNECTED_PROTOCOL
  static void onConnectedPathNewClientCb(void *arg, uint32_t from, uint16_t handle);
  void onConnectedPathNewClient(uint32_t from, uint16_t handle);
  static void onConnectedPathReceiveCb(void *arg, const uint8_t *data, uint16_t size, uint8_t connid);
  void onConnectedPathReceive(const uint8_t *data, uint16_t size, uint8_t connid);
#endif

  void sendLog(int level, const char *tag, const char *payload, size_t payload_len);

 private:
  uint32_t mLogDestination{0};
  uint32_t mFilterGroups{0};
  int mBaudRate{460800};
  int mTxBuffer;
  int mRxBuffer;

private:
  PacketBuf *packetbuf = nullptr;
  Broadcast *broadcast = nullptr;
  Broadcast2 *broadcast2 = nullptr;
  Unicast *unicast = nullptr;
  MultiPath *multipath = nullptr;
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  PoliteBroadcastProtocol *mPoliteBroadcast = nullptr;
#endif
  uint32_t mBroadcastFromAddress = 0;
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  uint32_t mPoliteFromAddress = 0;
#endif
#ifdef USE_CONNECTED_PROTOCOL
  ConnectedPath *mConnectedPath = nullptr;
  uint8_t mConnectionId = 0;
#endif

#ifdef ESP8266
  HardwareSerial *mHwSerial = nullptr;
#endif  // ESP8266

#ifdef IDF_VER
  uart_port_t mUartNum{UART_NUM_0};
#else
  int mUartNum{0};
#endif

  RecvState mRecvState = WAIT_START;
  uint8_t *mRecvBuffer = nullptr;
  uint16_t mRecvBufferPos = 0;
  MeshAddress mFromAddress;

  Discovery mDiscovery;

 private:
  // UartRingBuffer;
  MemRingBuffer mUartTxBuffer;
  // Use serial
  bool mUseSerial = false;
  // Elapsed time for stats
  uint32_t mElapsed1 = 0;
  // RSSI for handle frame
  int16_t mRssiHandle = 0;
  // Encryption password
  std::string mAesPassword;
#ifdef ESP8266
  bool mWorkAround{false};
#endif

 public:
  void addHandleFrameCb(PacketFrameHandler cb) { mHandleFrameCbs.push_back(cb); }

 private:
  std::list<PacketFrameHandler> mHandleFrameCbs;

 public:
  void setLogCb(LogCbFn cb) { setLibLogCb(cb); }

 public:
  friend class MeshSocket;
};

}  // namespace espmeshmesh
