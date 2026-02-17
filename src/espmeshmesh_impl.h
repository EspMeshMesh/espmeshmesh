#pragma once
#include "espmeshmesh.h"
#include "meshaddress.h"
#include "packetbuf.h"
#include "discovery.h"
#include "broadcast2.h"
#include "memringbuffer.h"
#include "log.h"
#include "defines.h"

#include <string>

namespace espmeshmesh {

typedef std::function<int8_t(const uint8_t *data, uint16_t len, uint32_t from)> HandleFrameCbFn;
typedef void (*EspHomeDataReceivedCbFn)(uint16_t, uint8_t *, uint16_t);

class Uart;
class WifiDrv;
class Broadcast;
class Broadcast2;
class Unicast;
class MultiPath;

#ifdef ESPMESH_STARPATH_ENABLED
class StarPathProtocol;
#endif
class PoliteBroadcastProtocol;
class ConnectedPath;

class MeshSocket;

#define HANDLE_UART_OK 0
#define HANDLE_UART_ERROR 1
#define FRAME_NOT_HANDLED -1

class EspMeshMesh::Impl {
public:
  ConnectedPath *getConnectedPath() const { return mConnectedPath; }
  WifiDrv *getWifiDriver() const { return mWifiDrv; }
public:
  struct SetupUart {
    uint16_t baudRate;
    uint16_t txBuffer;
    uint16_t rxBuffer;
  };
  struct SetupConfig {
    std::string hostname;
    uint8_t channel;
    uint8_t txPower;
    SetupUart uart;
    EspMeshMesh::NodeType nodeType;
    std::string fwVersion;
    std::string compileTime;
  };
public:
  Impl();
  
  void pre_setup();
  void setup(SetupConfig *config);
  void setAesPassword(std::string password);
  void dump_config();
  void loop();
  void shutdown();
  bool teardown();
public:
  const std::string libVersion() const;
  const std::string &fwVersion() const { return mFwVersion; }
  const std::string &hostname() const { return mHostName; }
  const std::string &compileTime() const { return mCompileTime; }
  const NodeType nodeType() const { return mNodeType; }
  const bool isCoordinator() const { return mNodeType == ESPMESH_NODE_TYPE_COORDINATOR; }
  const bool isBackbone() const { return mNodeType == ESPMESH_NODE_TYPE_BACKBONE; }
  const bool isEdge() const { return mNodeType == ESPMESH_NODE_TYPE_EDGE; }
 public:
  void setLockdownMode(bool active);
  void commandReply(const uint8_t *buff, uint16_t len);

  MeshAddress::DataSrc lastCommandSourceProtocol() const { return mFromAddress.sourceProtocol; }
  int16_t lastPacketRssi() const { return mRssiHandle; }
  const MeshAddress &lastFromAddress() const { return mFromAddress; }

  /*
   * @brief Send data using framed uart protocol.
   * @param buff Data to send
   * @param len Length of data to send
   */
  void uartSendData(const uint8_t *buff, uint16_t len);
  
  /**
   * @brief Send data using broadcast2 protocol with port selector.
   * @param buff Data to send
   * @param len Length of data to send
   * @param port Port to send data to. Data will be received only from callbacks registered with this port.
   */
  void broadcastSendData(const uint8_t *buff, uint16_t len);
  void broadcastSendData(const uint8_t *buff, uint16_t len, uint16_t port);
  bool broadCastBindPort(uint16_t port, ProtocolReceiveHandler handler);
  /**
   * @brief Send data using unicast protocol.
   * @param buff Data to send
   * @param len Length of data to send
   * @param addr Address to send data to
   * @param port Port to send data to. Data will be received only from callbacks registered with this port.
   */
  void unicastSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint16_t port = 0);
  void multipathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen, uint32_t *path);

 private:
  void handleUartFrame(const uint8_t *data, uint16_t length);
  void handleFrame(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi=0);
  void replyHandleFrame(const uint8_t *buf, uint16_t len, const MeshAddress &from, int16_t rssi = 0);

 private:
  void sendLog(int level, const char *tag, const char *payload, size_t payload_len);

 private:
  uint32_t mLogDestination{0};
  uint32_t mFilterGroups{0};

private:
  Uart *mUart = nullptr;
  WifiDrv *mWifiDrv = nullptr;
  PacketBuf *packetbuf = nullptr;
  Broadcast *broadcast = nullptr;
  Broadcast2 *broadcast2 = nullptr;
  Unicast *unicast = nullptr;
  MultiPath *multipath = nullptr;
#ifdef ESPMESH_STARPATH_ENABLED
  StarPathProtocol *starpath = nullptr;
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  PoliteBroadcastProtocol *mPoliteBroadcast = nullptr;
#endif
  uint32_t mBroadcastFromAddress = 0;
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  uint32_t mPoliteFromAddress = 0;
#endif
  ConnectedPath *mConnectedPath = nullptr;

  MeshAddress mFromAddress;
  Discovery mDiscovery;

private:
  // Work around to send a dummy packet to the network to enable esp8266 wifi radio
  bool mWorkAround{false};
  uint8_t mTeardownPhase = 0;
  // Deadline to delay the teardown
  uint32_t mTeardownDeadline = 0;
  // Elapsed time for stats
  uint32_t mElapsed1 = 0;
  // RSSI for handle frame
  int16_t mRssiHandle = 0;
  // Firmware version
  std::string mFwVersion;
  // Compile time
  std::string mCompileTime;
  // Hostname
  std::string mHostName;
  // Node type
  NodeType mNodeType = ESPMESH_NODE_TYPE_BACKBONE;
 public:
  void addHandleFrameCb(PacketFrameHandler cb) { mHandleFrameCbs.push_back(cb); }

 private:
  std::list<PacketFrameHandler> mHandleFrameCbs;

 public:
  void setLogCb(LogCbFn cb) { setLibLogCb(cb); }

 public:
  friend class MeshSocket;
  friend class Uart;
};

}  // namespace espmeshmesh