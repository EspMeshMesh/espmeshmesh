#pragma once
#include <memory>
#include <cstdint>
#include <string>
#include "meshsocket.h"

namespace espmeshmesh {
class WifiDrv;

class EspMeshMesh {
public:
  enum NodeType { 
    ESPMESH_NODE_TYPE_BACKBONE = 0, 
    ESPMESH_NODE_TYPE_COORDINATOR = 1, 
    ESPMESH_NODE_TYPE_EDGE = 2 
  };
  struct SetupUart {
    uint32_t baudRate;
    uint16_t txBuffer;
    uint16_t rxBuffer;
  };
  struct SetupWifi {
    std::string interface;
    uint8_t channel;
    uint8_t txPower;
  };
  struct SetupConfig {
    std::string hostname;
    SetupWifi wifi;
    SetupUart uart;
    EspMeshMesh::NodeType nodeType;
    std::string fwVersion;
    std::string compileTime;
  };
public:
  static EspMeshMesh *singleton;
  static EspMeshMesh *getInstance();
public:
  EspMeshMesh();
  ~EspMeshMesh();
  EspMeshMesh(const EspMeshMesh &) = delete;
  EspMeshMesh &operator=(const EspMeshMesh &) = delete;
  void setup(SetupConfig *config);
  void loop();
  void dumpConfig();
  void shutdown();
  bool teardown();
public:
  WifiDrv *getWifiDriver() const;
public:
  bool isCoordinator() const;
  bool isBackbone() const;
  bool isEdge() const;
  NodeType nodeType() const;
public:
  const std::string &hostname() const;
  const std::string &fwVersion() const;
  const std::string &compileTime() const;
  const std::string libVersion() const;
public:
  void uartSendData(const uint8_t *buff, uint16_t len);
  void broadcastSendData(const uint8_t *buff, uint16_t len);
  void broadcastSendData(const uint8_t *buff, uint16_t len, uint16_t port);
  void unicastSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint16_t port = 0);
  void multipathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen, uint32_t *path);
  void commandReply(const uint8_t *buff, uint16_t len);
private:
  class Impl;
  std::unique_ptr<Impl> mPimpl;
  friend class MeshSocket;
};

}  // namespace espmeshmesh
