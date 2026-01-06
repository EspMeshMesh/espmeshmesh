#pragma once

#include <cstdint>
#include <functional>
#include <list>
#include <cstring>

#include "packetbuf.h"
#include "recvdups.h"
#include "memringbuffer.h"

#include <cstdint>
#include <functional>

#define CONNPATH_FLAG_REVERSEDIR 0x80
#define CONNPATH_FLAG_RETRANSMIT_MASK 0x0F
#define CONNPATH_INVALID_ADDRESS 0xFFFFFFFF
#define CONNPATH_MAX_CONNECTIONS 0x10

namespace espmeshmesh {

typedef std::function<void(void *arg, const uint8_t *data, uint16_t size, uint8_t connid)> ConnectedPathReceiveHandler;
typedef std::function<void(void *arg)> ConnectedPathDisconnectHandler;
typedef std::function<void(void *arg, uint32_t from, uint16_t handle)> ConnectedPathNewConnectionHandler;

struct ConnectedPathHeaderSt {
  uint8_t protocol;       // ConnectedPath protocol identifier
  uint8_t subprotocol;    // Subprotocol identifier (CONNPATH_SEND_DATA, CONNPATH_INVALID_HANDLE, etc.)
  uint16_t sourceHandle;  // Handle for  this connection on the source
  uint16_t flags;         // Flags (CONNPATH_FLAG_REVERSEDIR, CONNPATH_FLAG_RETRANSMIT_MASK, etc.)
  uint16_t seqno;         // Sequence number of the packet this must be incremented for each packet sent
  uint16_t dataLength;    // Payload length
} __attribute__((packed));
typedef struct ConnectedPathHeaderSt ConnectedPathHeader_t;

struct ConnectedPathBindedPortSt {
  ConnectedPathNewConnectionHandler handler;
  void *arg;
  uint16_t port;
};
typedef ConnectedPathBindedPortSt ConnectedPathBindedPort_t;

struct ConnectedPathConnections {
  uint8_t isInvalid : 1;
  uint8_t isOperative : 1;
  uint16_t duplicatePacketCount;
  uint16_t sourceHandle;
  uint16_t destHandle;
  uint32_t sourceAddr;
  uint32_t destAddr;
  uint32_t lastTime;
  ConnectedPathReceiveHandler receive;
  ConnectedPathDisconnectHandler disconnect;
  void *arg;
};

struct ConnectedPathOutputBufferHeader {
  uint32_t pkttime;
  uint16_t forward : 1;
  uint8_t connId : 7;
  uint16_t subProtocol : 4;
  uint16_t dataSize : 12;
};

constexpr uint32_t CONNPATH_COORDINATOR_ADDRESS = 0x00000000;

class ConnectedPathPacket : public RadioPacket {
 public:
  explicit ConnectedPathPacket(PacketBufProtocol *owner, SentStatusHandler cb) : RadioPacket(owner, cb) {}

 public:
  virtual void allocClearData(uint16_t size);

 public:
  const ConnectedPathHeader_t *getHeader() const { return (const ConnectedPathHeader_t *) clearData(); }
  ConnectedPathHeader_t *getHeader() { return (ConnectedPathHeader_t *) clearData(); }
  void setPayload(const uint8_t *payoad);
  uint8_t *getPayload() { return clearData() + sizeof(ConnectedPathHeaderSt); }
  void setTarget(uint32_t target, uint16_t handle);
  uint32_t getTarget() const { return mTarget; }
  uint16_t getHandle() const { return getHeader()->sourceHandle; }

 private:
  uint32_t mTarget = 0;
};

class EspMeshMesh;
class ConnectedPath: public PacketBufProtocol {
 public:
  ConnectedPath(EspMeshMesh *meshmesh, PacketBuf *packetbuf);
  void loop(void) override;
  void shutdown(void) override;
  bool teardown(void) override;
  bool sendRawRadioPacket(ConnectedPathPacket *pkt);
  bool sendRadioPacket(ConnectedPathPacket *pkt, bool forward, bool initHeader);
  void sendRadioToConnection(uint8_t subprot, uint8_t connid, bool forward, uint16_t datasize, const uint8_t *data);
  void enqueueRadioDataToSource(const uint8_t *data, uint16_t size, uint32_t from, uint16_t handle);
  void closeConnection_(uint8_t connid);
  void closeConnection(uint32_t from, uint16_t handle);
  void closeAllConnections();
  uint8_t receiveUartPacket(const uint8_t *data, uint16_t size);

  void radioPacketRecv(uint8_t *p, uint16_t size, uint32_t f, int16_t r) override;
  void radioPacketSent(uint8_t status, RadioPacket *pkt) override;

  void setReceiveCallback(ConnectedPathReceiveHandler recvCb, ConnectedPathDisconnectHandler discCb, void *arg,
                          uint32_t from, uint16_t handle);
  void bindPort(ConnectedPathNewConnectionHandler h, void *arg, uint16_t port);
  void unbindPort(uint16_t port);
  bool isConnectionActive(uint32_t from, uint16_t handle) const;

 private:
  void radioPacketError(uint32_t address, uint16_t handle, uint8_t subprot);
  void duplicatePacketStats(uint32_t address, uint16_t handle, uint16_t seqno);
  void openConnection(uint32_t from, uint16_t handle, uint16_t datasize, const uint8_t *data);
  void openConnectionNack(uint32_t from, uint16_t handle);
  void openConnectionAck(uint32_t from, uint16_t handle);
  void disconnect(uint32_t from, uint16_t handle);
  void sendData(const uint8_t *buffer, uint16_t size, uint32_t source, uint16_t handle);
  void sendDataNack(uint32_t from, uint16_t handle);

 private:
  void connectionSetInvalid(uint8_t i) {
    if (i < CONNPATH_MAX_CONNECTIONS) {
      auto conn = mConnectsions + i;
      memset((void *) conn, 0, sizeof(ConnectedPathConnections));
      conn->isInvalid = true;
      conn->sourceAddr = CONNPATH_INVALID_ADDRESS;
    }
  }
  void connectionSetInoperative(uint8_t index) {
    if (index < CONNPATH_MAX_CONNECTIONS && mConnectsions[index].isOperative) {
      mConnectsions[index].isOperative = false;
      mConnectsions[index].receive = nullptr;
      mConnectsions[index].disconnect = nullptr;
      mConnectionInoperativeCount++;
    }
  }
  uint8_t connectionGetFirstInvalid() {
    uint8_t i;
    for (i = 0; i < CONNPATH_MAX_CONNECTIONS; i++)
      if (mConnectsions[i].isInvalid)
        break;
    return i;
  }

 private:
  const ConnectedPathConnections *findConnection(uint32_t from, uint16_t handle) const;
  ConnectedPathConnections *findConnection(uint32_t from, uint16_t handle);

  uint8_t findConnection(uint32_t from, uint16_t handle, bool &forward, uint32_t &otherAddress, uint16_t &otherHandle);
  uint8_t findConnectionIndex(uint32_t from, uint16_t handle, bool *forward);
  uint8_t findConnectionPeer(uint8_t connIdx, bool forward, uint32_t &peerAddress, uint16_t &peerHandle);

  void sendUartPacket(uint8_t command, uint16_t handle, const uint8_t *data, uint16_t size);
  ConnectedPathPacket *createPacket(uint8_t subprot, uint16_t size, uint32_t to, uint16_t handle,
                                    const uint8_t *payload);
  bool sendPacket(uint8_t subprot, uint8_t connid, bool forward, uint16_t size, const uint8_t *data);
  void sendImmediatePacket(uint8_t subprot, uint32_t to, uint16_t handle, uint16_t size, const uint8_t *data);
  void debugConnection() const;

 private:
 EspMeshMesh *mMeshMesh;
  uint16_t mLastSequenceNum = 0;
  RecvDups mRecvDups;
  ConnectedPathConnections mConnectsions[CONNPATH_MAX_CONNECTIONS];
  uint8_t mConnectionInoperativeCount = 0;
  uint32_t mConnectionsCheckTime = 0;
  std::list<ConnectedPathBindedPort_t> mBindedPorts;
  uint16_t mNextHandle = 1;

private:
  uint8_t *mOutputBuffer;
  uint16_t mOutputBufferIndex;
  uint32_t mOutputBufferDeadline = 0;
  uint8_t mOutputBufferConnId = 0;

private:
  void sendOutputBufferToRadio(uint8_t connid, bool forward, uint16_t datasize, const uint8_t *data);
  void sendOutputBuffer();
  void pushOutputBuffer(const uint8_t *data, uint16_t size, uint8_t connid);
  void clearOutputBuffer();

};

}  // namespace espmeshmesh

