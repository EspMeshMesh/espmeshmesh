#include "connectedpath.h"
#ifdef USE_CONNECTED_PROTOCOL
#include "log.h"
#include "espmeshmesh.h"
#include "commands.h"

#define RES_OK 0
#define RES_ERROR 1
#define RES_INVALID_HANDLE 2
#define FORWARD2TXT(X) (X ? "-->" : "<--")
#define FORWARD true
#define REVERSE false

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.ConnectedPath";

#define CONNPATH_MAX_RETRANSMISSIONS 0x04

#define CONNPATH_INVALID_REQ 0x00
#define CONNPATH_OPEN_CONNECTION_REQ 0x01
#define CONNPATH_OPEN_CONNECTION_REP 0x02
#define CONNPATH_DISCONNECT_ACK 0x03
#define CONNPATH_SEND_DATA_NACK 0x04
#define CONNPATH_SEND_DATA 0x05
#define CONNPATH_OPEN_CONNECTION_ACK 0x06
#define CONNPATH_OPEN_CONNECTION_NACK 0x07
#define CONNPATH_DISCONNECT_REQ 0x08
//#define CONNPATH_SEND_DATA_ERROR 0x09
#define CONNPATH_CLEAR_CONNECTIONS 0x0A

#define CONN_EXISTS(X) (X < CONNPATH_MAX_CONNECTIONS)
#define CONN_OPERATIVE(X) (X < CONNPATH_MAX_CONNECTIONS && !mConnectsions[X].isInvalid && mConnectsions[X].isOperative)

void ConnectedPathPacket::allocClearData(uint16_t size) {
  RadioPacket::allocClearData(size + sizeof(ConnectedPathHeaderSt));
  getHeader()->dataLength = size;
}

void ConnectedPathPacket::setPayload(const uint8_t *payoad) {
  memcpy(clearData() + sizeof(ConnectedPathHeaderSt), payoad, getHeader()->dataLength);
}

void ConnectedPathPacket::setTarget(uint32_t target, uint16_t handle) {
  mTarget = target;
  if (clearData() != nullptr)
    getHeader()->sourceHandle = handle;
}

ConnectedPath::ConnectedPath(EspMeshMesh *meshmesh, PacketBuf *packetbuf): mMeshMesh(meshmesh), mPacketBuf(packetbuf), mRecvDups(), mRadioOutputBuffer(256) {
  mPacketBuf->setRecvHandler(PROTOCOL_CONNPATH, std::bind(&ConnectedPath::receiveRadioPacket, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

void ConnectedPath::setup(void) {
  mRadioOutputBuffer.resize(1024);
  memset((uint8_t *) mConnectsions, 0x0, sizeof(mConnectsions));
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++)
    connectionSetInvalid(i);
  mConnectionsCheckTime = millis();
#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERY_VERBOSE
  mRecvDups.setDebug(true);
#endif
}

void ConnectedPath::loop() {
  mRecvDups.loop();

  if (mRetransmitPacket != nullptr) {
    sendRawRadioPacket(mRetransmitPacket);
    mRetransmitPacket = nullptr;
  } else if (mRadioOutputBuffer.filledSpace() > 0 && mIsRadioBusy == false) {
    processOutputBuffer();
  }

  if (mConnectionInoperativeCount > 0 && mRadioOutputBuffer.filledSpace() == 0) {
    for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
      if (!mConnectsions[i].isInvalid && !mConnectsions[i].isOperative) {
        LIB_LOGD(TAG, "ConnectedPath::loop invalidate inoperative connection %d", i);
        connectionSetInvalid(i);
        mConnectionInoperativeCount--;
      }
    }
  }

  uint32_t now = millis();
  if (EspMeshMesh::elapsedMillis(now, mConnectionsCheckTime) > 120000) {
    mConnectionsCheckTime = now;
    for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
      if (mConnectsions[i].isOperative && EspMeshMesh::elapsedMillis(now, mConnectsions[i].lastTime) > 300000) {
        closeConnection_(i);
      }
    }
  }
}

uint8_t ConnectedPath::sendRawRadioPacket(ConnectedPathPacket *pkt) {
  LIB_LOGVV(TAG, "ConnectedPath::sendRawRadioPacket sending to %06X %d bytes with flags %d", pkt->getTarget(),
           pkt->clearDataSize(), pkt->getHeader()->flags);
  if (pkt->encryptClearData()) {
    mIsRadioBusy = true;
    uint32_t target = pkt->getTarget();
    pkt->fill80211((uint8_t *) &target, mPacketBuf->nodeIdPtr());
    uint8_t res = mPacketBuf->send(pkt);
    if (res == PKT_SEND_ERR)
      delete pkt;
    return res;
  } else {
    return PKT_SEND_ERR;
    delete pkt;
  }
}

uint8_t ConnectedPath::sendRadioPacket(ConnectedPathPacket *pkt, bool forward, bool initHeader) {
  ConnectedPathHeader_t *header = pkt->getHeader();
  // Fill protocol header...
  header->protocol = PROTOCOL_CONNPATH;
  // Optional fields
  if (initHeader) {
    // Add flags to this packet
    header->flags = forward ? 0x00 : CONNPATH_FLAG_REVERSEDIR;
    // If is an ACK i use the last seqno
    header->seqno = ++mLastSequenceNum;
  }
  // Set this class as destination sent callback for retransmisisons
  pkt->setCallback(radioPacketSentCb, this);

  return sendRawRadioPacket(pkt);
}

void ConnectedPath::enqueueRadioPacket(uint8_t subprot, uint8_t connid, bool forward, uint16_t datasize,
                                       const uint8_t *data) {
  if (!CONN_EXISTS(connid) || !mConnectsions[connid].isOperative) {
    return;
  }

  ConnectedPathOutputBufferHeader header;
  header.pkttime = millis();
  header.connId = connid;
  header.forward = forward ? 1 : 0;
  header.subProtocol = subprot;
  header.dataSize = datasize;
  mRadioOutputBuffer.pushData((uint8_t *) &header, sizeof(header));
  if (datasize > 0)
    mRadioOutputBuffer.pushData(data, datasize);

  if (subprot != CONNPATH_SEND_DATA) {
    LIB_LOGD(TAG, "ConnectedPath::enqueueRadioPacket prot %d connid %d size %d from %06X:%04X %s to %06X:%04X", subprot,
              connid, datasize, mConnectsions[connid].sourceAddr, mConnectsions[connid].sourceHandle, FORWARD2TXT(forward),
              mConnectsions[connid].destAddr, mConnectsions[connid].destHandle);
  }
}

void ConnectedPath::enqueueRadioDataTo(const uint8_t *data, uint16_t size, uint8_t connid, bool forward) {
  if (!CONN_EXISTS(connid) || !mConnectsions[connid].isOperative) {
    return;
  }

  ConnectedPathOutputBufferHeader header;
  header.pkttime = millis();
  header.connId = connid;
  header.forward = forward ? 1 : 0;
  header.subProtocol = CONNPATH_SEND_DATA;
  header.dataSize = size;
  mRadioOutputBuffer.pushData((uint8_t *) &header, sizeof(header));
  mRadioOutputBuffer.pushData(data, size);

  LIB_LOGVV(TAG, "ConnectedPath::enqueueRadioDataTo connid %d size %d from %06X:%04X %s to %06X:%04X", connid, size,
           mConnectsions[connid].sourceAddr, mConnectsions[connid].sourceHandle, FORWARD2TXT(forward),
           mConnectsions[connid].destAddr, mConnectsions[connid].destHandle);
}

void ConnectedPath::enqueueRadioDataToSource(const uint8_t *data, uint16_t size, uint32_t source,
                                             uint16_t sourceHandle) {
  bool direction;
  uint8_t connid = findConnectionIndex(source, sourceHandle, &direction);
  if (CONN_EXISTS(connid)) {
    direction = !direction;
    enqueueRadioDataTo(data, size, connid, direction);
  }

  // sendData(data, size, from, handle);
}

void ConnectedPath::closeConnection_(uint8_t connid) {
  sendPacket(CONNPATH_DISCONNECT_REQ, connid, REVERSE, 0, nullptr);
  sendPacket(CONNPATH_DISCONNECT_REQ, connid, FORWARD, 0, nullptr);
  connectionSetInoperative(connid);
}

void ConnectedPath::closeConnection(uint32_t from, uint16_t handle) {
  uint8_t connid = findConnectionIndex(from, handle, nullptr);
  if (connid < CONNPATH_MAX_CONNECTIONS)
    closeConnection_(connid);
}

void ConnectedPath::closeAllConnections() {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if (mConnectsions[i].sourceAddr != CONNPATH_INVALID_ADDRESS) {
      closeConnection_(i);
    }
  }
}

/**
 * @brief \ive a ConnectedPath packet from the UART and handle it using the correct action.
 *
 * @param data - The packet data to be processed.
 * @param size - The size of the packet data.
 * @return uint8_t - The result of the operation. 0 if successful, 1 if not.
 */
uint8_t ConnectedPath::receiveUartPacket(uint8_t *data, uint16_t size) {
  if (size >= sizeof(ConnectedPathHeaderSt)) {
    ConnectedPathHeader_t *header = (ConnectedPathHeader_t *) data;
    uint8_t *payload = data + sizeof(ConnectedPathHeaderSt);
    uint16_t payloadSize = header->dataLength;
    // LIB_LOGD(TAG, "ConnectedPath::receiveUartPacket size %d subp %d", size, header->subprotocol);

    if (header->subprotocol == CONNPATH_OPEN_CONNECTION_REQ) {
      openConnection(0, header->sourceHandle, payloadSize, payload);
    } else if (header->subprotocol == CONNPATH_SEND_DATA) {
      sendData(payload, payloadSize, 0, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_DISCONNECT_REQ) {
      disconnect(0, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_SEND_DATA_NACK) {
      sendDataNack(0, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_CLEAR_CONNECTIONS) {
      closeAllConnections();
      mRecvDups.clear();
      LIB_LOGI(TAG, "All connection and duplicates tables has been cleared");
    } else {
      LIB_LOGE(TAG, "ConnectedPath::receiveUartPacket unknow sub protocol %d", header->subprotocol);
    }
  }
  return HANDLE_UART_OK;
}

/**
 * @brief Receive a ConnectedPath packet from the radio and handle it using the correct action.
 * @param buf - The packet data to be processed.
 * @param size - The size of the packet data. Must be greater than sizeof(ConnectedPathHeaderSt).
 * @param source - The source address of the packet.
 * @param rssi - The RSSI of the packet.
 */
void ConnectedPath::receiveRadioPacket(uint8_t *data, uint16_t size, uint32_t source, int16_t rssi) {
  if (size >= sizeof(ConnectedPathHeaderSt)) {
    ConnectedPathHeader_t *header = (ConnectedPathHeader_t *) data;
    uint8_t *payload = data + sizeof(ConnectedPathHeader_t);
    uint16_t payloadSize = header->dataLength;
    if (header->subprotocol != CONNPATH_SEND_DATA) {
      LIB_LOGD(TAG, "ConnectedPath::receiveRadioPacket cmd %02X from %06X with seq %d data %d rssi %d", header->subprotocol,
               source, header->seqno, header->dataLength, rssi);
    }
    if (mRecvDups.checkDuplicateTable(source, header->sourceHandle, header->seqno)) {
      duplicatePacketStats(source, header->sourceHandle, header->seqno);
      LIB_LOGV(TAG, "ConnectedPath duplicated packet received from %06X:%02X with seq %d", source, header->sourceHandle,
               header->seqno);
      return;
    }

    if (header->subprotocol == CONNPATH_OPEN_CONNECTION_REQ) {
      openConnection(source, header->sourceHandle, payloadSize, payload);
    } else if (header->subprotocol == CONNPATH_OPEN_CONNECTION_NACK) {
      openConnectionNack(source, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_OPEN_CONNECTION_ACK) {
      openConnectionAck(source, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_SEND_DATA) {
      sendData(payload, payloadSize, source, header->sourceHandle);  // sendData(data, size, source);
    } else if (header->subprotocol == CONNPATH_DISCONNECT_REQ) {
      disconnect(source, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_SEND_DATA_NACK) {
      sendDataNack(source, header->sourceHandle);
    } else {
      LIB_LOGE(TAG, "ConnectedPath::receiveRadioPacket unknow sub protocol %d received", header->subprotocol);
    }
  } else {
    LIB_LOGE(TAG, "ConnectedPath::recv invalid size %d but required at least %d", size, sizeof(ConnectedPathHeaderSt));
  }
}

void ConnectedPath::setReceiveCallback(ConnectedPathReceiveHandler recvCb, ConnectedPathDisconnectHandler discCb,
                                       void *arg, uint32_t from, uint16_t handle) {
  ConnectedPathConnections *conn = findConnection(from, handle);
  if (conn != nullptr) {
    conn->receive = recvCb;
    conn->disconnect = discCb;
    conn->arg = arg;
  } else {
    LIB_LOGE(TAG, "ConnectedPath::setReceiveCallback %06X:%04X handle not found", from, handle);
  }
}

void ConnectedPath::bindPort(ConnectedPathNewConnectionHandler h, void *arg, uint16_t port) {
  LIB_LOGD(TAG, "ConnectedPath::bind port %d", port);
  ConnectedPathBindedPort_t newclient = {h, arg, port};
  mBindedPorts.push_back(newclient);
}

bool ConnectedPath::isConnectionActive(uint32_t from, uint16_t handle) const {
  return findConnection(from, handle) != nullptr;
}

void ConnectedPath::radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt) {
  ((ConnectedPath *) arg)->radioPacketSent(status, pkt);
}

void ConnectedPath::radioPacketSent(uint8_t status, RadioPacket *pkt) {
  if (status) {
    // Handle transmission error onyl with packets with clean data
    ConnectedPathPacket *oldpkt = (ConnectedPathPacket *) pkt;
    ConnectedPathHeader_t *header = oldpkt->getHeader();
    if (header != nullptr) {
      if ((header->flags & CONNPATH_FLAG_RETRANSMIT_MASK) < CONNPATH_MAX_RETRANSMISSIONS) {
        mRetransmitPacket = new ConnectedPathPacket(radioPacketSentCb, this);
        mRetransmitPacket->fromRawData(pkt->clearData(), pkt->clearDataSize());
        mRetransmitPacket->getHeader()->flags++;
        mRetransmitPacket->setTarget(oldpkt->getTarget(), oldpkt->getHandle());
        return;
      } else {
        radioPacketError(pkt->target8211(), header->sourceHandle, header->subprotocol);
        // FIXME: Signal error to packet creator
      }
    }
  }
  // Free Radio for next packet
  mIsRadioBusy = false;
}

void ConnectedPath::radioPacketError(uint32_t address, uint16_t handle, uint8_t subprot) {
  bool forward;
  uint8_t connid = findConnectionIndex(address, handle, &forward);
  if (CONN_EXISTS(connid)) {
    uint8_t _subprot = subprot == CONNPATH_OPEN_CONNECTION_REQ
                           ? CONNPATH_OPEN_CONNECTION_NACK
                           : (subprot == CONNPATH_SEND_DATA ? CONNPATH_SEND_DATA_NACK : CONNPATH_INVALID_REQ);

    if (_subprot != CONNPATH_INVALID_REQ)
      sendPacket(_subprot, connid, forward, 0, nullptr);
    connectionSetInoperative(connid);
    LIB_LOGD(TAG, "ConnectedPath::radioPacketError prot %d connid %d from %06X:%04X %s to %06X:%04X", subprot,
      connid, mConnectsions[connid].sourceAddr, mConnectsions[connid].sourceHandle, FORWARD2TXT(forward),
      mConnectsions[connid].destAddr, mConnectsions[connid].destHandle);
  } else {
    LIB_LOGE(TAG, "ConnectedPath::radioPacketError %06X:%04X handle for this packet is not found", address, handle);
  }
}

void ConnectedPath::duplicatePacketStats(uint32_t address, uint16_t handle, uint16_t seqno) {
  uint8_t connid = findConnectionIndex(address, handle, nullptr);
  if (CONN_EXISTS(connid)) {
    mConnectsions[connid].duplicatePacketCount++;
  }
}

void ConnectedPath::openConnection(uint32_t from, uint16_t handle, uint16_t datasize, uint8_t *data) {
  uint16_t port = uint16FromBuffer(data);
  uint8_t pathLen = data[2];

  LIB_LOGD(TAG, "ConnectedPath::openConnection from %06X:%04X port %d pathLen %d", from, handle, port, pathLen);
  if (datasize >= pathLen * sizeof(uint32_t) + 3) {
    uint8_t connid = connectionGetFirstInvalid();

    if (CONN_EXISTS(connid)) {
      ConnectedPathConnections *conn = mConnectsions + connid;
      uint32_t nextAddress = pathLen > 0 ? uint32FromBuffer(data + 3) : 0;
      conn->isInvalid = false;
      conn->isOperative = true;
      conn->sourceAddr = from;
      conn->sourceHandle = handle;
      conn->lastTime = millis();

      if (pathLen > 0) {
        uint8_t newPathLen = pathLen - 1;
        conn->destAddr = nextAddress;
        conn->destHandle = mNextHandle++;
        // Forrward OPEN_CONNECTION request
        LIB_LOGD(TAG, "ConnectedPath::openConnection req %06X:%04X[%02X]", conn->destAddr, conn->destHandle, connid);
        uint16_t newdatasize = newPathLen * sizeof(uint32_t) + 3;
        uint8_t *newdata = new uint8_t[newdatasize];
        uint16toBuffer(newdata, port);
        newdata[2] = newPathLen;
        if (newPathLen)
          memcpy(newdata + 3, data + 7, newPathLen * sizeof(uint32_t));
        enqueueRadioPacket(CONNPATH_OPEN_CONNECTION_REQ, connid, FORWARD, newdatasize, newdata);
        delete[] newdata;
      } else {
        LIB_LOGI(TAG, "ConnectedPath::openConnection port %d", port);
        conn->destAddr = 0;
        conn->destHandle = 0;
        // Ack open connection request
        sendPacket(CONNPATH_OPEN_CONNECTION_ACK, connid, REVERSE, 0, nullptr);
        // Call the handler to receive data for this port
        bool portFound = false;
        for (ConnectedPathBindedPort_t bp : mBindedPorts) {
          if (bp.port == port) {
            bp.handler(bp.arg, conn->sourceAddr, conn->sourceHandle);
            portFound = true;
          }
        }
        if (!portFound) {
          LIB_LOGE(TAG, "ConnectedPath::openConnection port %d not found", port);
          sendPacket(CONNPATH_OPEN_CONNECTION_NACK, connid, REVERSE, 0, nullptr);
        }
      }
    } else {
      LIB_LOGE(TAG, "ConnectedPath::openConnection not enough connections for %06X:%04X", from, handle);
      sendPacket(CONNPATH_OPEN_CONNECTION_NACK, connid, REVERSE, 0, nullptr);
    }
  }
}

void ConnectedPath::openConnectionNack(uint32_t from, uint16_t handle) {
  bool forward;
  uint8_t connid = findConnectionIndex(from, handle, &forward);
  if (CONN_OPERATIVE(connid)) {
    LIB_LOGD(TAG, "ConnectedPath::openConnectionNack from %06X:%04X connid %d direction %s", from, handle, connid,
             FORWARD2TXT(forward));
    sendPacket(CONNPATH_OPEN_CONNECTION_NACK, connid, forward, 0, nullptr);
    connectionSetInoperative(connid);
  } else {
    LIB_LOGE(TAG, "ConnectedPath::openConnectionNack invalid handle %06X:%04X", from, handle);
    // Open connection nack is expected to be received by the client and travel back to the coordinator
    // But we loose the connection to the coordinator we do nothing
  }
}

void ConnectedPath::openConnectionAck(uint32_t from, uint16_t handle) {
  bool direction;
  uint8_t connid = findConnectionIndex(from, handle, &direction);
  if (CONN_OPERATIVE(connid)) {
    LIB_LOGD(TAG, "ConnectedPath::openConnectionAck from %06X:%04X connid %02X direction %s", from, handle, connid,
             FORWARD2TXT(direction));
    mConnectsions[connid].lastTime = millis();
    sendPacket(CONNPATH_OPEN_CONNECTION_ACK, connid, direction, 0, nullptr);
  } else {
    LIB_LOGE(TAG, "ConnectedPath::openConnectionAck on invalid connection from %06X:%04X", from, handle);
    // Open connection ack is expected to be received by the client and travel back to the coordinator
    // But we loose the connection to the coordinator we have to close the client connection
    // FIXME: sendImmediatePacket(CONNPATH_DISCONNECT_ACK, from, handle, 0, nullptr);
  }
}

void ConnectedPath::disconnect(uint32_t from, uint16_t handle) {
  bool direction;
  int8_t connid = findConnectionIndex(from, handle, &direction);
  if (CONN_OPERATIVE(connid)) {
    LIB_LOGI(TAG, "ConnectedPath::disconnect connid %d from %06X:%04X direction %s", connid, from, handle,
             FORWARD2TXT(direction));
    if (sendPacket(CONNPATH_DISCONNECT_REQ, connid, direction, 0, nullptr)) {
      if (mConnectsions[connid].disconnect != nullptr)
        mConnectsions[connid].disconnect(mConnectsions[connid].arg);
      else
        LIB_LOGE(TAG, "ConnectedPath::disconnect no disconnect callback for %06X:%04X", from, handle);
    }
    connectionSetInoperative(connid);
  } else {
    LIB_LOGE(TAG, "ConnectedPath::disconnect invalid handle %06X:%04X", from, handle);
    // Disconnect request can either from the client or from the coordinator.
    // But we already lost the connection we can't propagate the disconnect request
  }
}

void ConnectedPath::sendData(const uint8_t *buffer, uint16_t size, uint32_t source, uint16_t handle) {
  bool forward;
  int8_t connidx = findConnectionIndex(source, handle, &forward);

  if (CONN_OPERATIVE(connidx)) {
    mConnectsions[connidx].lastTime = millis();
    if (sendPacket(CONNPATH_SEND_DATA, connidx, forward, size, buffer)) {
      if (mConnectsions[connidx].receive != nullptr)
        mConnectsions[connidx].receive(mConnectsions[connidx].arg, buffer, size, connidx);
      else
        LIB_LOGE(TAG, "ConnectedPath::sendData no receive callback for %06X:%04X", source, handle);
    }
  } else {
    LIB_LOGE(TAG, "ConnectedPath::sendData on invalid connection from %06X:%04X", source, handle);
    // No handle for this data transmisison i send back a NACK
    sendImmediatePacket(CONNPATH_SEND_DATA_NACK, source, handle, 0, nullptr);
  }
}

/**
 * @brief If an invalid handle is received from a radio packet, invalidate the connection if it exists and propagate
 * the invalid handle to the next node in the path.
 * @param source - The address of the source of the packet.
 * @param sourceHandle - The handle of the source of the packet.
 */
void ConnectedPath::sendDataNack(uint32_t source, uint16_t sourceHandle) {
  bool forward;
  int8_t connid = findConnectionIndex(source, sourceHandle, &forward);
  if (CONN_OPERATIVE(connid)) {
    LIB_LOGD(TAG, "ConnectedPath::sendDataNack connid %d source %06X:%04X direction %s", connid, source, sourceHandle, FORWARD2TXT(forward));
    sendPacket(CONNPATH_SEND_DATA_NACK, connid, forward, 0, nullptr);
    connectionSetInoperative(connid);
  }
}

/**
 * @brief Process the mRadioOutputBuffer buffer in order to merge packets that belong to the same connection in a
 * single Radio packet.
 */
void ConnectedPath::processOutputBuffer() {
  // Maximum time to wait for a packet to be sent
  const unsigned long timeout = 20;
  // Check if is the header of a packet that can be sent
  auto checkHeader = [](ConnectedPathOutputBufferHeader &header, ConnectedPathOutputBufferHeader &lastHeader,
                        uint32_t now, bool isfirst) -> bool {
    // Check timeout only for first packet
    bool isnotfirst = !isfirst;
    bool istimeout = isnotfirst || (EspMeshMesh::elapsedMillis(now, header.pkttime) > timeout);
    bool issimilar = isfirst || ((header.connId == lastHeader.connId) && (header.forward == lastHeader.forward));
    bool result = istimeout && issimilar;
    // LIB_LOGI(TAG, "ConnectedPath::processOutputBuffer checkHeader istimeout %d issimilar %d result %d", istimeout,
    //          issimilar, result);
    return result;
  };

  auto setheader = [](ConnectedPathOutputBufferHeader &header, uint8_t connid, bool forward, uint8_t subProtocol,
                      uint32_t pkttime, uint16_t dataSize) {
    header.connId = connid;
    header.forward = forward;
    header.subProtocol = subProtocol;
    header.pkttime = pkttime;
    header.dataSize = dataSize;
  };

  ConnectedPathOutputBufferHeader header;

  uint32_t now = millis();
  uint8_t *buffer = nullptr;
  uint16_t bufferTotalSize = 0;

  ConnectedPathOutputBufferHeader lastHeader;
  setheader(lastHeader, CONNPATH_MAX_CONNECTIONS, false, CONNPATH_INVALID_REQ, 0, 0);
  uint16_t offset = 0;
  uint16_t pktProcessed = 0;
  bool processing = true;
  bool isfirst = true;

  // First pass to get the total size of the buffer
  while (processing) {
    processing = false;
    uint16_t readed = mRadioOutputBuffer.viewData2((uint8_t *) &header, sizeof(header), offset);
    if (readed == sizeof(ConnectedPathOutputBufferHeader)) {
      if (checkHeader(header, lastHeader, now, isfirst)) {
        if (bufferTotalSize + header.dataSize < 1024) {
          bufferTotalSize += header.dataSize;
          setheader(lastHeader, header.connId, header.forward, header.subProtocol, header.pkttime, header.dataSize);
          offset += sizeof(ConnectedPathOutputBufferHeader) + header.dataSize;
          isfirst = false;
          pktProcessed++;
          processing = true;
          // LIB_LOGI(TAG, "ConnectedPath::processOutputBuffer pkts %d connid %d subprot %d lastconnid %d size %d",
          //          pktProcessed, header.connId, header.subProtocol, lastHeader.connId, bufferTotalSize);
        }
      }
    }
  }

  if (bufferTotalSize > 0) {
    // Allocate the buffer
    buffer = new uint8_t[bufferTotalSize];
  }

  // setheader(lastHeader, CONNPATH_MAX_CONNECTIONS, false, CONNPATH_INVALID_REQ, 0, 0);
  isfirst = true;
  uint16_t bufferPos = 0;
  // Only for debug purposes
  uint16_t numPktProcessed = pktProcessed;
  uint32_t firstPktTime = 0;

  // Second pass to fill the buffer with the data
  while (pktProcessed--) {
    uint16_t readed = mRadioOutputBuffer.popData((uint8_t *) &header, sizeof(header));
    if (readed != sizeof(ConnectedPathOutputBufferHeader)) {
      LIB_LOGE(TAG, "ConnectedPath::processOutputBuffer invalid packet in buffer readed %d awaited %d", readed,
               sizeof(ConnectedPathOutputBufferHeader));
      continue;
    }
    if (isfirst) {
      firstPktTime = header.pkttime;
    }
    if (header.dataSize > 0) {
      readed = mRadioOutputBuffer.popData(buffer + bufferPos, header.dataSize);
      if (readed != header.dataSize) {
        LIB_LOGE(TAG, "ConnectedPath::processOutputBuffer invalid packet in buffer readed %d awaited %d", readed,
                 header.dataSize);
        continue;
      }
      bufferPos += header.dataSize;
    }
    isfirst = false;
  }

  if (numPktProcessed > 0) {
    if (CONN_EXISTS(lastHeader.connId)) {
      ConnectedPathConnections *conn = mConnectsions + lastHeader.connId;
      ConnectedPathPacket *pkt =
          cratePacket(lastHeader.subProtocol, bufferTotalSize, lastHeader.forward ? conn->destAddr : conn->sourceAddr,
                      lastHeader.forward ? conn->destHandle : conn->sourceHandle, buffer);
      LIB_LOGVV(TAG, "ConnectedPath::processOutputBuffer prot %d num packets %d tot size %d after %dms",
               lastHeader.subProtocol, numPktProcessed, bufferTotalSize,
               EspMeshMesh::elapsedMillis(now, lastHeader.pkttime));
      sendRadioPacket(pkt, false, true);
    } else {
      LIB_LOGE(TAG, "ConnectedPath::processOutputBuffer invalid connection id %d", lastHeader.connId);
    }
  }

  // Free the buffer
  if (buffer)
    delete[] buffer;
}

const ConnectedPathConnections *ConnectedPath::findConnection(uint32_t from, uint16_t handle) const {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if ((from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) ||
        (from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle)) {
      return mConnectsions + i;
    }
  }
  return nullptr;
}

ConnectedPathConnections *ConnectedPath::findConnection(uint32_t from, uint16_t handle) {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if ((from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) ||
        (from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle)) {
      return mConnectsions + i;
    }
  }
  return nullptr;
}

/**
 * @brief Find an active connection by source node address and  the source handle.
 * @param source - The address of the source of the packet.
 * @param sourceHandle - The handle of the source of the packet.
 * @param forward - If is true the packet is travelling from source to destination. If false the packet is
 * travelling from destination to source.
 * @param destinationAddress - The destination address of the connection.
 * @param destinationHandle - The destination handle of the connection.
 * @return The index of the connection in the mConnectsions array or CONNPATH_MAX_CONNECTIONS if not found.
 */
uint8_t ConnectedPath::findConnection(uint32_t source, uint16_t sourceHandle, bool &forward,
                                      uint32_t &destinationAddress, uint16_t &destinationHandle) {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if (source == mConnectsions[i].sourceAddr && sourceHandle == mConnectsions[i].sourceHandle) {
      forward = true;
      destinationAddress = mConnectsions[i].destAddr;
      destinationHandle = mConnectsions[i].destHandle;
      return i;
    }
    if (source == mConnectsions[i].destAddr && sourceHandle == mConnectsions[i].destHandle) {
      forward = false;
      destinationAddress = mConnectsions[i].sourceAddr;
      destinationHandle = mConnectsions[i].sourceHandle;
      return i;
    }
  }
  return CONNPATH_MAX_CONNECTIONS;
}

/**
 * @brief Find an active connection by node address and  the connection handle.
 * @param from - The address of the source of the packet.
 * @param handle - The handle of the source of the packet.
 * @param forward - Returned value. If is true the packet is travelling from source to destination. If false the
 * packet is travelling from destination to source. Can be nullptr if not needed.
 * @return The index of the connection in the mConnectsions array or CONNPATH_MAX_CONNECTIONS if not found.
 */
uint8_t ConnectedPath::findConnectionIndex(uint32_t from, uint16_t handle, bool *forward) {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if (from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) {
      if (forward)
        *forward = true;
      return i;
    }
    if (from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle) {
      if (forward)
        *forward = false;
      return i;
    }
  }
  return CONNPATH_MAX_CONNECTIONS;
}

uint8_t ConnectedPath::findConnectionPeer(uint8_t connIdx, bool forward, uint32_t &peerAddress, uint16_t &peerHandle) {
  if (connIdx >= CONNPATH_MAX_CONNECTIONS) {
    return 1;
  }

  if (forward) {
    peerAddress = mConnectsions[connIdx].destAddr;
    peerHandle = mConnectsions[connIdx].destHandle;
  } else {
    peerAddress = mConnectsions[connIdx].sourceAddr;
    peerHandle = mConnectsions[connIdx].sourceHandle;
  }

  return 0;
}

void ConnectedPath::sendUartPacket(uint8_t command, uint16_t handle, const uint8_t *data, uint16_t size) {
  if (size == 0) {
    uint8_t _data[4];
    _data[0] = CMD_CONNPATH_REPLY;
    _data[1] = command;
    uint16toBuffer(_data + 2, handle);
    mMeshMesh->uartSendData(_data, 4);
  } else {
    uint8_t *_data = new uint8_t[size + 4];
    _data[0] = CMD_CONNPATH_REPLY;
    _data[1] = command;
    uint16toBuffer(_data + 2, handle);
    memcpy(_data + 4, data, size);
    mMeshMesh->uartSendData(_data, size + 4);
    delete[] _data;
  }
}

ConnectedPathPacket *ConnectedPath::cratePacket(uint8_t subprot, uint16_t size, uint32_t target, uint16_t handle,
                                                const uint8_t *payload) {
  ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
  pkt->allocClearData(size);
  pkt->getHeader()->subprotocol = subprot;
  pkt->setTarget(target, handle);
  if (payload)
    pkt->setPayload(payload);
  return pkt;
}

/**
 * @brief Send a packet to the destination the destination is the peer of the connection. (Uart, Client or Radio)
 * @param subprot - The connection subprotocol of the packet.
 * @param connid - The index of the connection in the mConnectsions array.
 * @param forward - If is true the packet is travelling from source to destination. If false the packet is
 * travelling from destination to source.
 * @param size - The size of the payload.
 * @param data - The payload of the packet.
 * @return True if the packet must be handled by the caller, false otherwise.
 */
bool ConnectedPath::sendPacket(uint8_t subprot, uint8_t connid, bool direction, uint16_t size, const uint8_t *data) {
  uint32_t destAddress;
  uint16_t destHandle;
  findConnectionPeer(connid, direction, destAddress, destHandle);

  if (destAddress == CONNPATH_COORDINATOR_ADDRESS) {
    // Only send to UART if the packet is travelling from destination to source and the destination is 0
    // (coordinator), otherwise do nothing
    if (direction == REVERSE)
      sendUartPacket(subprot, destHandle, data, size);
    else
      return true;
  } else {
    enqueueRadioPacket(subprot, connid, direction, size, data);
  }
  return false;
}

void ConnectedPath::sendImmediatePacket(uint8_t subprot, uint32_t destination, uint16_t destinationHandle,
                                        uint16_t size, const uint8_t *data) {
  if (destination == CONNPATH_COORDINATOR_ADDRESS) {
    // FIXME: send to uart if we are the coordinator. Otherwise send to client
    sendUartPacket(subprot, destinationHandle, nullptr, 0);
  } else {
    sendRadioPacket(cratePacket(subprot, 0, destination, destinationHandle, nullptr), true, true);
  }
}

void ConnectedPath::debugConnection() const {
  uint32_t now = millis();
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if (mConnectsions[i].sourceAddr != CONNPATH_INVALID_ADDRESS) {
      const ConnectedPathConnections &c = mConnectsions[i];
      uint32_t t = EspMeshMesh::elapsedMillis(now, c.lastTime);
      LIB_LOGD(TAG, "connections: %02X %06X:%04X -> %06X:%04X (%ld)", i, c.sourceAddr, c.sourceHandle, c.destAddr,
               c.destHandle, t);
    }
  }
}

}  // namespace espmeshmesh

#endif
