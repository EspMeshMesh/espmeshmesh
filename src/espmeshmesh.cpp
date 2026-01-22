#include "espmeshmesh.h"
#include <algorithm>

#ifdef IDF_VER
#include <esp_sleep.h>
#endif

#ifdef ESP8266
#include <Esp.h>
#endif

#include "log.h"
#include "crc16.h"
#include "commands.h"
#include "packetbuf.h"
#include "broadcast.h"
#include "broadcast2.h"
#include "unicast.h"
#include "multipath.h"
#include "starpath.h"
#include "uart.h"
#include "wifi.h"



#ifdef USE_POLITE_BROADCAST_PROTOCOL
#include "polite.h"
#endif
#include "connectedpath.h"

extern "C" uint32_t _FS_start;
extern "C" uint32_t _SPIFFS_start;

namespace espmeshmesh {

using namespace std::placeholders;

static const char *TAG = "espmeshmesh";

#define BROADCAST_DEFAULT_PORT 0
#define UNICAST_DEFAULT_PORT 0
#define MULTIPATH_DEFAULT_PORT 0
#define SERIAL_DEFAULT_PORT 0

#define DEF_CMD_BUFFER_SIZE 0x500
#define MAX_CMD_BUFFER_SIZE 0x500

EspMeshMesh *EspMeshMesh::singleton = nullptr;

EspMeshMesh *EspMeshMesh::getInstance() { return singleton; }

EspMeshMesh::EspMeshMesh(int baud_rate, int tx_buffer, int rx_buffer): mBaudRate(baud_rate), mTxBuffer(tx_buffer), mRxBuffer(rx_buffer) {
  if (singleton == nullptr) singleton = this;
  mWifi = wifiFactory(this);
  if(baud_rate > 0) mUart = uartFactory(this);
}

void EspMeshMesh::pre_setup() {
}

void EspMeshMesh::setup(SetupConfig *config) {
  mTeardownDeadline = 0;

  if(mBaudRate > 0) {
    mUart->setBaudRate(mBaudRate);
    mUart->setTxBuffer(mTxBuffer);
    mUart->setRxBuffer(mRxBuffer);
    mUart->setup();
  }

  mWifi->setup(config->hostname.c_str(), config->channel, config->txPower);

  mFwVersion = config->fwVersion;
  mCompileTime = config->compileTime;
  mHostName = config->hostname;
  mNodeType = config->nodeType;

  auto handler = std::bind(&EspMeshMesh::handleFrame, this, _1, _2, _3, _4);

  packetbuf = PacketBuf::getInstance();
  packetbuf->setup(mWifi->getAesPassword(), 16);

  broadcast = new Broadcast(packetbuf, handler);
  broadcast2 = new Broadcast2(packetbuf, handler);

  unicast = new Unicast(packetbuf, handler);
  multipath = new MultiPath(packetbuf, handler);
#ifdef ESPMESH_STARPATH_ENABLED
  starpath = new StarPathProtocol(packetbuf, handler);
#endif

#ifdef USE_POLITE_BROADCAST_PROTOCOL
  mPoliteBroadcast = new PoliteBroadcastProtocol(packetbuf, handler);
#endif

  mConnectedPath = new ConnectedPath(this, packetbuf);

#ifdef ESPMESH_STARPATH_ENABLED
  starpath->setup();
#endif

  mDiscovery.init();
  dump_config();
  mElapsed1 = millis();



#ifdef USE_BINARY_SENSOR
  for (auto binary : App.get_binary_sensors()) {
    LIB_LOGCONFIG(TAG, "Found binary sensor %s with hash %08X", binary->get_object_id().c_str(),
                  binary->get_object_id_hash());
  }
#endif

#ifdef USE_TEST_PROCEDURE
  mTestProcedureTime = millis();
#endif
}

void EspMeshMesh::setAesPassword(std::string password) {
  if(mWifi) mWifi->setAesPassword(password);
}

void EspMeshMesh::dump_config() {
  LIB_LOGCONFIG(TAG, "EspMeshMesh " ESPMESHMESH_VERSION " configuration:");
  LIB_LOGCONFIG(TAG, "Hostname: %s", mHostName.c_str());
  LIB_LOGCONFIG(TAG, "Node type: %s", mNodeType == ESPMESH_NODE_TYPE_COORDINATOR ? "Coordinator" : mNodeType == ESPMESH_NODE_TYPE_BACKBONE ? "Backbone" : "Edge");
  LIB_LOGCONFIG(TAG, "Firmware version: %s", mFwVersion.c_str());
  LIB_LOGCONFIG(TAG, "Compile time: %s", mCompileTime.c_str());
  mWifi->dump_config();
#ifdef IDF_VER
  LIB_LOGCONFIG(TAG, "Reset cause: %d", esp_sleep_get_wakeup_cause());
#endif
}

void EspMeshMesh::loop() {
  if(mUart) mUart->loop();
  uint32_t now = millis();
#ifdef USE_ESP32
  packetbuf->loop();
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  if (mPoliteBroadcast)
    mPoliteBroadcast->loop();
#endif
  // execute multipath loop
  multipath->loop();
#ifdef ESPMESH_STARPATH_ENABLED
  starpath->loop();
#endif
  mConnectedPath->loop();
  // Execute discovery if is running
  if (mDiscovery.isRunning())
    mDiscovery.loop(this);

#ifdef ESP8266
  if (!mWorkAround && elapsedMillis(now, mElapsed1) > 2000) {
    mWorkAround = true;
    uint8_t data[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
    unicast->send(data, 0x6, 0x111111, UNICAST_DEFAULT_PORT, nullptr);
    LIB_LOGI(TAG, "Sent dummy workaround packet");
  }
#endif

  if (elapsedMillis(now, mElapsed1) > 60000) {
#ifdef IDF_VER
    LIB_LOGI(TAG, "Free Heap %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
#else
    LIB_LOGI(TAG, "Free Heap %d", ESP.getFreeHeap());
#endif
    mElapsed1 = millis();
  }
}

void EspMeshMesh::shutdown() {
  LIB_LOGI(TAG, "Shutting down meshmesh...");
  mConnectedPath->shutdown();
  mTeardownPhase = 1;
}

bool EspMeshMesh::teardown() {
  bool teardown = false;
  switch(mTeardownPhase) {
    case 1:
      if(mConnectedPath->teardown()) {
#ifdef ESPMESH_STARPATH_ENABLED
        starpath->shutdown();
#endif
        mTeardownPhase = 2;
      }
      break;
    case 2:
#ifdef ESPMESH_STARPATH_ENABLED
       if(starpath->teardown()) {
        mTeardownPhase = 3;
       }
#else
        mTeardownPhase = 3;
#endif
       break;
    case 3:
      mTeardownDeadline = millis() + 250;
      mTeardownPhase = 4;
      break;
    case 4:
      teardown = mTeardownDeadline != 0 && millis() > mTeardownDeadline ? true : false;
      break;
  }
  this->loop(); // Loop is not called by the App during teardown
  return teardown;
}

const std::string EspMeshMesh::libVersion() const { return ESPMESHMESH_VERSION; }

void EspMeshMesh::uartSendData(const uint8_t *buff, uint16_t len) {
  if(mUart) mUart->sendFramedData(buff, len);
  else LIB_LOGE(TAG, "uartSendData no uart");
}

void EspMeshMesh::broadcastSendData(const uint8_t *buff, uint16_t len) {
  if (broadcast)
    broadcast->send(buff, len);
}

void EspMeshMesh::broadcastSendData(const uint8_t *buff, uint16_t len, uint16_t port) {
  if (broadcast2)
    broadcast2->send(buff, len, port, nullptr);
}

void EspMeshMesh::unicastSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint16_t port) {
  if (unicast)
    unicast->send(buff, len, addr, port, nullptr);
}

void EspMeshMesh::multipathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen, uint32_t *path) {
  if (!multipath)
    return;
  MultiPathPacket *pkt = new MultiPathPacket(multipath, nullptr);
  pkt->allocClearData(len, pathlen);
  pkt->multipathHeader()->trargetAddress = addr;
  for (int i = 0; i < pathlen; i++)
    pkt->setPathItem(path[i], i);
  pkt->setPayload(buff);
  multipath->send(pkt, true, nullptr);
}


void EspMeshMesh::commandReply(const uint8_t *buff, uint16_t len) {
  switch (mFromAddress.sourceProtocol) {
    case MeshAddress::SRC_SERIAL:
      if(mUart) mUart->sendFramedData(buff, len); 
      else LIB_LOGE(TAG, "commandReply no uart");
      break;
    case MeshAddress::SRC_BROADCAST:
    case MeshAddress::SRC_BROADCAST2:
    case MeshAddress::SRC_UNICAST:
      unicast->send(buff, len, mFromAddress.address, UNICAST_DEFAULT_PORT, nullptr);
      break;
    case MeshAddress::SRC_STARPATH:
    case MeshAddress::SRC_MULTIPATH:
      multipath->send(buff, len, mFromAddress, nullptr);
      break;
    case MeshAddress::SRC_POLITEBRD:
#ifdef USE_POLITE_BROADCAST_PROTOCOL
      if (mFromAddress != POLITE_DEST_BROADCAST)
        mPoliteBroadcast->send(buff, len, mFromAddress);
      break;
#endif
    case MeshAddress::SRC_NONE:
    case MeshAddress::SRC_CONNPATH:
    case MeshAddress::SRC_FILTER:
    case MeshAddress::SRC_PREROUTED:
      LIB_LOGE(TAG, "commandReply invalid source protocol %02X", mFromAddress.sourceProtocol);
      break;
  }

  mFromAddress.sourceProtocol = MeshAddress::SRC_NONE;
}

void EspMeshMesh::handleFrame(const uint8_t *data, uint16_t len, const MeshAddress &from, int16_t rssi) {
  uint8_t err = HANDLE_UART_ERROR;

  if (len == 0 || data[0] == 0x7F)
    return;

  mFromAddress = from;
  mRssiHandle = rssi;

  if (data[0] & 0x01) {
    replyHandleFrame(data, len, from, rssi);
    return;
  }

  switch (data[0]) {
    case CMD_UART_ECHO_REQ:
      if (len > 1) {
        uint8_t *cpy = new uint8_t[len];
        memcpy(cpy, data, len);
        cpy[0] = CMD_UART_ECHO_REP;
        if(mUart) mUart->sendFramedData(cpy, len); 
        else LIB_LOGE(TAG, "handleFrame no uart");
        delete[] cpy;
        err = 0;
      }
      break;
    case CMD_NODE_ID_REQ:  // 04 --> 05 0000XXYY
      if (len == 1) {
        uint32_t id = Discovery::chipId();
        uint8_t rep[5] = {0};
        rep[0] = CMD_NODE_ID_REP;
        memcpy(rep + 1, (uint8_t *) &id, 4);
        commandReply(rep, 5);
        err = 0;
      }
      break;
    case CMD_DISCOVERY_REQ:
      if (len > 1) {
        err = mDiscovery.handle_frame(data + 1, len - 1, this);
      }
      break;
    case CMD_BROADCAST_SEND:  // 70 AABBCCDDEE...ZZ
      if (len > 1) {
        broadcast->send(data + 1, len - 1);
        err = 0;
      }
      break;
    case CMD_UNICAST_SEND:  // 72 0000XXYY AABBCCDDEE...ZZ
      if (len > 5) {
        LIB_LOGD(TAG, "CMD_UNICAST_SEND len %d", len);
        unicast->send(data + 5, len - 5, uint32FromBuffer(data + 1), UNICAST_DEFAULT_PORT, nullptr);
        err = 0;
      }
      break;
    case CMD_MULTIPATH_SEND:  // 76 AAAAAAAA BB CCCCCCCC........DDDDDDDD AABBCCDDEE...ZZ
      if (len > 5) {
        uint8_t pathlen = data[5];
        if (len > 6 + pathlen * sizeof(uint32_t)) {
          uint16_t payloadsize = len - (6 + pathlen * sizeof(uint32_t));
          const uint8_t *payload = data + 6 + sizeof(uint32_t) * pathlen;
          uint32_t target = uint32FromBuffer(data + 1);
          // Send the packet
          MeshAddress targetAddress(MULTIPATH_DEFAULT_PORT, target, data + 6, pathlen, false);
          multipath->send(payload, payloadsize, targetAddress, nullptr);
          err = 0;
        }
      }
      break;
#ifdef USE_POLITE_BROADCAST_PROTOCOL
    case CMD_POLITEBRD_SEND:
      if (len > 5) {
        mPoliteBroadcast->send(buf + 5, len - 5, uint32FromBuffer(buf + 1));
        err = 0;
      }
      break;
#endif
    case CMD_CONNPATH_REQUEST:
      if (len > 1 && from.sourceProtocol == MeshAddress::SRC_SERIAL) {
        err = mConnectedPath->receiveUartPacket(data + 1, len - 1);
      }
      break;
    case CMD_FILTERED_REQUEST:
      if (len > 5) {
        uint8_t i;
        uint8_t *groups = (uint8_t *) &mFilterGroups;
        for (i = 0; i < 4; i++)
          if (data[i + 1] != 0xFF && (groups[i] == 0 || (groups[i] != 0xFF && groups[i] != data[i + 1])))
            break;
        if (i == 4) {
          MeshAddress fromFilter(from);
          fromFilter.sourceProtocol = MeshAddress::SRC_FILTER;
          handleFrame(data + 5, len - 5, fromFilter, rssi);
        }
        err = 0;
      }
      break;
    default:
      for (auto cb : mHandleFrameCbs) {
        int8_t handled = cb(data, len, from, rssi);
        // If callback handled the frame...
        if (handled >= 0) {
          // Keep status and exit loop
          err = handled;
          break;
        }
      }
      break;
  }

  if (err == HANDLE_UART_ERROR && mFromAddress.sourceProtocol != MeshAddress::SRC_BROADCAST) {
    // Don't reply errors when commd came from broadcast
    uint8_t *rep = new uint8_t[len + 1];
    rep[0] = CMD_ERROR_REP;
    memcpy(rep + 1, data, len);
    commandReply(rep, len + 1);
    LIB_LOGE(TAG, "EspMeshMesh::handleFrame error frame %02X %02X size %d", data[0], data[1], len);
    delete[] rep;
  }
}

void EspMeshMesh::replyHandleFrame(const uint8_t *buf, uint16_t len, const MeshAddress &from, int16_t rssi) {
  // All replies go to the serial, if the serial is active
  switch (buf[0]) {
    case CMD_LOGEVENT_REP: {
      // Add the source to the log essage
      uint8_t *cpy = new uint8_t[len];
      memcpy(cpy, buf, len);
      memcpy(cpy + 3, (uint8_t *) &from, 4);
      if(mUart) mUart->sendFramedData(cpy, len); 
      delete[] cpy;
    } break;
    default:
      if(mUart) mUart->sendFramedData(buf, len); 
      break;
  }
}

void EspMeshMesh::sendLog(int level, const char *tag, const char *payload, size_t payload_len) {
  if (mUart == nullptr && mLogDestination == 0)
    return;

  // uint16_t buffersize = 7+taglen+1+payloadlen;
  uint16_t buffersize = 7 + payload_len;

  auto buffer = new uint8_t[buffersize];
  auto buffer_ptr = buffer;

  *buffer_ptr = CMD_LOGEVENT_REP;
  buffer_ptr++;
  uint16toBuffer(buffer_ptr, (uint16_t) level);
  buffer_ptr += 2;
  uint32toBuffer(buffer_ptr, 0);
  buffer_ptr += 4;

  if(payload_len > 0) memcpy(buffer_ptr, payload, payload_len);
  if(mUart) mUart->sendFramedData(buffer, buffersize);

  if (mLogDestination == 1) {
    if (broadcast)
      broadcast->send(buffer, buffersize);
  } else if (mLogDestination > 1) {
    if (unicast)
      unicast->send(buffer, buffersize, mLogDestination, UNICAST_DEFAULT_PORT, nullptr);
  }

  delete[] buffer;
}


}  // namespace espmeshmesh
