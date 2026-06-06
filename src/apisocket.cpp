#include "apisocket.h"
#ifdef API_SOCKET_ENABLED
#include "commands.h"
#include "defines.h"
#include "meshsocket.h"
#include "espmeshmesh.h"

#include <pb_decode.h>
#include <pb_encode.h>

#include "protoc/apisocketsend.pb.h"

#define CMD_MESHSSOCK_BIND_REQ 0x00
#define CMD_MESHSSOCK_UNBIND_REQ 0x01
#define CMD_MESHSSOCK_SEND_REQ 0x02
#define CMD_MESHSSOCK_RECV_REQ 0x03
#define CMD_MESHSSOCK_SET_TIMEOUT_REQ 0x04

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.apisocket";

void ApiSocket::init(EspMeshMesh *parent) {
    mParent = parent;
}

void ApiSocket::loop() {
  if(!mPendingReply) {
    return;
  }

  if(millis() > mPendingReplyTimeout) {
    frameSocketApiSendReply(CMD_MESHSSOCK_SEND_REQ, 0, mPendingReplyPort, nullptr, 0);
    mPendingReplyPort = 0;
    mPendingReplyTimeout = 0;
    mPendingReply = false;
    return;
  }

  recvPendingDatagram(CMD_MESHSSOCK_SEND_REQ, mPendingReplyPort);
}

uint8_t ApiSocket::handleFrame(const uint8_t *data, uint16_t len) {
  LIB_LOGD(TAG, "handle_frame_socket_api len %d", len);

  if(len < 3) {
    return HANDLE_UART_ERROR;
  }

  switch (data[0]) {
    case CMD_MESHSSOCK_BIND_REQ:
      if(len == 3) {
        uint16_t port = uint16FromBuffer(data + 1);
        bindSocket(port);
        frameSocketApiReply(CMD_MESHSSOCK_BIND_REQ, port, 1);
        return HANDLE_UART_OK;
      }
      break;
    case CMD_MESHSSOCK_UNBIND_REQ:
      if(len == 3) {
        uint16_t port = uint16FromBuffer(data + 1);
        unbindSocket(port);
        frameSocketApiReply(CMD_MESHSSOCK_UNBIND_REQ, port, 1);
        return HANDLE_UART_OK;
      }
      break;
    case CMD_MESHSSOCK_SEND_REQ:
      return decodeFrameSocketApiSendRequest(data, len) ? HANDLE_UART_OK : HANDLE_UART_ERROR;
    case CMD_MESHSSOCK_RECV_REQ:
      if(len == 3) {
        uint16_t port = uint16FromBuffer(data + 1);
        recvPendingDatagram(CMD_MESHSSOCK_RECV_REQ, port);
        return HANDLE_UART_OK;
      }
      break;
      return HANDLE_UART_OK;
    case CMD_MESHSSOCK_SET_TIMEOUT_REQ:
      if(len == 3) {
        mPendingReplyDelay = uint16FromBuffer(data + 1);
        frameSocketApiReply(CMD_MESHSSOCK_SET_TIMEOUT_REQ, 0, 1);
        return HANDLE_UART_OK;
      }
      break;
    default:
      return HANDLE_UART_ERROR;
  }

  LIB_LOGE(TAG, "handleFrame unknown command %02X", data[0]);
  return HANDLE_UART_ERROR;
}

void ApiSocket::recvPendingDatagram(uint8_t cmd, uint16_t port) {
  if(mBindedSockets.size() > 0) {
    for(auto socket : mBindedSockets) {
      if(socket->getTargetPort() == port && socket->pendingDatagrams() > 0) {
        LIB_LOGD(TAG, "recvPendingDatagram found socket for port %d", port);
        recvDatagram(cmd, socket);
      }
    }
  }
}

void ApiSocket::recvDatagram(uint8_t cmd, MeshSocket *socket) {
  uint16_t sizeNext = socket->sizeOfNextDatagram();
  LIB_LOGD(TAG, "recvDatagram sizeNext %d", sizeNext);
  if(sizeNext == 0) {
    return;
  }

  int16_t rssi;
  MeshAddress from;
  uint8_t *data = new uint8_t[sizeNext];

  int16_t res = socket->recvDatagram(data, sizeNext, from, rssi);
  if(res > 0) {
    if(mPendingReply) {
      frameSocketApiSendReply(cmd, from.address, socket->getTargetPort(), data, res);
      mPendingReply = false;
    } else {
      // TODO: Implement the logic to handle the received datagram
    }
  } else {
    LIB_LOGE(TAG, "recvDatagram failed %s", socket->error2string((MeshSocket::ErrorCodes)res));
  }

  delete[] data;
}

bool ApiSocket::bindSocket(uint16_t port) {
    auto socket = findMeshSocket(port);
    if(socket == nullptr) {
        socket = new MeshSocket(port);
        socket->open();
    }
    mBindedSockets.push_back(socket);
    return true;
}

void ApiSocket::unbindSocket(uint16_t port) {
    auto socket = findMeshSocket(port);
    if(socket != nullptr) {
        mBindedSockets.remove(socket);
        delete socket;
    }
}

bool ApiSocket::sendTo(MeshAddress target, const uint8_t *data, uint16_t len) {
  auto socket = findMeshSocket(target.port);
  if(socket == nullptr) {
      LIB_LOGE(TAG, "sendTo failed to find socket for port %d", target.port);
      return false;
  }
  int16_t err = socket->sendDatagram(data, len, target, nullptr);
  if(err < 0) {
    LIB_LOGE(TAG, "sendTo failed %s", socket->error2string((MeshSocket::ErrorCodes)err));
  }
  return err >= 0;
}

bool ApiSocket::recvFrom(MeshAddress &from, uint8_t *data, uint16_t len) {
  auto socket = findMeshSocket(from.port);
  if(socket == nullptr) {
      return false;
  }
  int16_t rssi = 0;
  int16_t size = socket->recvDatagram(data, len, from, rssi);
  if(size > 0) {
      return true;
  }
  return false;
}

MeshSocket *ApiSocket::findMeshSocket(uint16_t port) {
  for(auto socket : mBindedSockets) {
      if(socket->getTargetPort() == port) {
          return socket;
      }
  }
  return nullptr;
}

void ApiSocket::frameSocketApiReply(uint8_t cmd, uint16_t port, uint8_t status) {
  uint8_t rep[5];
  rep[0] = CMD_MESHSSOCK_REP;
  rep[1] = cmd;
  uint16toBuffer(rep + 2, port);
  rep[4] = status;
  mParent->commandReply(rep, 5);
}

void ApiSocket::frameSocketApiSendReply(uint8_t cmd, uint32_t address, uint16_t port, const uint8_t *data, uint16_t len) {
  uint8_t header[2] = {CMD_MESHSSOCK_REP, cmd};

  espmeshmesh_ApiSocketSend *apiSocketSend = new espmeshmesh_ApiSocketSend(espmeshmesh_ApiSocketSend_init_zero);
  apiSocketSend->has_address = true;
  apiSocketSend->address.address = address;
  apiSocketSend->address.port = port;
  if(len > 0) memcpy(apiSocketSend->data.bytes, data, len);
  apiSocketSend->data.size = len;

  pb_ostream_t sizestream = {0};
  if(!pb_encode(&sizestream, espmeshmesh_ApiSocketSend_fields, apiSocketSend)) {
    LIB_LOGE(TAG, "frameSocketApiSendReply encode size failed %s", PB_GET_ERROR(&sizestream));
    return;
  }
  uint16_t size = sizestream.bytes_written + 2;

  uint8_t *buffer = new uint8_t[size];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, size);
  pb_write(&stream, header, 2);
  if(!pb_encode(&stream, espmeshmesh_ApiSocketSend_fields, apiSocketSend)) {
    LIB_LOGE(TAG, "frameSocketApiSendReply encode failed %s", PB_GET_ERROR(&stream));
  } else {
    mParent->commandReply(buffer, size);
  }
  delete apiSocketSend;
  delete[] buffer;
}

bool ApiSocket::decodeFrameSocketApiSendRequest(const uint8_t *data, uint16_t len) {
  bool ret = false;
  espmeshmesh_ApiSocketSend *apiSocketSend = new espmeshmesh_ApiSocketSend;
  pb_istream_t istream = pb_istream_from_buffer(data+1, len-1);
  if(!pb_decode(&istream, espmeshmesh_ApiSocketSend_fields, apiSocketSend)) {
    LIB_LOGE(TAG, "handleFrame decode api socket send failed %s", PB_GET_ERROR(&istream));
  } else {
    LIB_LOGD(TAG, "handleFrame api socket send %d bytes to %06X:%d", apiSocketSend->data.size, apiSocketSend->address.address, apiSocketSend->address.port);
    if(sendTo(MeshAddress((uint16_t)apiSocketSend->address.port, apiSocketSend->address.address), apiSocketSend->data.bytes, apiSocketSend->data.size)) {
      // Wait for reply
      mPendingReply = true;
      mPendingReplyPort = apiSocketSend->address.port;
      mPendingReplyTimeout = millis() + mPendingReplyDelay;
      ret = true;
    }
  }
  delete apiSocketSend;
  return ret;
}

} // namespace espmeshmesh
#endif
