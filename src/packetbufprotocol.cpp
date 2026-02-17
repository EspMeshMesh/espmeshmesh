#include "packetbufprotocol.h"
#include "log.h"
#include "packetbuf.h"
#include "radiopacket.h"

namespace espmeshmesh {

PacketBufProtocol::PacketBufProtocol(PacketBuf *pbuf, ProtocolReceiveHandler rx_fn, MeshAddress::DataSrc protocol):  
    mPacketBuf(pbuf), mProtocolType(protocol) {

    if (rx_fn) {
    this->bindPort(0, rx_fn);
    }
    if (protocol != MeshAddress::SRC_NONE) {
    pbuf->setRecvHandler(protocol, this);
    }
}

bool PacketBufProtocol::isPortAvailable(uint16_t port) const {
    //LIB_LOGV(TAG, "isPortAvailable: protocol type %d port %d total ports %d", (int)mProtocolType, port, this->mBindedPorts.size());
    auto it = this->mBindedPorts.find(port);
    return (it == this->mBindedPorts.end());
}

bool PacketBufProtocol::bindPort(uint16_t port, ProtocolReceiveHandler handler) {
    if (!this->isPortAvailable(port)) {
        LIB_LOGE(TAG, "bindPort: port %d already binded on protocol type %d", port, (int)mProtocolType);
        return false;
    }
    this->mBindedPorts[port] = handler;
    LIB_LOGV(TAG, "bindPort: port %d is now binded on protocol type %d total ports %d", port, (int)mProtocolType, this->mBindedPorts.size());
    return true;
}

void PacketBufProtocol::unbindPort(uint16_t port) {
  LIB_LOGV(TAG, "unbindPort: ports size %d", this->mBindedPorts.size());
  const auto it = this->mBindedPorts.find(port);
  if (it != this->mBindedPorts.end()) {
    this->mBindedPorts.erase(it);
    return;
  }

  LIB_LOGE(TAG, "unbindPort: port %d is not binded on protocol type %d", port, (int)mProtocolType);
}

void PacketBufProtocol::callReceiveHandler(const uint8_t *payload, uint16_t size, const MeshAddress &from, int16_t rssi) {
  if (isPortAvailable(from.port)) {
    LIB_LOGE(TAG, "from %06X port %d is not binded to any callback on protocol type %d", from.address, from.port, (int)mProtocolType);
  } else {
    auto cb = mBindedPorts[from.port];
    cb(payload, size, from, rssi);
  }
}

void PacketBufProtocol::radioPacketSent(uint8_t status, RadioPacket *pkt) {
    pkt->callCallback(status, pkt);
}

}