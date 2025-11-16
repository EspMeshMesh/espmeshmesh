#include "starpath.h"
#include <functional>

namespace espmeshmesh {

StarPathProtocol::StarPathProtocol(PacketBuf *pbuf, ReceiveHandler rx_fn): PacketBufProtocol(pbuf, rx_fn, MeshAddress::SRC_STARPATH) {
    mPacketBuf->setRecvHandler(MeshAddress::SRC_STARPATH,  this);
}

void StarPathProtocol::setup() {
}

void StarPathProtocol::loop() {
}

uint8_t StarPathProtocol::send(const uint8_t *data, uint16_t size, MeshAddress target, SentStatusHandler handler) {
    return PKT_SEND_ERR;
}

void StarPathProtocol::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {

}

void StarPathProtocol::radioPacketSent(uint8_t status, RadioPacket *pkt) {

}


}