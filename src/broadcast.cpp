#include "broadcast.h"
#include "meshaddress.h"
#include <cstring>

namespace espmeshmesh {

void BroadCastPacket::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast_header_st));
}

uint8_t Broadcast::send(const uint8_t *data, uint16_t size) {
  BroadCastPacket *pkt = new BroadCastPacket(this, nullptr);
  pkt->allocClearData(size);
	pkt->broadcastHeader()->protocol = PROTOCOL_BROADCAST;
	pkt->broadcastHeader()->lenght = size;
	memcpy(pkt->broadcastPayload(), data, size);

    pkt->encryptClearData();
    pkt->fill80211(nullptr, mPacketBuf->nodeIdPtr());
    uint8_t res = mPacketBuf->send(pkt);
    if (res == PKT_SEND_ERR)
      delete pkt;
    return res;
}

void Broadcast::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {
  broadcast_header_t *header = (broadcast_header_t *) payload;
  MeshAddress sourceAddress = MeshAddress(0, from);
  sourceAddress.sourceProtocol = SRC_BROADCAST;
  this->callReceiveHandler(payload + sizeof(broadcast_header_t), header->lenght, sourceAddress, rssi);
}

}  // namespace espmeshmesh
