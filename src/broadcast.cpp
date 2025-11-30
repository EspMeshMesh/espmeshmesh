#include "broadcast.h"
#include "meshaddress.h"
#include <cstring>

namespace espmeshmesh {

void BroadCastPacket::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast_header_st));
}

void Broadcast::send(const uint8_t *data, uint16_t size) {
  BroadCastPacket *pkt = new BroadCastPacket(this, nullptr);
  pkt->allocClearData(size);
	pkt->broadcastHeader()->protocol = PROTOCOL_BROADCAST;
	pkt->broadcastHeader()->lenght = size;
	memcpy(pkt->broadcastPayload(), data, size);

  pkt->encryptClearData();
  pkt->fill80211(nullptr, mPacketBuf->nodeIdPtr());
  mPacketBuf->send(pkt);
}

void Broadcast::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {
  broadcast_header_t *header = (broadcast_header_t *) payload;
  MeshAddress sourceAddress = MeshAddress(0, from);
  sourceAddress.sourceProtocol = MeshAddress::SRC_BROADCAST;
  this->callReceiveHandler(payload + sizeof(broadcast_header_t), header->lenght, sourceAddress, rssi);
}

}  // namespace espmeshmesh
