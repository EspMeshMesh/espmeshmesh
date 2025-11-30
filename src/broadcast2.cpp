#include "broadcast2.h"
#include "meshaddress.h"
#include "log.h"
#include <cstring>

static const char *TAG = "espmeshmesh.broadcast2";

namespace espmeshmesh {

void BroadCast2Packet::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast2_header_st));
}

void Broadcast2::send(const uint8_t *data, uint16_t size, uint16_t port, SentStatusHandler handler) {
  LIB_LOGV(TAG, "Broadcast2::send port %d size %d", port, size);
  BroadCast2Packet *pkt = new BroadCast2Packet(this, handler);
  pkt->allocClearData(size);
	pkt->broadcastHeader()->protocol = PROTOCOL_BROADCAST_V2;
	pkt->broadcastHeader()->lenght = size;
	pkt->broadcastHeader()->flags = 0;
	pkt->broadcastHeader()->port = port;
	memcpy(pkt->broadcastPayload(), data, size);

    pkt->encryptClearData();
    pkt->fill80211(nullptr, mPacketBuf->nodeIdPtr());
    mPacketBuf->send(pkt);
}

void Broadcast2::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {
  broadcast2_header_t *header = (broadcast2_header_t *) payload;
  LIB_LOGV(TAG, "Broadcast2::recv port %d size %d", header->port, header->lenght);
  MeshAddress sourceAddress = MeshAddress(0, from);
  sourceAddress.sourceProtocol = MeshAddress::SRC_BROADCAST2;
  this->callReceiveHandler(payload + sizeof(broadcast2_header_t), header->lenght, sourceAddress, rssi);
}

}  // namespace espmeshmesh
