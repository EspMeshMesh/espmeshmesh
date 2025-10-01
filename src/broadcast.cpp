#include "broadcast.h"
#include <cstring>

namespace espmeshmesh {

void BroadCastPacket::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast_header_st));
}

Broadcast::Broadcast(PacketBuf *pbuf): mPacketbuf(pbuf) {
	mPacketbuf->setRecvHandler(PROTOCOL_BROADCAST, std::bind(&Broadcast::recv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

uint8_t Broadcast::send(const uint8_t *data, uint16_t size) {
	BroadCastPacket *pkt = new BroadCastPacket(nullptr, nullptr);
	pkt->allocClearData(size);
	pkt->broadcastHeader()->protocol = PROTOCOL_BROADCAST;
	pkt->broadcastHeader()->lenght = size;
	memcpy(pkt->broadcastPayload(), data, size);

    pkt->encryptClearData();
    pkt->fill80211(nullptr, mPacketbuf->nodeIdPtr());
	uint8_t res = mPacketbuf->send(pkt);
	if(res == PKT_SEND_ERR) delete pkt;
    return res;
}

void Broadcast::recv(uint8_t *p, uint16_t size, uint32_t from, int16_t rssi) {
	broadcast_header_t *brdchead = (broadcast_header_t *)p;
	if (rx_func) rx_func(p+sizeof(broadcast_header_t), brdchead->lenght, from, rssi);
}

void Broadcast::setRecv_cb(breadcast_recv_cb_fn rx_fn) {
	rx_func = rx_fn;
}

}  // namespace espmeshmesh
