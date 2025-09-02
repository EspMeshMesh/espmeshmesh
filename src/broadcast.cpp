#include "broadcast.h"

namespace espmeshmesh {

void BroadCastPacket::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast_header_st));
}

uint8_t Broadcast::send(const uint8_t *data, uint16_t size) {
	BroadCastPacket *pkt = new BroadCastPacket(nullptr, nullptr);
	pkt->allocClearData(size);
	pkt->broadcastHeader()->protocol = PROTOCOL_BROADCAST;
	pkt->broadcastHeader()->lenght = size;
	os_memcpy(pkt->broadcastPayload(), data, size);

    pkt->encryptClearData();
    pkt->fill80211(nullptr, packetbuf->nodeIdPtr());
	uint8_t res = packetbuf->send(pkt);
	if(res == PKT_SEND_ERR) delete pkt;
    return res;
}

void Broadcast::recv(uint8_t *p, uint16_t size, uint8_t *f, int16_t r) {
	broadcast_header_t *brdchead = (broadcast_header_t *)p;
	if (rx_func) rx_func(p+sizeof(broadcast_header_t), brdchead->lenght, f, r);
}

void Broadcast::setRecv_cb(breadcast_recv_cb_fn rx_fn) {
	rx_func = rx_fn;
}

void Broadcast::open() {
	seqno = 0;
}

}  // namespace espmeshmesh
