#include "broadcast2.h"
#include "log.h"
#include <cstring>

static const char *TAG = "espmeshmesh.broadcast2";

namespace espmeshmesh {

void BroadCast2Packet::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast2_header_st));
}

uint8_t Broadcast2::send(const uint8_t *data, uint16_t size, bool port) {
	BroadCast2Packet *pkt = new BroadCast2Packet(nullptr, nullptr);
	pkt->allocClearData(size);
	pkt->broadcastHeader()->protocol = PROTOCOL_BROADCAST_V2;
	pkt->broadcastHeader()->lenght = size;
	pkt->broadcastHeader()->flags = 0;
	pkt->broadcastHeader()->port = port;
	memcpy(pkt->broadcastPayload(), data, size);

    pkt->encryptClearData();
    pkt->fill80211(nullptr, packetbuf->nodeIdPtr());
	uint8_t res = packetbuf->send(pkt);
	if(res == PKT_SEND_ERR) delete pkt;
    return res;
}

void Broadcast2::bindPort(uint16_t port, Broadcast2ReceiveRadioPacketHandler h) {
	LIB_LOGD(TAG, "Broadcast2::bindPort port %d", port);
	Broadcast2BindedPort_t newhandler = {h, port};
	mBindedPorts.push_back(newhandler);
}

void Broadcast2::recv(uint8_t *p, uint16_t size, uint32_t from, int16_t rssi) {
	broadcast2_header_t *header = (broadcast2_header_t *)p;
	LIB_LOGD(TAG, "Broadcast2::recv port %d", header->port);

	for (Broadcast2BindedPort_t port : mBindedPorts) {
		if (port.port == header->port) {
		  port.handler(p + sizeof(broadcast2_header_t), header->lenght, from, rssi);
		}
	}
}

void Broadcast2::open() {
}

}  // namespace espmeshmesh
