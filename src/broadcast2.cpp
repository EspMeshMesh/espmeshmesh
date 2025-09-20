#include "broadcast2.h"
#include "log.h"
#include <cstring>

static const char *TAG = "espmeshmesh.broadcast2";

namespace espmeshmesh {

void BroadCast2Packet::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast2_header_st));
}

uint8_t Broadcast2::send(const uint8_t *data, uint16_t size, uint8_t port) {
	LIB_LOGV(TAG, "Broadcast2::send port %d size %d", port, size);
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

bool Broadcast2::isPortAvailable(uint16_t port) const {
	for (Broadcast2BindedPort_t bindedPort : mBindedPorts) {
		if (bindedPort.port == port) {
			return false;
		}
	}
	return true;
}

bool Broadcast2::bindPort(uint16_t port, Broadcast2ReceiveRadioPacketHandler h) {
	if(!isPortAvailable(port)) {
		LIB_LOGE(TAG, "Broadcast2::bindPort port %d already binded", port);
		return false;
	}
	LIB_LOGV(TAG, "Broadcast2::bindPort port %d", port);
	Broadcast2BindedPort_t newhandler = {h, port};
	mBindedPorts.push_back(newhandler);
	return true;
}

void Broadcast2::unbindPort(uint16_t port) {
	LIB_LOGV(TAG, "Broadcast2::unbindPort ports size %d", mBindedPorts.size());
	for(std::list<Broadcast2BindedPort_t>::iterator it = mBindedPorts.begin(); it != mBindedPorts.end(); it++) {
		if (it->port == port) {
			mBindedPorts.erase(it);
			return;
		}
	}

	LIB_LOGE(TAG, "Broadcast2::unbindPort port %d is not binded", port);
}

void Broadcast2::recv(uint8_t *p, uint16_t size, uint32_t from, int16_t rssi) {
	broadcast2_header_t *header = (broadcast2_header_t *)p;
	LIB_LOGV(TAG, "Broadcast2::recv port %d size %d", header->port, header->lenght);

	for (Broadcast2BindedPort_t port : mBindedPorts) {
		if (port.port == header->port) {
		  port.handler(p + sizeof(broadcast2_header_t), header->lenght, from, rssi);
		}
	}
}

void Broadcast2::open() {
}

}  // namespace espmeshmesh
